"""Canonical action/service owner for a single NJORD mission execution."""

from __future__ import annotations

import asyncio
import threading
from pathlib import Path
from typing import Callable, Optional, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from njord_interfaces.action import RunTask
from njord_interfaces.msg import ControlState, MissionStatus, TaskInfo
from njord_interfaces.srv import GetMissionStatus, ListTasks, SetControlMode, StartTask, StopTask

from .executors import DummyExecutor, ExecutorStatus, StagedDockingExecutor, WaypointSequenceExecutor
from .state_machine import MissionState, MissionStateMachine, ResultCode, StartRequest
from .task_registry import RegistryError, TaskDefinition, TaskRegistry
from .waypoint_config import Route, Waypoint, WaypointConfigLoader


_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class _RosNavigationClient:
    """Converts typed waypoints to Nav2 messages and keeps one active goal."""

    def __init__(self, node: Node, path_publisher) -> None:
        self._node = node
        self._path_publisher = path_publisher
        self._client = ActionClient(node, NavigateThroughPoses, "/navigate_through_poses")
        self._goal_handle = None
        self._callbacks = None
        self._frame_id = "map"

    def send(self, poses: Sequence[Waypoint], accepted, completed) -> None:
        self._callbacks = (accepted, completed)
        if not self._client.server_is_ready():
            accepted(False)
            return
        messages = [self._pose(waypoint) for waypoint in poses]
        self._publish_path(messages)
        goal = NavigateThroughPoses.Goal()
        goal.poses = messages
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def set_frame_id(self, frame_id: str) -> None:
        self._frame_id = frame_id

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _pose(self, waypoint: Waypoint) -> PoseStamped:
        import math

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose

    def _publish_path(self, poses: list[PoseStamped]) -> None:
        path = NavPath()
        path.header.frame_id = self._frame_id
        path.header.stamp = self._node.get_clock().now().to_msg()
        path.poses = poses
        self._path_publisher.publish(path)

    def _on_goal_response(self, future) -> None:
        callbacks = self._callbacks
        if callbacks is None:
            return
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            callbacks[0](False)
            self._node.get_logger().error(f"Nav2 goal send failed: {exc}")
            return
        callbacks[0](self._goal_handle.accepted)
        if self._goal_handle.accepted:
            self._goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        callbacks = self._callbacks
        self._goal_handle = None
        if callbacks is None:
            return
        try:
            wrapped = future.result()
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                callbacks[1](ExecutorStatus.SUCCEEDED, "Nav2 goal succeeded")
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                callbacks[1](ExecutorStatus.CANCELED, "Nav2 goal canceled")
            else:
                callbacks[1](ExecutorStatus.NAVIGATION_FAILED, f"Nav2 goal status={wrapped.status}")
        except Exception as exc:
            callbacks[1](ExecutorStatus.NAVIGATION_FAILED, f"Nav2 result failed: {exc}")


class MissionManager(Node):
    """Thread-safe adapter around the serialized :class:`MissionStateMachine`."""

    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("registry_file", "")
        self.declare_parameter("active_nav2_profile", "task1")
        self.declare_parameter("auto_permission_timeout_sec", 30.0)
        self._lock = threading.RLock()
        self._cb_group = ReentrantCallbackGroup()
        self._machine = MissionStateMachine()
        self._loader = WaypointConfigLoader()
        self._active_executor = None
        self._active_route: Route | None = None
        self._pending_start = None
        self._auto_permission_deadline_ns: int | None = None
        self._auto_mode_request_sent = False
        self._action_results: dict[str, tuple[ResultCode, str]] = {}
        self._auto_permitted = False
        self._requested_mode = MissionStatus.MODE_MANUAL
        self._effective_source = MissionStatus.SOURCE_ZERO
        self._inhibit_reasons: list[str] = ["control state unavailable"]
        self._last_transition = self.get_clock().now().to_msg()
        self._last_snapshot = self._machine.snapshot
        self._task_ready = False
        self._task_requirements_ready = False
        self._active_control_policy = ""

        registry_path = self._registry_path()
        shares = {"waypoint_publisher": self._waypoint_share_path()}
        self._registry = TaskRegistry.from_file(registry_path, package_shares=shares)

        self._status_pub = self.create_publisher(MissionStatus, "/mission/status", _STATUS_QOS)
        self._plan_pub = self.create_publisher(NavPath, "/task/plan", _STATUS_QOS)
        self._task_ready_pub = self.create_publisher(Bool, "/mission/task_ready", _STATUS_QOS)
        self._task_requirements_ready_pub = self.create_publisher(
            Bool, "/mission/task_requirements_ready", _STATUS_QOS
        )
        self._active_policy_pub = self.create_publisher(
            String, "/mission/active_control_policy", _STATUS_QOS
        )
        self._navigation = _RosNavigationClient(self, self._plan_pub)
        self._control_sub = self.create_subscription(
            ControlState, "/control/state", self._on_control_state, _STATUS_QOS,
            callback_group=self._cb_group,
        )
        self._action = ActionServer(
            self, RunTask, "/mission/run_task", self._execute_action,
            goal_callback=self._goal_callback, cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._start_srv = self.create_service(StartTask, "/mission/start_task", self._on_start)
        self._stop_srv = self.create_service(StopTask, "/mission/stop_task", self._on_stop)
        self._list_srv = self.create_service(ListTasks, "/mission/list_tasks", self._on_list)
        self._status_srv = self.create_service(GetMissionStatus, "/mission/get_status", self._on_status)
        self._set_mode_client = self.create_client(SetControlMode, "/control/set_mode")
        self._timer = self.create_timer(0.1, self._tick, callback_group=self._cb_group)
        self._publish_task_readiness(False, False, "")
        self._publish_status()
        self.get_logger().info(f"Mission Manager ready with registry {registry_path}")

    def destroy_node(self):
        with self._lock:
            if self._active_executor and self._machine.snapshot.execution_id:
                self._active_executor.cleanup(self._machine.snapshot.execution_id)
        return super().destroy_node()

    def _registry_path(self) -> Path:
        configured = str(self.get_parameter("registry_file").value)
        if configured:
            return Path(configured)
        return Path(__file__).resolve().parents[1] / "config" / "task_registry.yaml"

    @staticmethod
    def _waypoint_share_path() -> Path:
        # Source-tree fallback keeps the first migration slice testable before install.
        return Path(__file__).resolve().parents[3] / "navigation" / "path_generator" / "waypoint_publisher"

    def _goal_callback(self, goal_request) -> GoalResponse:
        # Admission happens atomically in execute/start callbacks to preserve request-id semantics.
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_action(self, goal_handle):
        request = StartRequest(
            goal_handle.request.task_id,
            goal_handle.request.request_auto_mode,
            goal_handle.request.dry_run,
            goal_handle.request.request_id,
        )
        with self._lock:
            decision = self._start(request)
        if not decision.accepted:
            goal_handle.abort()
            return self._action_result(decision.result_code, decision.message, decision.execution_id)
        execution_id = decision.execution_id
        while rclpy.ok():
            with self._lock:
                if goal_handle.is_cancel_requested:
                    self._stop(execution_id)
                snapshot = self._machine.snapshot
                result = self._action_results.get(execution_id)
                feedback = self._feedback(snapshot)
            goal_handle.publish_feedback(feedback)
            if result is not None:
                code, message = result
                if code == ResultCode.CANCELED:
                    goal_handle.canceled()
                elif code == ResultCode.SUCCEEDED:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return self._action_result(code, message, execution_id)
            await asyncio.sleep(0.1)
        return self._action_result(ResultCode.INTERNAL_ERROR, "ROS shutdown", execution_id)

    def _on_start(self, request, response):
        with self._lock:
            decision = self._start(StartRequest(
                request.task_id, request.request_auto_mode, request.dry_run, request.request_id
            ))
        response.accepted = decision.accepted
        response.reason_code = self._start_reason_code(decision)
        response.message = decision.message
        response.execution_id = decision.execution_id
        return response

    def _on_stop(self, request, response):
        with self._lock:
            accepted, message = self._stop(request.execution_id)
        response.accepted = accepted
        response.message = message
        return response

    def _on_list(self, _request, response):
        response.tasks = [self._task_info(item) for item in self._registry.all()]
        return response

    def _on_status(self, _request, response):
        response.status = self._status_message()
        return response

    def _start(self, request: StartRequest):
        task = self._registry.get(request.task_id)
        available = bool(task and task.runnable)
        reason = "unknown task" if task is None else task.reason
        decision = self._machine.request_start(request, task_available=available, reason=reason)
        if not decision.accepted or decision.duplicate:
            return decision
        assert task is not None
        if request.request_auto_mode and not task.supports_auto_mode:
            return self._reject_started(decision.execution_id, ResultCode.REJECTED, "task does not support AUTO")
        if request.dry_run and not task.supports_dry_run:
            return self._reject_started(decision.execution_id, ResultCode.REJECTED, "task does not support dry run")
        active_profile = str(self.get_parameter("active_nav2_profile").value)
        if task.nav2_profile != active_profile:
            return self._reject_started(
                decision.execution_id,
                ResultCode.CONFIGURATION_FAILED,
                f"task requires Nav2 profile {task.nav2_profile}; active profile is {active_profile}",
            )
        try:
            route = self._load_route(task)
        except RegistryError as exc:
            return self._reject_started(decision.execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))
        if route.frame_id != task.frame_id:
            return self._reject_started(
                decision.execution_id, ResultCode.CONFIGURATION_FAILED,
                f"registry frame {task.frame_id} differs from route frame {route.frame_id}",
            )
        self._active_route = route
        self._navigation.set_frame_id(route.frame_id)
        # Task 1's only required runtime input is the route just validated
        # above.  Publish it before requesting AUTO so AUTO admission does not
        # depend on a Nav2 goal that has not been sent yet.
        self._publish_task_readiness(True, True, task.control_policy)
        self._machine.transition(MissionState.CONFIGURING, execution_id=decision.execution_id,
                                 stage="configure", message="task route validated")
        if request.dry_run:
            self._complete(decision.execution_id, ResultCode.SUCCEEDED, "dry run validated route and profile")
            return decision
        if request.request_auto_mode:
            # Start Nav2 while command selection remains zero/MANUAL.  Waiting
            # for a fresh Nav2 command before starting the executor would form a
            # deadlock with SafetySupervisor's freshness interlock.
            self._machine.transition(
                MissionState.WAITING_FOR_AUTO_PERMISSION,
                execution_id=decision.execution_id,
                stage="permission",
                message="waiting for AUTO permission before starting Nav2",
            )
            self._pending_start = (decision.execution_id, task)
            self._auto_permission_deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("auto_permission_timeout_sec").value) * 1e9)
            )
            self._auto_mode_request_sent = False
            self._request_auto_mode()
        else:
            self._begin_executor(decision.execution_id, task)
        self._publish_status()
        return decision

    def _reject_started(self, execution_id: str, code: ResultCode, message: str):
        self._machine.finish(execution_id, code, message)
        self._action_results[execution_id] = (code, message)
        self._publish_status()
        self._machine.reset_to_idle(execution_id)
        self._publish_status()
        return type("Decision", (), {"accepted": False, "result_code": code, "message": message,
                                      "execution_id": execution_id, "duplicate": False})()

    def _load_route(self, task: TaskDefinition) -> Route:
        root = self._waypoint_share_path() if task.route_package == "waypoint_publisher" else self._registry_path().parent
        return self._loader.load(root / task.route, task.route_key)

    def _begin_executor(self, execution_id: str, task: TaskDefinition) -> None:
        if not self._machine.is_current(execution_id) or self._active_route is None:
            return
        self._pending_start = None
        self._machine.transition(MissionState.RUNNING, execution_id=execution_id,
                                 stage="starting", message="starting task executor")
        if task.executor == "waypoint_sequence":
            executor = WaypointSequenceExecutor(self._navigation)
            executor.start(execution_id, self._active_route, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        elif task.executor == "staged_docking":
            executor = StagedDockingExecutor(
                self._navigation, self._create_wait_timer, self.destroy_timer
            )
            executor.start(execution_id, self._active_route, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        elif task.executor == "dummy":
            executor = DummyExecutor()
            executor.start(execution_id, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        self._active_executor = executor

    def _request_auto_mode(self) -> None:
        """Request AUTO through the sole typed mode owner, never by topic spoofing."""
        if not self._set_mode_client.service_is_ready():
            return
        self._auto_mode_request_sent = True
        request = SetControlMode.Request()
        request.requested_mode = SetControlMode.Request.MODE_AUTO
        request.request_id = f"mission-{self._machine.snapshot.execution_id}"
        future = self._set_mode_client.call_async(request)

        def completed(result_future) -> None:
            try:
                result = result_future.result()
                if not result.accepted:
                    self.get_logger().error(f"AUTO mode request rejected: {result.message}")
            except Exception as exc:
                self.get_logger().error(f"AUTO mode request failed: {exc}")

        future.add_done_callback(completed)

    def _create_wait_timer(self, seconds: float, callback: Callable[[], None]):
        return self.create_timer(seconds if seconds > 0 else 0.001, callback, callback_group=self._cb_group)

    def _feedback_update(self, execution_id: str):
        def update(stage: str, progress: float, message: str) -> None:
            with self._lock:
                if self._machine.update_progress(execution_id, stage, progress, message):
                    self._publish_status()
        return update

    def _executor_complete(self, execution_id: str):
        def complete(result) -> None:
            with self._lock:
                mapping = {
                    ExecutorStatus.SUCCEEDED: ResultCode.SUCCEEDED,
                    ExecutorStatus.CANCELED: ResultCode.CANCELED,
                    ExecutorStatus.REJECTED: ResultCode.NAVIGATION_FAILED,
                    ExecutorStatus.NAVIGATION_FAILED: ResultCode.NAVIGATION_FAILED,
                    ExecutorStatus.INTERNAL_ERROR: ResultCode.INTERNAL_ERROR,
                }
                self._complete(execution_id, mapping[result.status], result.message)
        return complete

    def _stop(self, execution_id: str) -> tuple[bool, str]:
        if not self._machine.is_current(execution_id) or not self._machine.active:
            return False, "execution is not active"
        self._machine.begin_cancel(execution_id)
        self._publish_task_readiness(False, False, "")
        self._pending_start = None
        self._auto_permission_deadline_ns = None
        self._auto_mode_request_sent = False
        if self._active_executor:
            self._active_executor.cancel(execution_id)
        else:
            self._complete(execution_id, ResultCode.CANCELED, "task canceled before navigation started")
        self._publish_status()
        return True, "cancellation requested"

    def _complete(self, execution_id: str, code: ResultCode, message: str) -> None:
        if not self._machine.finish(execution_id, code, message):
            return
        self._action_results[execution_id] = (code, message)
        self._pending_start = None
        self._auto_permission_deadline_ns = None
        self._auto_mode_request_sent = False
        self._active_executor = None
        self._publish_status()
        self._publish_task_readiness(False, False, "")
        self._machine.reset_to_idle(execution_id)
        self._publish_status()

    def _on_control_state(self, message: ControlState) -> None:
        with self._lock:
            self._auto_permitted = message.auto_permitted
            self._requested_mode = message.requested_mode
            self._effective_source = message.effective_source
            self._inhibit_reasons = list(message.inhibit_reasons)

    def _tick(self) -> None:
        with self._lock:
            snapshot = self._machine.snapshot
            if snapshot.state == MissionState.WAITING_FOR_AUTO_PERMISSION:
                if not self._auto_mode_request_sent:
                    self._request_auto_mode()
                if self._auto_permitted:
                    pending = self._pending_start
                    self._auto_permission_deadline_ns = None
                    if pending is not None:
                        self._begin_executor(*pending)
                elif (
                    self._auto_permission_deadline_ns is not None
                    and self.get_clock().now().nanoseconds >= self._auto_permission_deadline_ns
                ):
                    if self._active_executor:
                        self._active_executor.cleanup(snapshot.execution_id)
                    self._complete(
                        snapshot.execution_id,
                        ResultCode.SAFETY_INHIBITED,
                        "AUTO permission timed out: " + "; ".join(self._inhibit_reasons),
                    )
            self._publish_status()

    def _publish_status(self) -> None:
        if self._machine.snapshot != self._last_snapshot:
            self._last_transition = self.get_clock().now().to_msg()
            self._last_snapshot = self._machine.snapshot
        self._status_pub.publish(self._status_message())

    def _publish_task_readiness(
        self, task_ready: bool, requirements_ready: bool, control_policy: str
    ) -> None:
        """Publish the mission-owned safety inputs with latched safe defaults."""
        self._task_ready = task_ready
        self._task_requirements_ready = requirements_ready
        self._active_control_policy = control_policy
        task_message = Bool()
        task_message.data = task_ready
        requirements_message = Bool()
        requirements_message.data = requirements_ready
        policy_message = String()
        policy_message.data = control_policy
        self._task_ready_pub.publish(task_message)
        self._task_requirements_ready_pub.publish(requirements_message)
        self._active_policy_pub.publish(policy_message)

    def _status_message(self) -> MissionStatus:
        snapshot = self._machine.snapshot
        message = MissionStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = int(snapshot.state)
        message.task_id = snapshot.task_id
        message.execution_id = snapshot.execution_id
        message.stage = snapshot.stage
        message.progress = snapshot.progress
        message.message = snapshot.message
        message.requested_control_mode = self._requested_mode
        message.effective_control_source = self._effective_source
        message.auto_permitted = self._auto_permitted
        message.inhibit_reasons = self._inhibit_reasons
        message.last_transition_time = self._last_transition
        return message

    def _feedback(self, snapshot) -> RunTask.Feedback:
        message = RunTask.Feedback()
        message.state = int(snapshot.state)
        message.task_id = snapshot.task_id
        message.execution_id = snapshot.execution_id
        message.stage = snapshot.stage
        message.progress = snapshot.progress
        message.message = snapshot.message
        message.inhibit_reasons = self._inhibit_reasons
        return message

    def _action_result(self, code: ResultCode, message: str, execution_id: str) -> RunTask.Result:
        result = RunTask.Result()
        result.result_code = int(code)
        result.message = message
        result.execution_id = execution_id
        result.final_inhibit_reasons = self._inhibit_reasons
        return result

    @staticmethod
    def _task_info(task: TaskDefinition) -> TaskInfo:
        message = TaskInfo()
        availability = {
            "available": TaskInfo.AVAILABLE,
            "experimental": TaskInfo.EXPERIMENTAL,
            "not_implemented": TaskInfo.NOT_IMPLEMENTED,
            "disabled": TaskInfo.DISABLED,
        }
        message.task_id = task.task_id
        message.display_name = task.display_name
        message.availability = availability[task.availability]
        message.reason = task.reason
        message.supports_dry_run = task.supports_dry_run
        message.supports_auto_mode = task.supports_auto_mode
        return message

    @staticmethod
    def _start_reason_code(decision) -> int:
        if decision.accepted:
            return StartTask.Response.DUPLICATE_REQUEST if decision.duplicate else StartTask.Response.ACCEPTED
        if decision.result_code == ResultCode.CONFIGURATION_FAILED:
            return StartTask.Response.CONFIGURATION_FAILED
        return StartTask.Response.TASK_UNAVAILABLE


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = MissionManager()
        rclpy.spin(node)
    except RegistryError as exc:
        rclpy.logging.get_logger("mission_manager").fatal(f"mission registry invalid: {exc}")
        raise
    finally:
        rclpy.shutdown()
