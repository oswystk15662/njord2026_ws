"""Own the one replaceable Nav2 runtime process on the miniPC."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from njord_interfaces.action import ConfigureSystem
from njord_interfaces.msg import Nav2RuntimeStatus


_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RuntimeManager(Node):
    def __init__(self):
        super().__init__("runtime_manager")
        self.declare_parameter("sigint_timeout_sec", 10.0)
        self.declare_parameter("sigterm_timeout_sec", 5.0)
        self.declare_parameter("nav2_ready_timeout_sec", 30.0)
        self.declare_parameter("nav2_ready_poll_sec", 0.5)
        self._process = None
        self._profile = ""
        self._lock = threading.Lock()
        self._pub = self.create_publisher(Nav2RuntimeStatus, "/runtime/nav2/status", _QOS)
        self._action = ActionServer(
            self,
            ConfigureSystem,
            "/system/configure",
            self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self._publish(Nav2RuntimeStatus.STOPPED, "")

    def _goal(self, goal):
        return GoalResponse.ACCEPT if goal.parameter == "nav2_profile" and goal.value in {"task1", "task2", "task3"} else GoalResponse.REJECT

    @staticmethod
    def _cancel(_goal):
        return CancelResponse.ACCEPT

    def _publish(self, state, message):
        message_out = Nav2RuntimeStatus()
        message_out.stamp = self.get_clock().now().to_msg()
        message_out.state, message_out.active_profile, message_out.message = state, self._profile, message
        self._pub.publish(message_out)

    def _stop(self):
        process = self._process
        if not process or process.poll() is not None:
            self._process = None
            return
        self._publish(Nav2RuntimeStatus.STOPPING, "stopping previous Nav2 runtime")
        for sig, timeout in ((signal.SIGINT, self.get_parameter("sigint_timeout_sec").value),
                             (signal.SIGTERM, self.get_parameter("sigterm_timeout_sec").value),
                             (signal.SIGKILL, 0)):
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=timeout or None)
                break
            except subprocess.TimeoutExpired:
                continue
        self._process = None

    def _wait_for_bt_navigator_active(self) -> None:
        """Block until Nav2 has activated the action server used by Task 1/3.

        Discovering ``/navigate_through_poses`` is insufficient: bt_navigator
        exposes its action endpoints before its lifecycle state is ACTIVE and
        rejects goals in that interval.  Querying lifecycle through the ROS 2
        CLI uses an independent DDS participant, so it remains safe while this
        action server's execute callback is waiting.
        """
        timeout = float(self.get_parameter("nav2_ready_timeout_sec").value)
        poll = float(self.get_parameter("nav2_ready_poll_sec").value)
        if timeout <= 0.0 or poll <= 0.0:
            raise ValueError("nav2_ready_timeout_sec and nav2_ready_poll_sec must be positive")

        deadline = time.monotonic() + timeout
        last_output = "lifecycle state is not available yet"
        while time.monotonic() < deadline:
            if self._process is None or self._process.poll() is not None:
                raise RuntimeError("Nav2 runtime exited before bt_navigator became active")
            try:
                result = subprocess.run(
                    ["ros2", "lifecycle", "get", "/bt_navigator"],
                    capture_output=True,
                    text=True,
                    timeout=poll,
                    check=False,
                )
                output = (result.stdout + result.stderr).strip()
                if result.returncode == 0 and output.lower().startswith("active"):
                    self.get_logger().info("bt_navigator lifecycle state is active")
                    return
                if output:
                    last_output = output
            except subprocess.TimeoutExpired:
                last_output = "timed out querying /bt_navigator lifecycle state"
            time.sleep(poll)

        raise RuntimeError(
            f"timed out after {timeout:g}s waiting for /bt_navigator to become active: {last_output}"
        )

    def shutdown(self):
        """Stop the Nav2 process group before this manager exits.

        ``task_runtime.launch.py`` is intentionally started in its own process
        session so profile switches can terminate the complete Nav2 tree.  It
        must therefore be stopped explicitly when the enclosing miniPC
        bringup receives Ctrl-C; otherwise it is reparented to PID 1.
        """
        with self._lock:
            self._stop()
            self._profile = ""

    def _execute(self, handle):
        profile = handle.request.value
        with self._lock:
            try:
                if self._profile != profile:
                    self._stop()
                    self._profile = profile
                    self._publish(Nav2RuntimeStatus.STARTING, "starting Nav2 runtime")
                    self._process = subprocess.Popen(
                        ["ros2", "launch", "robot", "task_runtime.launch.py", f"profile:={profile}"],
                        start_new_session=True)
                    if profile in {"task1", "task3"}:
                        self._wait_for_bt_navigator_active()
                self._publish(Nav2RuntimeStatus.READY, "Nav2 runtime started")
                result = ConfigureSystem.Result(success=True, message=f"active Nav2 profile: {profile}")
                handle.succeed()
                return result
            except Exception as error:
                self._stop()
                self._profile = ""
                self._publish(Nav2RuntimeStatus.FAILED, str(error))
                handle.abort()
                return ConfigureSystem.Result(success=False, message=str(error))


def main():
    rclpy.init()
    node = RuntimeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
