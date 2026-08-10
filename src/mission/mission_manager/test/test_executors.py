from mission_manager.executors import (
    ExecutorStatus, StagedDockingExecutor, Task2MppiExecutor, WaypointSequenceExecutor,
)
from mission_manager.mission_manager_node import _RosNavigationClient
from mission_manager.waypoint_config import Route, Waypoint
from geometry_msgs.msg import PoseStamped


class _MarkerPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _MarkerNode:
    class _Clock:
        class _Now:
            @staticmethod
            def to_msg():
                return PoseStamped().header.stamp

        @staticmethod
        def now():
            return _MarkerNode._Clock._Now()

    @staticmethod
    def get_clock():
        return _MarkerNode._Clock()


class FakeNavigation:
    def __init__(self):
        self.sent = []
        self.cancelled = False

    def send(self, poses, accepted, completed):
        self.sent.append((list(poses), accepted, completed))

    def cancel(self):
        self.cancelled = True


def _route():
    waypoints = (
        Waypoint("gate", 0.0, 0.0, 0.0, "gate", "gate"),
        Waypoint("dock", 1.0, 0.0, 0.0, "dock", "dock"),
        Waypoint("exit", 2.0, 0.0, 0.0, "exit", "goal"),
    )
    return Route("map", waypoints, {"stage_1_gate": ("gate",), "stage_1": ("dock",), "stage_2": ("exit",)}, {},
                 {"attempts": 2, "wait_time_s": 1})


def test_sequence_maps_nav_success_to_result():
    nav = FakeNavigation()
    results = []
    executor = WaypointSequenceExecutor(nav)
    executor.start("execution", _route(), lambda *_: None, results.append)
    _, accepted, completed = nav.sent[0]
    accepted(True)
    completed(ExecutorStatus.SUCCEEDED, "ok")
    assert results[0].status == ExecutorStatus.SUCCEEDED


def test_navigation_client_publishes_waypoint_markers():
    client = object.__new__(_RosNavigationClient)
    client._node = _MarkerNode()
    client._frame_id = "map"
    client._marker_publisher = _MarkerPublisher()
    poses = [PoseStamped(), PoseStamped()]
    poses[1].pose.position.x = 2.0

    client._publish_markers(poses)

    route, points = client._marker_publisher.messages[0].markers
    assert route.type == route.LINE_STRIP
    assert [(point.x, point.y) for point in points.points] == [(0.0, 0.0), (2.0, 0.0)]


def test_staged_docking_cancels_wait_timer_and_ignores_late_callback():
    nav = FakeNavigation()
    timers = []
    canceled = []
    results = []
    executor = StagedDockingExecutor(nav, lambda _seconds, callback: timers.append(callback) or callback,
                                     canceled.append)
    executor.start("execution", _route(), lambda *_: None, results.append)
    _, accepted, completed = nav.sent[0]
    accepted(True)
    completed(ExecutorStatus.SUCCEEDED, "gate")
    _, accepted, completed = nav.sent[1]
    accepted(True)
    completed(ExecutorStatus.SUCCEEDED, "dock")
    executor.cancel("execution")
    assert nav.cancelled
    assert canceled
    timers[0]()
    assert results[0].status == ExecutorStatus.CANCELED


def test_staged_docking_does_not_wait_after_an_undocking_stage():
    nav = FakeNavigation()
    timers = []
    results = []
    executor = StagedDockingExecutor(nav, lambda _seconds, callback: timers.append(callback) or callback,
                                     lambda _timer: None)
    executor.start("execution", _route(), lambda *_: None, results.append)

    # Gate then berth.  The berth wait advances to the exit stage.
    nav.sent[0][2](ExecutorStatus.SUCCEEDED, "gate")
    nav.sent[1][2](ExecutorStatus.SUCCEEDED, "dock")
    timers[0]()
    assert len(nav.sent) == 3

    # stage_2 ends at an exit waypoint, so it completes immediately.
    nav.sent[2][2](ExecutorStatus.SUCCEEDED, "exit")
    assert results[0].status == ExecutorStatus.SUCCEEDED


def test_task2_mppi_withdraws_its_gate_before_reporting_canceled():
    enabled = []
    results = []
    executor = Task2MppiExecutor(enabled.append)
    executor.start("execution", _route(), lambda *_: None, results.append)
    executor.cancel("execution")
    assert enabled == [True, False]
    assert results[0].status == ExecutorStatus.CANCELED
