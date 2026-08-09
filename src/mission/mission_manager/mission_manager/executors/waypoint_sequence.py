"""One NavigateThroughPoses goal for Task 1 and Task 2 routes."""

from __future__ import annotations

from typing import Callable

from .base import CompletionCallback, ExecutorStatus, FeedbackCallback, TaskExecutor
from ..waypoint_config import Route


class WaypointSequenceExecutor(TaskExecutor):
    def start(
        self, execution_id: str, route: Route, feedback: FeedbackCallback, complete: CompletionCallback
    ) -> None:
        if not route.waypoints:
            complete_status = ExecutorStatus.INTERNAL_ERROR
            complete_result = "route has no waypoints"
            self._begin(execution_id, feedback, complete)
            self._finish(complete_status, complete_result)
            return
        self._begin(execution_id, feedback, complete)
        self._report("navigate", 0.0, f"sending {len(route.waypoints)} planned waypoints")
        expected_id = execution_id

        def accepted(ok: bool) -> None:
            if expected_id != self.execution_id or self._finished:
                return
            if not ok:
                self._finish(ExecutorStatus.REJECTED, "Nav2 rejected waypoint goal")
            else:
                self._report("navigate", 0.05, "Nav2 accepted waypoint goal")

        def completed(status: ExecutorStatus, message: str) -> None:
            if expected_id != self.execution_id or self._finished:
                return
            if self._cancel_requested:
                self._finish(ExecutorStatus.CANCELED, "waypoint goal canceled")
            elif status == ExecutorStatus.SUCCEEDED:
                self._report("navigate", 1.0, "waypoint route complete")
                self._finish(status, message or "waypoint route complete")
            else:
                self._finish(status, message or "waypoint navigation failed")

        self._navigation.send(route.waypoints, accepted, completed)
