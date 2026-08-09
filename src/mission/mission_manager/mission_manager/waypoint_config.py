"""Typed validation for the legacy waypoint YAML files.

The migration deliberately reads the existing waypoint_publisher configuration
files rather than changing their coordinates.  Once equivalence tests are in
place these files can move into this package without changing this API.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Mapping

from .task_registry import RegistryError, load_yaml
from .geodesy import HomeDatum, wgs84_to_enu


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    x: float
    y: float
    yaw: float
    name: str
    waypoint_type: str
    gate_pair: object = None


@dataclass(frozen=True)
class Route:
    frame_id: str
    waypoints: tuple[Waypoint, ...]
    stages: Mapping[str, tuple[str, ...]]
    full_sequence_stages: Mapping[str, tuple[str, ...]]
    constraints: Mapping[str, object]

    def stage(self, name: str, *, full_sequence: bool = False) -> tuple[Waypoint, ...]:
        stage_set = self.full_sequence_stages if full_sequence else self.stages
        ids = stage_set.get(name, ())
        by_id = {waypoint.waypoint_id: waypoint for waypoint in self.waypoints}
        return tuple(by_id[item] for item in ids)


class WaypointConfigLoader:
    """Loads one named route config, without sending any Nav2 goal."""

    def __init__(self, home_datum: HomeDatum | None = None) -> None:
        self._home_datum = home_datum

    def load(self, path: Path, route_key: str) -> Route:
        root = load_yaml(path)
        raw = root.get(route_key)
        if not isinstance(raw, dict):
            raise RegistryError(f"route key {route_key!r} is absent from {path}")
        allowed = {
            "frame_id", "publish_rate_hz", "origin", "gps_points", "waypoints", "constraints",
            "scenario", "stages", "full_sequence_stages",
        }
        unknown = set(raw).difference(allowed)
        if unknown:
            raise RegistryError(f"route {route_key!r} has unknown keys: {sorted(unknown)}")
        frame_id = raw.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id:
            raise RegistryError(f"route {route_key!r} requires a non-empty frame_id")
        waypoints = self._waypoints(raw.get("waypoints"), route_key)
        ids = {waypoint.waypoint_id for waypoint in waypoints}
        stages = self._stages(raw.get("stages", {}), ids, route_key, "stages")
        full_stages = self._stages(
            raw.get("full_sequence_stages", {}), ids, route_key, "full_sequence_stages"
        )
        constraints = raw.get("constraints", {})
        if not isinstance(constraints, dict):
            raise RegistryError(f"route {route_key!r} constraints must be a mapping")
        return Route(frame_id, tuple(waypoints), stages, full_stages, constraints)

    def _waypoints(self, raw: object, route_key: str) -> list[Waypoint]:
        if not isinstance(raw, list) or not raw:
            raise RegistryError(f"route {route_key!r} requires non-empty waypoints")
        result = []
        seen = set()
        for index, item in enumerate(raw):
            if not isinstance(item, dict):
                raise RegistryError(f"route {route_key!r} waypoint {index} must be a mapping")
            waypoint_id = item.get("id")
            if isinstance(waypoint_id, bool) or waypoint_id is None:
                raise RegistryError(f"route {route_key!r} waypoint {index} has invalid id")
            waypoint_id = str(waypoint_id)
            if not waypoint_id or waypoint_id in seen:
                raise RegistryError(f"route {route_key!r} waypoint IDs must be unique")
            seen.add(waypoint_id)
            values = {}
            for name in ("yaw",):
                value = item.get(name)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                    raise RegistryError(
                        f"route {route_key!r} waypoint {waypoint_id!r} has non-finite {name}"
                    )
                values[name] = float(value)
            has_map = "x" in item or "y" in item
            has_geographic = "latitude" in item or "longitude" in item
            if has_map == has_geographic:
                raise RegistryError(
                    f"route {route_key!r} waypoint {waypoint_id!r} requires exactly one of x/y or latitude/longitude"
                )
            if has_map:
                for name in ("x", "y"):
                    value = item.get(name)
                    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                        raise RegistryError(
                            f"route {route_key!r} waypoint {waypoint_id!r} has non-finite {name}"
                        )
                    values[name] = float(value)
            else:
                if self._home_datum is None:
                    raise RegistryError(
                        f"route {route_key!r} waypoint {waypoint_id!r} uses latitude/longitude but no home datum is configured"
                    )
                latitude = item.get("latitude")
                longitude = item.get("longitude")
                if (
                    isinstance(latitude, bool) or isinstance(longitude, bool)
                    or not isinstance(latitude, (int, float)) or not isinstance(longitude, (int, float))
                    or not isfinite(latitude) or not isfinite(longitude)
                    or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0
                ):
                    raise RegistryError(
                        f"route {route_key!r} waypoint {waypoint_id!r} has invalid latitude/longitude"
                    )
                values["x"], values["y"] = wgs84_to_enu(latitude, longitude, self._home_datum)
            name = item.get("name", waypoint_id)
            waypoint_type = item.get("type", "waypoint")
            if not isinstance(name, str) or not isinstance(waypoint_type, str):
                raise RegistryError(f"route {route_key!r} waypoint {waypoint_id!r} has invalid metadata")
            result.append(
                Waypoint(waypoint_id, values["x"], values["y"], values["yaw"], name, waypoint_type,
                         item.get("gate_pair"))
            )
        return result

    @staticmethod
    def _stages(
        raw: object, ids: set[str], route_key: str, field: str
    ) -> Mapping[str, tuple[str, ...]]:
        if not isinstance(raw, dict):
            raise RegistryError(f"route {route_key!r} {field} must be a mapping")
        result = {}
        for stage_name, waypoint_ids in raw.items():
            if not isinstance(stage_name, str) or not isinstance(waypoint_ids, list):
                raise RegistryError(f"route {route_key!r} {field} contains an invalid stage")
            converted = tuple(str(item) for item in waypoint_ids)
            missing = set(converted).difference(ids)
            if missing:
                raise RegistryError(
                    f"route {route_key!r} stage {stage_name!r} references missing IDs {sorted(missing)}"
                )
            result[stage_name] = converted
        return result
