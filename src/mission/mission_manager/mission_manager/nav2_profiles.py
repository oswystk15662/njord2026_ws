"""Typed, startup-only Nav2 profile resolution and admission policy."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class Nav2ProfileError(ValueError):
    """Raised when the profile catalog or a resolved profile is invalid."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise Nav2ProfileError(f"duplicate YAML key {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


@dataclass(frozen=True)
class Nav2Topology:
    controller_plugin: str
    planner_plugin: str
    global_costmap_plugins: tuple[str, ...]
    lifecycle_nodes: tuple[str, ...]
    collision_monitor: bool


@dataclass(frozen=True)
class Nav2Profile:
    name: str
    task_overlay: Path
    distro_overlays: dict[str, Path]
    topology: Nav2Topology
    required_actions: tuple[str, ...]
    required_topics: tuple[str, ...]


@dataclass(frozen=True)
class ProfileApplyDecision:
    accepted: bool
    requires_restart: bool
    message: str


class Nav2ProfileCatalog:
    """Resolves lossless base + task + ROS-distro parameter overlays."""

    def __init__(
        self,
        root: Path,
        base_file: Path,
        supported_distros: tuple[str, ...],
        profiles: dict[str, Nav2Profile],
    ) -> None:
        self.root = root
        self.base_file = base_file
        self.supported_distros = supported_distros
        self._profiles = profiles

    @classmethod
    def from_file(cls, path: Path | str) -> "Nav2ProfileCatalog":
        path = Path(path)
        data = _load_mapping(path)
        allowed = {"schema_version", "apply_mode", "base", "supported_distros", "profiles"}
        _reject_unknown(data, allowed, "catalog")
        if data.get("schema_version") != 1:
            raise Nav2ProfileError("Nav2 profile schema_version must be 1")
        if data.get("apply_mode") != "startup_only":
            raise Nav2ProfileError("only safe apply_mode 'startup_only' is supported")
        distros = _strings(data.get("supported_distros"), "supported_distros")
        root = path.parent
        base = _relative_file(root, data.get("base"), "base")
        raw_profiles = data.get("profiles")
        if not isinstance(raw_profiles, dict) or not raw_profiles:
            raise Nav2ProfileError("profiles must be a non-empty mapping")
        profiles = {
            str(name): _parse_profile(str(name), value, root, distros)
            for name, value in raw_profiles.items()
        }
        catalog = cls(root, base, distros, profiles)
        for profile in profiles:
            for distro in distros:
                catalog.resolve(profile, distro)
        return catalog

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self._profiles)

    def profile(self, name: str) -> Nav2Profile:
        try:
            return self._profiles[name]
        except KeyError as exc:
            raise Nav2ProfileError(f"unknown Nav2 profile {name!r}") from exc

    def resolve(self, name: str, distro: str) -> dict[str, Any]:
        if distro not in self.supported_distros:
            raise Nav2ProfileError(f"unsupported ROS distribution {distro!r}")
        profile = self.profile(name)
        params = _deep_merge({}, _load_mapping(self.base_file))
        params = _deep_merge(params, _load_mapping(profile.task_overlay))
        distro_overlay = profile.distro_overlays.get(distro)
        if distro_overlay is not None:
            params = _deep_merge(params, _load_mapping(distro_overlay))
        _validate_topology(profile, params)
        return params


class Nav2ProfileApplicationManager:
    """Admission boundary that never performs an unsafe live Nav2 mutation."""

    def __init__(self, catalog: Nav2ProfileCatalog, active_profile: str) -> None:
        catalog.profile(active_profile)
        self.catalog = catalog
        self.active_profile = active_profile

    def plan(self, requested_profile: str, effective_output_is_zero: bool) -> ProfileApplyDecision:
        self.catalog.profile(requested_profile)
        if requested_profile == self.active_profile:
            return ProfileApplyDecision(True, False, "resident Nav2 profile already active")
        suffix = ""
        if not effective_output_is_zero:
            suffix = "; effective command output is not ZERO"
        return ProfileApplyDecision(
            False,
            True,
            f"Nav2 profile {requested_profile} requires a lifecycle restart from "
            f"{self.active_profile}; runtime profile apply is unavailable{suffix}",
        )


def _parse_profile(
    name: str, raw: Any, root: Path, supported_distros: tuple[str, ...]
) -> Nav2Profile:
    if not isinstance(raw, dict):
        raise Nav2ProfileError(f"profile {name!r} must be a mapping")
    _reject_unknown(
        raw,
        {"task_overlay", "distro_overlays", "topology", "required_actions", "required_topics"},
        f"profile {name}",
    )
    overlays = raw.get("distro_overlays", {})
    if not isinstance(overlays, dict):
        raise Nav2ProfileError(f"profile {name} distro_overlays must be a mapping")
    unknown_distros = set(overlays) - set(supported_distros)
    if unknown_distros:
        raise Nav2ProfileError(f"profile {name} has unknown distro overlays {sorted(unknown_distros)}")
    topology = raw.get("topology")
    if not isinstance(topology, dict):
        raise Nav2ProfileError(f"profile {name} topology must be a mapping")
    _reject_unknown(
        topology,
        {"controller_plugin", "planner_plugin", "global_costmap_plugins", "lifecycle_nodes", "collision_monitor"},
        f"profile {name} topology",
    )
    collision_monitor = topology.get("collision_monitor")
    if not isinstance(collision_monitor, bool):
        raise Nav2ProfileError(f"profile {name} collision_monitor must be boolean")
    return Nav2Profile(
        name=name,
        task_overlay=_relative_file(root, raw.get("task_overlay"), f"profile {name} task_overlay"),
        distro_overlays={
            str(distro): _relative_file(root, value, f"profile {name} distro overlay")
            for distro, value in overlays.items()
        },
        topology=Nav2Topology(
            controller_plugin=_required_string(topology, "controller_plugin", name),
            planner_plugin=_required_string(topology, "planner_plugin", name),
            global_costmap_plugins=_strings(topology.get("global_costmap_plugins"), "global_costmap_plugins"),
            lifecycle_nodes=_strings(topology.get("lifecycle_nodes"), "lifecycle_nodes"),
            collision_monitor=collision_monitor,
        ),
        required_actions=_strings(raw.get("required_actions"), "required_actions"),
        required_topics=_strings(raw.get("required_topics"), "required_topics"),
    )


def _validate_topology(profile: Nav2Profile, params: dict[str, Any]) -> None:
    actual_controller = _path(params, "controller_server.ros__parameters.FollowPath.plugin")
    actual_planner = _path(params, "planner_server.ros__parameters.GridBased.plugin")
    actual_plugins = _path(params, "global_costmap.global_costmap.ros__parameters.plugins")
    actual_lifecycle = _path(params, "lifecycle_manager.ros__parameters.node_names")
    checks = (
        (_canonical_plugin(actual_controller), profile.topology.controller_plugin, "controller plugin"),
        (_canonical_plugin(actual_planner), profile.topology.planner_plugin, "planner plugin"),
        (tuple(actual_plugins), profile.topology.global_costmap_plugins, "global costmap plugins"),
        (tuple(actual_lifecycle), profile.topology.lifecycle_nodes, "lifecycle nodes"),
    )
    for actual, expected, label in checks:
        if actual != expected:
            raise Nav2ProfileError(
                f"profile {profile.name} {label} mismatch: expected {expected!r}, got {actual!r}"
            )
    has_collision_monitor = "collision_monitor" in params and "collision_monitor" in actual_lifecycle
    if has_collision_monitor != profile.topology.collision_monitor:
        raise Nav2ProfileError(f"profile {profile.name} collision-monitor topology mismatch")


def _load_mapping(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            value = yaml.load(stream, Loader=_UniqueKeyLoader)
    except OSError as exc:
        raise Nav2ProfileError(f"cannot read Nav2 profile file {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise Nav2ProfileError(f"{path} must contain a YAML mapping")
    return value


def _deep_merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _path(mapping: dict[str, Any], path: str) -> Any:
    value: Any = mapping
    for component in path.split("."):
        if not isinstance(value, dict) or component not in value:
            raise Nav2ProfileError(f"resolved parameters missing {path}")
        value = value[component]
    return value


def _canonical_plugin(value: Any) -> str:
    if not isinstance(value, str):
        raise Nav2ProfileError("plugin identifier must be a string")
    return value.replace("/", "::")


def _required_string(mapping: dict[str, Any], key: str, context: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise Nav2ProfileError(f"{context} {key} must be a non-empty string")
    return value


def _strings(value: Any, context: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value or not all(isinstance(item, str) and item for item in value):
        raise Nav2ProfileError(f"{context} must be a non-empty string list")
    if len(set(value)) != len(value):
        raise Nav2ProfileError(f"{context} contains duplicates")
    return tuple(value)


def _relative_file(root: Path, value: Any, context: str) -> Path:
    if not isinstance(value, str) or not value:
        raise Nav2ProfileError(f"{context} must be a relative file path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise Nav2ProfileError(f"{context} must stay inside the catalog directory")
    resolved = root / path
    if not resolved.is_file():
        raise Nav2ProfileError(f"{context} does not exist: {resolved}")
    return resolved


def _reject_unknown(mapping: dict[str, Any], allowed: set[str], context: str) -> None:
    unknown = set(mapping) - allowed
    if unknown:
        raise Nav2ProfileError(f"{context} has unknown keys {sorted(unknown)}")
