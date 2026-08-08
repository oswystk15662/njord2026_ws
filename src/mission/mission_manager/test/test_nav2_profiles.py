from pathlib import Path

import pytest
import yaml

from mission_manager.nav2_profiles import (
    Nav2ProfileApplicationManager,
    Nav2ProfileCatalog,
    Nav2ProfileError,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parents[1]
CATALOG_FILE = PACKAGE_ROOT / "config/nav2_profiles/profiles.yaml"
ROBOT_CONFIG = SOURCE_ROOT / "robot/config"


@pytest.mark.parametrize("profile,legacy_stem", [
    ("task1", "nav2_params"),
    ("task2", "nav2_params_task2"),
    ("task3", "nav2_params_task3"),
])
@pytest.mark.parametrize("distro", ["humble", "jazzy"])
def test_resolved_profiles_are_lossless(profile, legacy_stem, distro):
    catalog = Nav2ProfileCatalog.from_file(CATALOG_FILE)
    expected = yaml.safe_load(
        (ROBOT_CONFIG / f"{legacy_stem}_{distro}.yaml").read_text(encoding="utf-8")
    )
    assert catalog.resolve(profile, distro) == expected


def test_runtime_profile_change_is_rejected_without_partial_apply():
    catalog = Nav2ProfileCatalog.from_file(CATALOG_FILE)
    manager = Nav2ProfileApplicationManager(catalog, "task1")
    assert manager.plan("task1", effective_output_is_zero=False).accepted
    decision = manager.plan("task3", effective_output_is_zero=True)
    assert not decision.accepted
    assert decision.requires_restart
    assert "lifecycle restart" in decision.message
    assert manager.active_profile == "task1"


def test_unknown_profile_and_distribution_fail_closed():
    catalog = Nav2ProfileCatalog.from_file(CATALOG_FILE)
    manager = Nav2ProfileApplicationManager(catalog, "task1")
    with pytest.raises(Nav2ProfileError, match="unknown Nav2 profile"):
        manager.plan("task4", effective_output_is_zero=True)
    with pytest.raises(Nav2ProfileError, match="unsupported ROS distribution"):
        catalog.resolve("task1", "rolling")
