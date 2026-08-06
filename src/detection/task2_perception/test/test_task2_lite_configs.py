"""Regression checks for the Task 2 CPU-light perception profile."""

from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"


def _params(name):
    return yaml.safe_load((CONFIG_DIR / name).read_text())["/**"]["ros__parameters"]


def test_cloud_filter_profile_reduces_work_without_debug_changes():
    params = yaml.safe_load(
        (CONFIG_DIR / "task2_perception_params.yaml").read_text())
    cloud = params["task2_cloud_filter"]["ros__parameters"]
    assert cloud["max_range_m"] == 40.0
    assert cloud["process_rate_hz"] == 5.0
    assert cloud["use_water_plane_ransac"] is False
    assert cloud["publish_debug"] is False


def test_task2_lite_pipeline_uses_latest_compact_clouds():
    preprocessing = _params("task2_preprocessing_lite.yaml")
    segmentation = _params("task2_segmentation_lite.yaml")
    assert preprocessing["voxel_leaf_x"] == 0.15
    assert preprocessing["accumulate_frames"] is False
    assert preprocessing["input_queue_depth"] == 1
    assert segmentation["input_queue_depth"] == 1
    assert segmentation["ransac_max_iterations"] == 80
    assert segmentation["min_cluster_size"] == 30
