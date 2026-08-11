"""Regression checks for the Task 2 CPU-light perception profile."""

from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"


def _params():
    return yaml.safe_load((CONFIG_DIR / "task2_params.yaml").read_text())


def test_cloud_filter_profile_reduces_work_without_debug_changes():
    params = _params()
    cloud = params["task2_cloud_filter"]["ros__parameters"]
    assert cloud["max_range_m"] == 40.0
    assert cloud["process_rate_hz"] == 5.0
    assert cloud["use_water_plane_ransac"] is False
    assert cloud["publish_debug"] is False


def test_task2_lite_pipeline_uses_latest_compact_clouds():
    params = _params()
    preprocessing = params["preprocessing_node"]["ros__parameters"]
    ground = params["ground_remover_node"]["ros__parameters"]
    cluster = params["cluster_node"]["ros__parameters"]
    tracker = params["ship_tracker_node"]["ros__parameters"]
    assert preprocessing["voxel_leaf_x"] == 0.15
    assert preprocessing["accumulate_frames"] is False
    assert preprocessing["input_queue_depth"] == 1
    assert ground["input_queue_depth"] == 1
    assert ground["ransac_max_iterations"] == 80
    assert cluster["input_queue_depth"] == 1
    assert cluster["min_cluster_size"] == 15
    assert cluster["min_cluster_height"] == 0.15
    assert tracker["straight_hits_to_confirm"] == 5
    selector = params["opponent_selector"]["ros__parameters"]
    assert selector["straight_min_hit_count"] == 5
    assert selector["straight_max_velocity_stddev_mps"] == 0.5
    assert selector["straight_continue_after_loss"] is True
    assert selector["min_absolute_speed_knots"] == 0.3
    assert selector["max_absolute_speed_knots"] == 6.0
    assert selector["straight_coast_timeout_sec"] == 2.0
    assert selector["corridor_enabled"] is True
    assert selector["corridor_start_offset_m"] == 5.0
    assert selector["corridor_end_margin_m"] == 5.0
    assert selector["corridor_half_width_m"] == 20.0
    buoy = params["task2_buoy_selector"]["ros__parameters"]
    assert buoy["expected_lateral_offset_m"] == 2.5
    assert buoy["lateral_tolerance_m"] == 1.0
    assert buoy["stationary_speed_max_mps"] == 0.35
    assert buoy["min_point_count"] == 5
    assert buoy["max_length_m"] == 1.2
    assert buoy["max_width_m"] == 1.2


def test_single_task2_yaml_contains_every_opponent_pipeline_node():
    params = _params()
    assert {
        "task2_cloud_filter", "preprocessing_node", "ground_remover_node",
        "cluster_node", "ship_tracker_node", "opponent_selector", "task2_buoy_selector",
        "task2_safety_cloud_filter", "task2_safety_cloud_gate",
        "task2_odometry_selector", "planner_node",
    }.issubset(params)
    assert params["ship_tracker_node"]["ros__parameters"]["motion_mode"] == "straight_line"
    assert params["task2_safety_cloud_filter"]["ros__parameters"]["output_topic"] == "/task2/safety_points"
    assert params["planner_node"]["ros__parameters"]["mppi.opponent_loa"] == 1.8
    assert params["planner_node"]["ros__parameters"]["buoy_detection_array_topic"] == "/task2/buoy_detections"
    assert params["task2_odometry_selector"]["ros__parameters"]["candidate_topics"] == [
        "/odometry/filtered/local", "/odom", "/odometry/filtered/global"]
