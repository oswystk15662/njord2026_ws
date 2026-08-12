from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "buoy_obstacle_publisher" / "cardinal_wall_publisher_node.py"


def test_visualization_uses_incremental_track_markers_without_preview_walls():
    source = NODE_SOURCE.read_text(encoding="utf-8")
    assert "self._marker_reset_pending" in source
    assert "self._published_marker_ids" in source
    assert "Marker.DELETE" in source
    assert "preview_virtual_obstacle_wall" not in source


def test_task1_launches_nearest_active_wall_track_limit_at_four():
    repo_root = PACKAGE_ROOT.parents[2]
    sim_launch = (repo_root / "src" / "sim" / "task1_sim" / "launch" / "task1_sim.launch.py").read_text(
        encoding="utf-8"
    )
    minipc_launch = (repo_root / "src" / "robot" / "launch" / "minipc_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"max_active_wall_tracks": 4' in sim_launch
    assert '"max_active_wall_tracks": 4' in minipc_launch


def test_task4_wall_uses_3d_buoy_positions_and_map_frame():
    repo_root = PACKAGE_ROOT.parents[2]
    minipc_launch = (repo_root / "src" / "robot" / "launch" / "minipc_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'name="task4_buoy_wall_publisher"' in minipc_launch
    assert '"detection_topic": "/buoy_detections_3d"' in minipc_launch
    assert '"map_frame": "map"' in minipc_launch
    assert '"wall_enable_topic": "/task4/buoy_wall_enable"' in minipc_launch
    assert '"retirement_heading_topic": "/task4/gps14_to_gps16_heading"' in minipc_launch
    assert '"allow_cardinal_classes": False' in minipc_launch
    assert '"latest_per_class_only": True' in minipc_launch
    assert '"track_ttl_s": 1.0' in minipc_launch


def test_wall_node_supports_latest_camera_track_expiry_for_task4():
    source = NODE_SOURCE.read_text(encoding="utf-8")
    assert "latest_per_class_only" in source
    assert "track_ttl_s" in source
    assert "last_seen_ns" in source
    assert "_discard_stale_tracks" in source
