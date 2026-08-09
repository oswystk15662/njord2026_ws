from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "buoy_obstacle_publisher" / "cardinal_wall_publisher_node.py"


def test_visualization_uses_incremental_track_markers_without_preview_walls():
    source = NODE_SOURCE.read_text(encoding="utf-8")
    assert "self._marker_reset_pending" in source
    assert "self._published_marker_ids" in source
    assert "Marker.DELETE" in source
    assert "preview_virtual_obstacle_wall" not in source
