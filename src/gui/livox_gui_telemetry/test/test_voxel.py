from livox_gui_telemetry.voxel import voxelize


def test_voxelize_keeps_one_per_cell_and_honours_limit():
    assert len(voxelize([(0.1, 0.1, 0.1, 1), (0.2, 0.2, 0.2, 2), (1.1, 0, 0, 3)], 1.0, 10)) == 2
    assert len(voxelize([(float(i), 0, 0, 0) for i in range(10)], .1, 3)) == 3
