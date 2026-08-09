"""Regression checks for the low-load rear-camera default profile."""

from pathlib import Path

import yaml


_ROBOT_DIR = Path(__file__).resolve().parents[1]


def _back_camera_params():
    config_path = _ROBOT_DIR / "config" / "back_cam.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    return config["/back_cam/back_cam"]["ros__parameters"]


def test_back_camera_default_is_a_low_load_gui_profile():
    params = _back_camera_params()

    assert (params["image_width"], params["image_height"]) == (320, 240)
    assert params["framerate"] == 5.0
    # 320x240 at 5 FPS is 24 times fewer captured pixels per second than the
    # previous 640x480 at 30 FPS default.
    assert params["image_width"] * params["image_height"] * params["framerate"] == 384_000


def test_back_camera_manual_exposure_remains_preconfigured_at_launch():
    params = _back_camera_params()

    assert params["autoexposure"] is False
    assert params["exposure"] == 1
