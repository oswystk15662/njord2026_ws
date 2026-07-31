import json
import os

from ament_index_python.packages import get_package_share_directory


def _load_config(filename):
    share_dir = get_package_share_directory("robot")
    config_path = os.path.join(share_dir, "config", "livox", filename)
    with open(config_path, "r") as f:
        return json.load(f)


def test_mid360_config_is_valid_json():
    config = _load_config("MID360_config.json")
    assert isinstance(config, dict)


def test_mid360s_config_is_valid_json():
    config = _load_config("MID360S_config.json")
    assert isinstance(config, dict)


def test_mid360_net_info_key():
    config = _load_config("MID360_config.json")
    assert "MID360" in config


def test_mid360s_net_info_key():
    config = _load_config("MID360S_config.json")
    assert "Mid360s" in config
    assert "MID360" not in config


def test_mid360_lidar_type_is_8():
    config = _load_config("MID360_config.json")
    assert config["lidar_summary_info"]["lidar_type"] == 8


def test_mid360s_lidar_type_is_8():
    config = _load_config("MID360S_config.json")
    assert config["lidar_summary_info"]["lidar_type"] == 8


def test_mid360_lidar_configs_present():
    config = _load_config("MID360_config.json")
    lidar_configs = config["lidar_configs"]
    assert isinstance(lidar_configs, list)
    assert len(lidar_configs) > 0
    for entry in lidar_configs:
        assert "ip" in entry
    assert "host_net_info" in config["MID360"]


def test_mid360s_lidar_configs_present():
    config = _load_config("MID360S_config.json")
    lidar_configs = config["lidar_configs"]
    assert isinstance(lidar_configs, list)
    assert len(lidar_configs) > 0
    for entry in lidar_configs:
        assert "ip" in entry
    assert "host_net_info" in config["Mid360s"]
