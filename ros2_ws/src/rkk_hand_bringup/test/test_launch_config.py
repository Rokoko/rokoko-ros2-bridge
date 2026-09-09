import os

import yaml
from ament_index_python.packages import get_package_share_directory


def _shipped(*parts):
    return os.path.join(get_package_share_directory("rkk_hand_bringup"), *parts)


def test_the_upstream_config_is_installed_and_parses():
    config = yaml.safe_load(open(_shipped("config", "upstream.yaml")))
    assert "solver" in config
    assert config["solver"]["args"]


def test_the_launch_file_is_installed():
    assert os.path.exists(_shipped("launch", "smartglove_hands.launch.py"))
