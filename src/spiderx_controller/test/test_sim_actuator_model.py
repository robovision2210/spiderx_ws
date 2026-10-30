"""Simulated actuator model (S2 follow-up): an explicit gz_ros2_control gain for the plugin node.

Offline: reads the installed configuration and expands the xacro; no Gazebo, no ROS graph. The
patched plugin that applies the gain is tested in its own package
(third_party/gz_ros2_control_overlay, gtest test_parameter_loading).
"""
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

from spiderx_controller import config_check

DESCRIPTION = get_package_share_directory('spiderx_description')
CONTROLLER = get_package_share_directory('spiderx_controller')
PLANT = os.path.join(DESCRIPTION, 'config', 'gz_ros2_control_plant.yaml')
CONTROLLERS = os.path.join(CONTROLLER, 'config', 'spiderx_ros2_controllers.yaml')
GAIN = 'position_proportional_gain'


def _gz_plugins(root):
    return [p for g in root.findall('gazebo') for p in g.findall('plugin')
            if p.get('filename') == 'gz_ros2_control-system']


def test_plant_file_sets_only_the_plugin_node_gain_to_one():
    with open(PLANT) as f:
        data = yaml.safe_load(f)
    assert data == {'gz_ros2_control': {'ros__parameters': {GAIN: 1.0}}}
    gain = data['gz_ros2_control']['ros__parameters'][GAIN]
    assert type(gain) is float and math.isfinite(gain)   # the plugin rejects an integer 1


def test_control_urdf_passes_the_controllers_file_then_the_plant_file():
    plugins = _gz_plugins(config_check.load_control_urdf())
    assert len(plugins) == 1
    assert [c.tag for c in plugins[0]] == ['parameters', 'parameters']   # no other XML syntax
    files = [c.text.strip() for c in plugins[0]]
    assert os.path.realpath(files[0]) == os.path.realpath(CONTROLLERS)
    assert os.path.realpath(files[1]) == os.path.realpath(PLANT)


def test_passive_description_is_unchanged():
    assert _gz_plugins(config_check.load_passive_fortress_urdf()) == []
    assert config_check.load_passive_fortress_urdf().findall('ros2_control') == []


def test_controller_config_keeps_its_rate_and_has_no_gain():
    with open(CONTROLLERS) as f:
        text = f.read()
    ctrl = yaml.safe_load(text)
    assert set(ctrl) == {'controller_manager', 'leg_trajectory_controller'}
    assert ctrl['controller_manager']['ros__parameters']['update_rate'] == 100
    assert GAIN not in text


def test_physics_timestep_and_real_time_factor_are_unchanged():
    physics = ET.parse(os.path.join(DESCRIPTION, 'worlds', 'spiderx_fortress.sdf')).find(
        'world/physics')
    assert float(physics.findtext('max_step_size')) == 0.001
    assert float(physics.findtext('real_time_factor')) == 1.0


def test_legacy_classic_configuration_is_untouched():
    for name in ('config/controllers.yaml', 'urdf/spiderx.gazebo', 'urdf/spiderx.trans'):
        with open(os.path.join(DESCRIPTION, name)) as f:
            text = f.read()
        assert GAIN not in text and 'gz_ros2_control_plant' not in text, name


def test_all_three_dispatch_gates_stay_false():
    from spiderx_controller import m55_contract, m61_live_contract, m6_live_contract
    assert m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED is False
    assert m61_live_contract.M61_LIVE_DISPATCH_ENABLED is False
    assert m6_live_contract.LIVE_DISPATCH_ENABLED is False
