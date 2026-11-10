from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_teleop_uses_native_ee_goal_contract():
    source = (ROOT / "src" / "teleop_iphone.cpp").read_text()
    zmq = (ROOT / "scripts" / "teleop_iphone_zmq.py").read_text()
    config = (ROOT / "config" / "teleop_iphone.yaml").read_text()
    assert '"/go2_x5/ee/goal"' in source
    assert '"/go2_x5/ee/state"' in source
    assert '"/mobile_manipulator_mpc_target"' in source
    assert '"/go2_x5/ee/goal"' in config
    assert '"/go2_x5/ee/state"' in config
    assert '"/mobile_manipulator_mpc_target"' in config
    assert '"/go2_x5/arm/mode/request"' in source
    assert '"/go2_x5/arm/mode/request"' in config
    assert 'request.data = home_mode_' in source
    assert '"/go2_x5/arm/gripper/command"' in source
    assert '"/go2_x5/arm/gripper/command"' in config
    assert "command_gripper(touch_count_ == 2)" in source
    assert "phone_to_world_yaw: 1.5707963267948966" in config
    assert "/arx5_controller" not in source
    assert "reset_to_home" not in source
    assert "Arx5Client" not in zmq
    assert "set_ee_pose" not in zmq
    assert "if (!mode_.empty() && msg->data != mode_)" in source


def test_osc_pose_frame_matches_teleop_lookup():
    osc = (ROOT / "config" / "osc_params.yaml").read_text()
    teleop = (ROOT / "config" / "teleop_iphone.yaml").read_text()
    assert 'frame_id: "iphone"' in osc
    assert 'iphone_target_frame: "iphone"' in teleop
    assert 'iphone_source_frame: "iphone_odom"' in teleop
