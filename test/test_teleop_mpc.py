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
    assert "touch_count_ >= 3" not in source
    assert "on_gripper_opening" in source
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


def test_teleoperation_schema_topics_match():
    osc = (ROOT / "src" / "osc_node.cpp").read_text()
    config = (ROOT / "config" / "teleop_iphone.yaml").read_text()
    assert 'j.value("type", "") == "teleoperation"' in osc
    for topic in ("iphone/buttons", "iphone/tracking_ok", "iphone/gripper/opening"):
        assert f'"{topic}"' in osc
        assert f'"/{topic}"' in config
    for key, button in (("position_scale_button", 1), ("home_button", 2), ("axis_lock_button", 3),
                        ("rotation_scale_button", 4), ("precision_button", 5), ("align_button", 6)):
        assert f"{key}: {button}" in config
