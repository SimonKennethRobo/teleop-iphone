"""Synthetic ROS feedback -> actual UDP bytes compatible with the App listener."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

import pytest


def test_robot_state_round_trip(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from nav_msgs.msg import Odometry
    from ocs2_msgs.msg import MpcObservation
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Float64, String

    root = Path(__file__).parents[1]
    binaries = Path(os.environ.get("TELEOP_TEST_BIN_DIR", "build/teleop_iphone"))
    if not (binaries / "teleop_iphone").exists():
        pytest.skip("Set TELEOP_TEST_BIN_DIR to built binaries")
    old = {key: os.environ.get(key) for key in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE")}
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 50)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    phone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    phone.bind(("127.0.0.1", 0))
    phone.setblocking(False)
    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    context = rclpy.Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node("robot_feedback_test", context=context)
    mode_pub = node.create_publisher(String, "/go2_x5/arm/mode/state",
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    gripper_pub = node.create_publisher(Float64, "/arx5_controller/gripper_state", qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, "/go2_x5/slam/odom", qos_profile_sensor_data)
    battery_pub = node.create_publisher(BatteryState, "/test/battery", qos_profile_sensor_data)
    diagnostics_pub = node.create_publisher(DiagnosticArray, "/test/diagnostics", qos_profile_sensor_data)
    temperature_pub = node.create_publisher(Float64, "/test/temperature", qos_profile_sensor_data)
    ee_pub = node.create_publisher(MpcObservation, "/go2_x5/ee/state", qos_profile_sensor_data)
    snapshots = []
    processes, logs = [], []
    packet = dict(schemaVersion=1, type="teleoperation", streamID=str(uuid.uuid4()), sequence=0,
                  active=True, poseValid=False, tracking="unavailable", controlMode="gamepad",
                  touch=dict(active=False, count=0),
                  buttons={**{f"button{i}": False for i in range(1, 9)},
                           "presses": [0] * 8, "values": [0.0] * 8, "latched": [False] * 8},
                  gamepad=dict(left=dict(x=0.0, y=0.0, active=False),
                               right=dict(x=0.0, y=0.0, active=False),
                               functions=dict(states=[False] * 4, presses=[0] * 4, values=[0.0] * 4)))
    sensors_on = False
    ee_on = False

    def drain():
        while True:
            try:
                body, _ = phone.recvfrom(65536)
            except BlockingIOError:
                return
            assert len(body) <= 32768
            value = json.loads(body, parse_constant=lambda bad: pytest.fail(f"Invalid JSON number: {bad}"))
            assert isinstance(value, dict)
            snapshots.append(value)

    def pump(seconds, send=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if send:
                packet["sequence"] += 1
                control.sendto(json.dumps(packet).encode(), ("127.0.0.1", port))
            if sensors_on:
                gripper_pub.publish(Float64(data=0.03))
                odom = Odometry()
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.child_frame_id = "base"
                odom.twist.twist.linear.x = 0.2
                odom.twist.twist.angular.z = -0.1
                odom_pub.publish(odom)
                battery = BatteryState(present=True, percentage=0.7)
                battery.header.stamp = node.get_clock().now().to_msg()
                battery_pub.publish(battery)
            if ee_on:
                observation = MpcObservation(time=time.monotonic(), mode=1)
                observation.state.value = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
                ee_pub.publish(observation)
            rclpy.spin_once(node, executor=executor, timeout_sec=0.025)
            time.sleep(0.01)
            drain()

    def start(command, name):
        log = (tmp_path / f"{name}.log").open("w+")
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    try:
        start([str(binaries / "osc_node"), "--ros-args", "-p", "host:=127.0.0.1", "-p", f"port:={port}"], "osc")
        teleop = start([str(binaries / "teleop_iphone")], "teleop")
        start([sys.executable, str(root / "scripts/robot_state_feedback.py"), "--ros-args",
               "-p", f"port:={phone.getsockname()[1]}", "-p", "peer_timeout:=0.5",
               "-p", "teleop_timeout:=0.35", "-p", "feedback_timeout:=0.2", "-p", "health_timeout:=0.5",
               "-p", "battery_topic:=/test/battery", "-p", "diagnostics_topic:=/test/diagnostics",
               "-p", "temperature_topic:=/test/temperature"], "feedback")
        pump(1.5, send=False)
        assert not snapshots  # No default/broadcast destination before valid phone controls.
        pump(0.6)
        assert all(process.poll() is None for process in processes)
        state = snapshots[-1]
        assert state["robot"] == "go2-x5" and state["telemetry"]["valid"]
        assert state["input_ack"]["stream_id"] == packet["streamID"]
        assert 0 < state["input_ack"]["sequence"] <= packet["sequence"]
        assert state["gripper"]["measured_mm"] is None and not state["gripper"]["valid"]
        assert state["fault"] == dict(valid=False, age_ms=None, summary=None)
        assert state["battery"]["percent"] is None
        mode_pub.publish(String(data="HOME"))
        pump(0.3)
        packet["buttons"]["button7"] = True  # X requests OCS2.
        packet["buttons"]["presses"][6] += 1
        pump(0.3)
        state = snapshots[-1]
        assert state["mode"] == "HOME"
        assert state["mode_request"] == dict(requested="OCS2", state="pending")
        mode_pub.publish(String(data="OCS2"))
        packet["buttons"]["button7"] = False
        packet["gamepad"]["functions"]["states"][0] = True
        packet["gamepad"]["functions"]["values"][0] = 0.5
        sensors_on = True
        pump(0.4)
        state = snapshots[-1]
        assert state["mode_request"]["state"] == "matched"
        assert state["gripper"]["command_mm"] == pytest.approx(44.0)
        assert state["gripper"]["measured_mm"] == pytest.approx(30.0)
        assert state["base"]["measured_mps_rps"] == pytest.approx([0.2, 0.0, -0.1])
        assert state["base"]["frame"] == "base" and state["base"]["valid"]
        assert state["battery"]["percent"] == pytest.approx(70.0)
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = node.get_clock().now().to_msg()
        diagnostics.status = [DiagnosticStatus(level=bytes([2]), name="arm", message="test drive fault")]
        diagnostics_pub.publish(diagnostics)
        temperature_pub.publish(Float64(data=55.0))
        pump(0.3)
        state = snapshots[-1]
        assert state["fault"]["valid"] and "test drive fault" in state["fault"]["summary"]
        assert state["temperature"]["max_c"] == pytest.approx(55.0)
        diagnostics.header.stamp = node.get_clock().now().to_msg()
        diagnostics.status[0].level = bytes([0])
        diagnostics_pub.publish(diagnostics)
        pump(0.25)
        assert snapshots[-1]["fault"]["valid"] and snapshots[-1]["fault"]["summary"] == "OK"
        packet["controlMode"] = "arkit"
        packet["buttons"] = {**{f"button{i}": False for i in range(1, 9)},
                             "presses": [0] * 8, "values": [0.0] * 8, "latched": [False] * 8}
        packet["poseValid"] = True
        packet["tracking"] = "normal"
        packet["pose"] = dict(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
        ee_on = True
        pump(0.4)
        packet["buttons"]["button1"] = True
        packet["buttons"]["presses"][0] = 1
        pump(0.04)
        packet["buttons"]["button1"] = False
        pump(0.3)
        state = snapshots[-1]
        assert state["alignment"]["state"] == "aligned" and state["alignment"]["count"] == 1
        assert abs(state["alignment"]["yaw_deg"]) < 0.1
        assert state["teleop"]["engaged"]
        packet["pose"]["position"][2] = -0.1
        pump(0.3)
        assert snapshots[-1]["ee_error"]["valid"]
        assert snapshots[-1]["ee_error"]["position_mm"] == pytest.approx(100, abs=0.1)
        assert snapshots[-1]["ee_error"]["rotation_deg"] == pytest.approx(0, abs=0.1)
        # The App displays at most 64 leaves: check the populated snapshot fits.
        def leaves(value):
            return sum(leaves(child) for child in value.values()) if isinstance(value, dict) else 1
        assert leaves(snapshots[-1]) <= 64
        # Stamped data already old when received must not be accepted as fresh.
        sensors_on = False
        old_odom = Odometry()
        old_odom.header.stamp.sec = 1
        old_odom.child_frame_id = "base"
        odom_pub.publish(old_odom)
        pump(0.7)
        state = snapshots[-1]
        assert state["base"]["measured_mps_rps"] is None and not state["base"]["valid"]
        assert state["gripper"]["measured_mm"] is None and not state["gripper"]["valid"]
        assert state["battery"]["percent"] is None and not state["battery"]["valid"]
        ee_on = False
        pump(0.7)
        assert not snapshots[-1]["teleop"]["engaged"]
        assert not snapshots[-1]["ee_error"]["valid"]
        assert snapshots[-1]["ee_error"]["position_mm"] is None
        teleop.terminate()
        teleop.wait(timeout=5)
        pump(0.6)
        state = snapshots[-1]
        assert not state["telemetry"]["valid"]
        assert state["status"] == "teleop_status_stale" and state["mode"] is None
        assert state["gripper"]["command_mm"] is None
        # Auto destination expires even while the bridge itself is healthy.
        pump(0.8, send=False)
        count = len(snapshots)
        pump(0.35, send=False)
        assert len(snapshots) == count
    except Exception:
        for log in logs:
            log.flush()
            log.seek(0)
            print(log.read())
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        phone.close()
        control.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        for key, value in old.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
