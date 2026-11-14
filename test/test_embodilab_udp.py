"""Exercise current App JSON through the real receiver and teleop binaries."""
import json
import os
import re
from pathlib import Path
import socket
import subprocess
import time
import uuid

import pytest


def test_embodilab_controls_and_stop():
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Float64, String

    binaries = Path(os.environ.get("TELEOP_TEST_BIN_DIR", "build/teleop_iphone"))
    if not (binaries / "osc_node").exists():
        pytest.skip("Build binaries and set TELEOP_TEST_BIN_DIR for UDP integration")
    # Isolate test traffic from existing ROS graphs, including robot drivers.
    old_domain = os.environ.get("ROS_DOMAIN_ID")
    old_range = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE")
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 50)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    processes = []
    context = rclpy.Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node("embodilab_protocol_test", context=context)
    messages = {"buttons": [], "gamepad": [], "velocity": [], "mode": [], "status": [], "gripper": []}
    subscriptions = [node.create_subscription(kind, topic, messages[key].append, 20)
                     for kind, topic, key in (
                         (Joy, "/iphone/buttons", "buttons"),
                         (Joy, "/iphone/gamepad", "gamepad"),
                         (Twist, "/go2_x5/base/cmd_vel", "velocity"),
                         (String, "/go2_x5/arm/mode/request", "mode"),
                         (String, "/teleop_iphone/status", "status"),
                         (Float64, "/go2_x5/arm/gripper/command", "gripper"))]
    mode_pub = node.create_publisher(String, "/go2_x5/arm/mode/state",
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    marker_pub = node.create_publisher(Float64, "/iphone/gripper/opening", 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = dict(schemaVersion=1, type="teleoperation", streamID=str(uuid.uuid4()),
                  sequence=0, active=True, poseValid=False, tracking="unavailable",
                  controlMode="gamepad", touch=dict(active=False, count=0),
                  buttons={**{f"button{i}": False for i in range(1, 9)},
                           "presses": [0] * 8, "values": [0] * 8, "latched": [False] * 8},
                  gamepad=dict(left=dict(x=0.0, y=0.0, active=False),
                               right=dict(x=0.0, y=0.0, active=False),
                               functions=dict(states=[False] * 4, presses=[0] * 4, values=[0.0] * 4)))

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=0.01)

    def send(seconds=0.15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            packet["sequence"] += 1
            sock.sendto(json.dumps(packet).encode(), ("127.0.0.1", port))
            spin(0.025)

    def velocity():
        assert messages["velocity"]
        return messages["velocity"][-1]

    try:
        for name, args in (("osc_node", ["-p", "host:=127.0.0.1", "-p", f"port:={port}"]),
                           ("teleop_iphone", [])):
            processes.append(subprocess.Popen([str(binaries / name), "--ros-args", *args],
                                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        spin(1.5)
        send(0.4)
        assert all(p.poll() is None for p in processes)
        assert messages["buttons"] and messages["gamepad"]
        # D-pad must never request an arm mode.
        packet["buttons"]["button2"] = True
        packet["buttons"]["presses"][1] += 1
        send()
        assert not messages["mode"]
        # The visible B/X/Y occupy different wire slots in Gamepad.
        for wire, expected in ((6, "HOME"), (7, "OCS2"), (8, "WBC")):
            packet["buttons"][f"button{wire}"] = True
            packet["buttons"]["presses"][wire - 1] += 1
            send()
            assert messages["mode"][-1].data == expected
            packet["buttons"][f"button{wire}"] = False
            send()
        packet["gamepad"]["left"] = dict(x=0.0, y=1.0, active=True)
        send()
        assert velocity().linear.x == pytest.approx(0.8)
        packet["buttons"]["button5"] = True  # A no longer changes base velocity
        send()
        assert velocity().linear.x == pytest.approx(0.8)
        packet["buttons"]["button5"] = False
        packet["gamepad"]["functions"]["states"][3] = True
        packet["gamepad"]["functions"]["presses"][3] += 1
        send()
        assert velocity().linear.x == pytest.approx(0.24)
        packet["gamepad"]["functions"]["states"][3] = False
        mode_pub.publish(String(data="WBC"))
        spin(0.1)
        # Independent function values and L3/R3 remain available on raw Joy.
        packet["gamepad"]["left"]["click"] = True
        packet["gamepad"]["functions"] = dict(states=[True, False, True, False],
                                                   presses=[1, 0, 1, 0], values=[0.7, 0, 0.2, 0])
        send()
        raw = messages["gamepad"][-1]
        assert list(raw.buttons[8:12]) == [1, 0, 1, 0]
        assert raw.buttons[12] == 1
        assert list(raw.axes[5:]) == pytest.approx([0.7, 0, 0.2, 0])
        assert messages["gripper"][-1].data == pytest.approx(0.088 * 0.7)
        packet["gamepad"]["functions"]["values"][0] = 0.25
        send()
        assert messages["gripper"][-1].data == pytest.approx(0.088 * 0.25)
        packet["gamepad"]["functions"]["states"][0] = False
        packet["gamepad"]["functions"]["values"][0] = 0
        send()
        gripper_count = len(messages["gripper"])
        marker_pub.publish(Float64(data=1.0))
        send()
        assert len(messages["gripper"]) == gripper_count  # Release holds; marker cannot override.
        # Both pedals may reach one: no unit-disc rejection and full brake stops.
        packet["gamepad"]["layout"] = "steering"
        packet["gamepad"]["left"] = dict(x=0.5, y=0.0, active=True)
        packet["gamepad"]["right"] = dict(x=1.0, y=1.0, active=True)
        send()
        assert velocity().linear.x == 0
        assert velocity().linear.y == 0
        assert velocity().angular.z == pytest.approx(-0.5)
        packet["gamepad"]["right"]["x"] = 0.25
        send()
        assert velocity().linear.x == pytest.approx(0.6)
        packet["active"] = False
        send()
        assert velocity().linear.x == velocity().angular.z == 0
        assert not any(messages["gamepad"][-1].buttons)
        packet["active"] = True
        send()
        # Switching to AR stops the base immediately, before its watchdog.
        packet["controlMode"] = "arkit"
        packet["buttons"] = {**{f"button{i}": False for i in range(1, 9)},
                             "presses": [0] * 8, "values": [0] * 8, "latched": [False] * 8}
        send()
        assert velocity().linear.x == velocity().angular.z == 0
        # AR slot 2 is B; F2 scales both motion channels, F4 enables precision.
        packet["buttons"]["button2"] = True
        packet["buttons"]["presses"][1] = 1
        packet["buttons"]["button6"] = True
        packet["buttons"]["latched"][5] = True
        packet["buttons"]["values"][5] = 1.0
        packet["buttons"]["values"][7] = 0.0
        send(0.3)
        assert messages["mode"][-1].data == "HOME"
        assert "pos_scale=3.00 rot_scale=1.00" in messages["status"][-1].data
        packet["buttons"]["button8"] = True
        send(0.3)
        assert "pos_scale=0.90 rot_scale=0.54" in messages["status"][-1].data
        assert "precision=on" in messages["status"][-1].data
        packet["buttons"]["button8"] = False
        packet["buttons"]["button7"] = True
        packet["buttons"]["latched"][6] = True
        packet["buttons"]["values"][6] = 0.8
        send(0.3)
        assert "pos_scale=0.00" in messages["status"][-1].data
        assert "lock=rotation" in messages["status"][-1].data
        # A aligns on a short tap; hold time is no longer required.
        packet["poseValid"] = True
        packet["tracking"] = "normal"
        packet["pose"] = dict(position=[0, 0, 0], orientation=[0, 0, 0, 1])
        send()  # Fresh pose before tapping.
        packet["buttons"]["button1"] = True
        packet["buttons"]["presses"][0] += 1
        send(0.04)
        packet["buttons"]["button1"] = False
        send(0.25)
        assert abs(float(re.search(r"yaw=([+-]?[0-9.]+)deg", messages["status"][-1].data)[1])) < 0.1
        assert "precision=off" in messages["status"][-1].data
        packet["buttons"]["button5"] = True
        packet["buttons"]["values"][4] = 1.0
        send()
        assert messages["gripper"][-1].data == pytest.approx(0.088)
        packet["buttons"]["values"][4] = 0.0
        send()
        assert messages["gripper"][-1].data == 0.0
        # Inactive counters cannot fire a mode request, even if a release is lost.
        count = len(messages["mode"])
        packet["active"] = False
        packet["buttons"]["presses"][1] += 1
        send()
        assert len(messages["mode"]) == count
        assert not any(messages["buttons"][-1].buttons[:16])
        # New stream counters are baselines, not fresh historical taps.
        packet["active"] = True
        packet["streamID"] = str(uuid.uuid4())
        packet["buttons"]["presses"][1] = 100
        send()
        assert len(messages["mode"]) == count
        # Lost packets and malformed/duplicate traffic must not keep driving.
        packet["controlMode"] = "gamepad"
        packet["gamepad"].pop("layout")
        packet["gamepad"]["left"] = dict(x=0.0, y=1.0, active=True)
        packet["gamepad"]["right"] = dict(x=0.0, y=0.0, active=False)
        send()
        assert velocity().linear.x > 0
        last = json.dumps(packet).encode()
        for _ in range(15):
            sock.sendto(last, ("127.0.0.1", port))
            spin(0.025)
        assert velocity().linear.x == velocity().angular.z == 0
        send()
        assert velocity().linear.x > 0
        packet["gamepad"]["right"]["x"] = 2.0
        send(0.4)
        assert velocity().linear.x == velocity().angular.z == 0
    finally:
        sock.close()
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        for key, original in (("ROS_DOMAIN_ID", old_domain), ("ROS_AUTOMATIC_DISCOVERY_RANGE", old_range)):
            if original is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = original
