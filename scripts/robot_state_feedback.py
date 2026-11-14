#!/usr/bin/env python3
"""Send complete, display-only robot snapshots to EmbodiLab's JSON listener."""
import copy
import ipaddress
import json
import math
import socket
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, String
from diagnostic_msgs.msg import DiagnosticArray


class RobotStateFeedback(Node):
    def __init__(self):
        super().__init__("robot_state_feedback")
        defaults = {
            "robot_name": "go2-x5", "host": "", "port": 5567, "rate": 5.0,
            "peer_timeout": 30.0, "teleop_timeout": 1.0, "feedback_timeout": 0.5, "health_timeout": 3.0,
            "peer_topic": "/iphone/peer", "teleop_topic": "/teleop_iphone/robot_state",
            "gripper_state_topic": "/arx5_controller/gripper_state",
            "odom_topic": "/go2_x5/slam/odom", "battery_topic": "",
            "diagnostics_topic": "", "temperature_topic": "",
        }
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        for key in ("peer_timeout", "teleop_timeout", "feedback_timeout", "health_timeout", "rate"):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if not 1 <= self.settings["port"] <= 65535 or self.settings["rate"] > 20:
            raise ValueError("Use port 1–65535 and rate <= 20 Hz")
        if self.settings["host"]:
            self.check_host(self.settings["host"])
        self.peer = None
        self.core = None
        self.samples = {}
        self.sequence = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.last_warning = 0.0
        self.subscriptions_ = [
            self.create_subscription(String, self.settings["peer_topic"], self.on_peer, 1),
            self.create_subscription(String, self.settings["teleop_topic"], self.on_core, 1),
        ]
        for kind, key, callback in (
            (Float64, "gripper_state_topic", self.on_gripper),
            (Odometry, "odom_topic", self.on_odom),
            (BatteryState, "battery_topic", self.on_battery),
            (DiagnosticArray, "diagnostics_topic", self.on_diagnostics),
            (Float64, "temperature_topic", self.on_temperature),
        ):
            if self.settings[key]:
                self.subscriptions_.append(self.create_subscription(
                    kind, self.settings[key], callback, qos_profile_sensor_data))
        self.timer = self.create_timer(1.0 / self.settings["rate"], self.send_snapshot)
        self.get_logger().info(
            f"Robot JSON UDP -> {self.settings['host'] or 'current iPhone (auto)'}:{self.settings['port']}")

    @staticmethod
    def check_host(host):
        address = ipaddress.ip_address(host)
        if address.version != 4 or address.is_multicast or address.is_unspecified:
            raise ValueError("Use a unicast IPv4 address")

    def on_peer(self, msg):
        try:
            peer = json.loads(msg.data)
            self.check_host(peer["host"])
            if not isinstance(peer["stream_id"], str) or type(peer["sequence"]) is not int:
                return
            self.peer = (peer, time.monotonic())
        except (ValueError, TypeError, KeyError):
            return

    def on_core(self, msg):
        try:
            state = json.loads(msg.data, parse_constant=lambda value: None)
            if not isinstance(state, dict):
                return
            self.core = (state, time.monotonic())
        except (ValueError, TypeError):
            return

    def record(self, name, value, stamp=None):
        self.samples[name] = (value, time.monotonic(), stamp)

    def on_gripper(self, msg):
        self.record("gripper", msg.data * 1000.0 if math.isfinite(msg.data) and msg.data >= 0 else None)

    def on_temperature(self, msg):
        self.record("temperature", msg.data if math.isfinite(msg.data) else None)

    def on_battery(self, msg):
        value = msg.percentage * 100.0 if msg.present and math.isfinite(msg.percentage) and 0 <= msg.percentage <= 1 else None
        self.record("battery", value, msg.header.stamp)

    def on_odom(self, msg):
        t = msg.twist.twist
        velocity = [t.linear.x, t.linear.y, t.angular.z]
        value = {"velocity_mps_rps": velocity, "frame": msg.child_frame_id} if (
            msg.child_frame_id and all(math.isfinite(v) for v in velocity)) else None
        self.record("base", value, msg.header.stamp)

    def on_diagnostics(self, msg):
        # Only a deliberately configured, robot-specific diagnostics source is
        # interpreted. Missing diagnostics must not appear as "no faults".
        value = None
        if msg.status:
            warnings = []
            for status in msg.status:
                # ROS diagnostic level is a byte (bytes in Jazzy Python).
                level = int.from_bytes(status.level, "big") if isinstance(status.level, (bytes, bytearray)) else int(status.level)
                if level != 0:
                    warnings.append(f"{status.name} [{level}]: {status.message}")
            value = {"summary": "; ".join(warnings)[:512] if warnings else "OK"}
        previous = self.samples.get("fault")
        changed = previous is None or previous[0] != value
        self.record("fault", value, msg.header.stamp)
        if changed:
            self.send_snapshot()  # Surface changed diagnostics before the next tick.

    def measurement(self, name, now):
        sample = self.samples.get(name)
        if sample is None:
            return {"valid": False, "age_ms": None}, None
        value, received, stamp = sample
        age = now - received
        timeout = self.settings["health_timeout" if name in ("battery", "temperature", "fault") else "feedback_timeout"]
        valid = value is not None and 0 <= age <= timeout
        info = {"age_ms": round(age * 1000.0, 1)}
        if stamp is not None:
            seconds = stamp.sec + stamp.nanosec * 1e-9
            source_age = self.get_clock().now().nanoseconds * 1e-9 - seconds
            valid = valid and seconds > 0 and 0 <= source_age <= timeout
            info["source_age_ms"] = round(source_age * 1000.0, 1) if seconds > 0 else None
        return {"valid": bool(valid), **info}, value if valid else None

    def snapshot(self, now):
        fresh = self.core is not None and now - self.core[1] <= self.settings["teleop_timeout"]
        if fresh:
            state = copy.deepcopy(self.core[0])
        else:
            # This bridge may still be LIVE on the phone after teleop dies.
            # Do not retain old commands or engagement as current values.
            state = {"status": "teleop_status_stale", "mode": None,
                     "teleop": {"engaged": False, "reason": "waiting for fresh teleop status", "input_valid": False, "ee_valid": False},
                     "gripper": {"command_mm": None},
                     "ee_error": {"valid": False, "position_mm": None, "rotation_deg": None},
                     "base": {"command_mps_rps": None}}
        state["robot"] = self.settings["robot_name"]
        state["sequence"] = self.sequence
        state["timestamp"] = time.time()
        state["telemetry"] = {"valid": fresh, "age_ms": round((now - self.core[1]) * 1000.0, 1) if self.core else None}
        state["input_ack"] = {
            "stream_id": self.peer[0]["stream_id"] if self.peer else None,
            "sequence": self.peer[0]["sequence"] if self.peer else None,
            "age_ms": round((now - self.peer[1]) * 1000.0, 1) if self.peer else None,
        }
        info, value = self.measurement("gripper", now)
        state.setdefault("gripper", {}).update({"measured_mm": value, **info})
        info, value = self.measurement("base", now)
        state.setdefault("base", {}).update({"measured_mps_rps": value["velocity_mps_rps"] if value else None,
                                             "frame": value["frame"] if value else None, **info})
        for name, field in (("battery", "percent"), ("temperature", "max_c"), ("fault", "summary")):
            info, value = self.measurement(name, now)
            state[name] = {**info, field: value["summary"] if name == "fault" and value is not None else value}
        return state

    def send_snapshot(self):
        now = time.monotonic()
        host = self.settings["host"]
        if not host:
            if self.peer is None or now - self.peer[1] > self.settings["peer_timeout"]:
                return
            host = self.peer[0]["host"]
        self.sequence += 1
        try:
            body = json.dumps(self.snapshot(now), allow_nan=False, ensure_ascii=False, separators=(",", ":")).encode()
            if len(body) > 32768:
                raise ValueError("Robot state exceeds the App's 32 KiB limit")
            self.socket.sendto(body, (host, self.settings["port"]))
        except (OSError, ValueError) as error:
            if now - self.last_warning > 5:
                self.get_logger().warning(f"Robot state UDP send failed: {error}")
                self.last_warning = now

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = RobotStateFeedback()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
