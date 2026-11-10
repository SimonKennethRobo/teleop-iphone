#!/usr/bin/env python3
"""Deprecated diagnostic retained for scripts that still call its old name."""

print("The direct ARX5 ZMQ path was removed for Go2-X5. Check the native ROS graph instead:")
print("  ros2 topic echo /go2_x5/ee/state")
print("  ros2 topic echo /go2_x5/ee/goal")
raise SystemExit(2)
