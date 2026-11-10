#!/usr/bin/env python3
"""Compatibility entry point for the old direct-ARX5 teleop command.

The Go2-X5 stack has one command owner: go2_x5_ocs2. Keep this executable as
an alias for existing launch files, but always run the native ROS teleop node
instead of opening the removed ARX5 ZMQ control path.
"""
import os
import sys


def main() -> None:
    print("teleop_iphone_zmq is deprecated; forwarding to the OCS2 EE teleop node.", flush=True)
    os.execvp("ros2", ["ros2", "run", "teleop_iphone", "teleop_iphone", *sys.argv[1:]])


if __name__ == "__main__":
    main()
