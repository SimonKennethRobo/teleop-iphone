# Teleop-iPhone

The ROS2 package is adapted for the Go2-X5 OCS2 stack: iPhone motion is converted into world-frame EE goals for OCS2, with an optional `go2_x5_planning` bridge. OCS2 remains the only joint-command producer. The legacy sensor receiver topics are still available for diagnostics.

This repository contains tools for receiving ARKit and sensor data from ZigSim Pro iOS app:

1. **Python UDP/JSON Receiver** - Simple standalone script for testing
2. **ROS2 OSC Node** - Full ROS2 integration for robotics applications
3. **OSC Test Receiver** - Lightweight OSC parser for debugging

## Quick Start

### Option 1: Python Script (JSON/UDP)

Simple receiver for testing and visualization:

```bash
python receive_zigsim_data.py -p 8888
```

Configure ZigSim Pro to send **JSON** data to your computer's IP on port **8888**.

### Option 2: ROS2 Package (JSON)

For ROS2 integration:

```bash
# Quick setup and build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_iphone --symlink-install

# Source workspace
source ros2_ws/install/setup.bash

# Launch node
ros2 launch teleop_iphone teleop.launch.py
```

Configure ZigSim Pro to send **JSON** data to your computer's IP on port **3333** (or your configured port).

See [ROS2_README.md](ROS2_README.md) for detailed instructions.

### Option 3: OSC Test Receiver

Test OSC connectivity:

```bash
python test_osc_receiver.py 8000
```

---

## Python UDP Receiver (receive_zigsim_data.py)

Receives JSON-formatted sensor data via UDP.

### Usage

```bash
# Basic usage (default port 8888)
python receive_zigsim_data.py

# Custom port
python receive_zigsim_data.py -p 9000

# Specific host and port
python receive_zigsim_data.py -H 192.168.1.100 -p 8888

# Help
python receive_zigsim_data.py --help
```

### ZigSim Pro Configuration

1. Open ZigSim Pro on your iPhone
2. Set Protocol: **JSON** or **UDP**
3. Set Target IP: Your computer's IP address
4. Set Target Port: **8888** (or your custom port)
5. Enable desired sensors
6. Start streaming

### Data Displayed

- **ARKit Data**: Camera position, rotation, quaternion
- **Attitude**: Roll, Pitch, Yaw
- **Accelerometer**: 3-axis acceleration
- **Gyroscope**: 3-axis rotation rate
- **Magnetometer**: 3-axis magnetic field
- **GPS/Location**: Latitude, Longitude, Altitude
- **Touch**: Touch events

---

## ROS2 Package (teleop_iphone)

Full-featured ROS2 node that receives OSC data and publishes to standard ROS2 topics.

### Quick Setup

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_iphone --symlink-install
```

### Published Topics

| Topic                       | Message Type                     | Description             |
| --------------------------- | -------------------------------- | ----------------------- |
| `/iphone/imu`             | `sensor_msgs/Imu`              | IMU data (accel + gyro) |
| `/iphone/pose`            | `geometry_msgs/PoseStamped`    | Device orientation      |
| `/iphone/arkit_pose`      | `geometry_msgs/PoseStamped`    | ARKit 6DOF pose         |
| `/iphone/gps`             | `sensor_msgs/NavSatFix`        | GPS location            |
| `/iphone/magnetic_field`  | `sensor_msgs/MagneticField`    | Magnetometer            |
| `/iphone/gravity`         | `geometry_msgs/Vector3Stamped` | Gravity vector          |
| `/iphone/pressure`        | `sensor_msgs/FluidPressure`    | Barometric pressure     |
| `/iphone/compass_heading` | `std_msgs/Float64`             | Compass heading         |

**See [TOPICS.md](TOPICS.md) for complete topic documentation.**

### ZigSim Pro Configuration

1. Open ZigSim Pro on your iPhone
2. Set Protocol: **JSON** or **UDP**
3. Set Target IP: Your computer's IP address
4. Set Target Port: **3333** (ROS2) or **8888** (Python)
5. Enable desired sensors
6. Start streaming

For detailed instructions, see [ROS2_README.md](ROS2_README.md).

---

## OSC Test Receiver (test_osc_receiver.py)

Lightweight script to verify OSC data is arriving correctly. Note: Use this only if ZigSim Pro is configured to send OSC data.

### Usage

```bash
# Default port 8000
python test_osc_receiver.py

# Custom port
python test_osc_receiver.py 9000
```

Configure ZigSim Pro to send **OSC** data to the specified port (if using OSC protocol).

---

## Finding Your Computer's IP Address

### Linux

```bash
hostname -I
# or
ip addr show
```

### Mac

```bash
ifconfig | grep "inet " | grep -v 127.0.0.1
```

### Windows

```cmd
ipconfig
```

Look for your local network IP (usually starts with `192.168.x.x` or `10.x.x.x`).

---

## Troubleshooting

### No Data Received

1. **Check network**: Ensure iPhone and computer are on the same WiFi network
2. **Check firewall**: Allow UDP traffic on the specified port
   ```bash
   sudo ufw allow 8000/udp
   sudo ufw allow 8888/udp
   ```
3. **Verify data is arriving**:
   ```bash
   sudo tcpdump -i any -n port 8000
   ```
4. **Check IP address**: Make sure you're using the correct IP in ZigSim Pro

### Port Already in Use

Change the port number in both the script and ZigSim Pro configuration.

---

## File Structure

```
zigsim_tele/
├── receive_zigsim_data.py    # Python UDP/JSON receiver
├── test_osc_receiver.py      # OSC test script
├── README.md                 # This file
├── ROS2_README.md           # Detailed ROS2 documentation
├── package.xml              # ROS2 package manifest
├── CMakeLists.txt           # ROS2 build configuration
├── src/
│   └── osc_node.cpp  # ROS2 node source
├── launch/
│   └── teleop.launch.py     # ROS2 launch file
└── config/
    ├── osc_params.yaml          # UDP/OSC input and ARKit correction
    └── teleop_iphone.yaml       # Go2-X5 OCS2 goal routing
```

---

## Go2-X5 OCS2 teleoperation

The ROS2 teleop node publishes end-effector goals for the existing Go2-X5 OCS2 graph. It does not publish joint, Cartesian-controller, gripper, or reset commands. It publishes both the planner-facing `PoseStamped` goal and the native OCS2 target topic, so the planner is optional for this teleop path.

Start the MPC graph first. The launch below also starts the existing reference manager, although teleop can publish the native target directly:

```bash
# Terminal 1: start OCS2 and its EE reference manager.
ros2 launch go2_x5_ocs2 go2_x5_ros_mpc.launch.py \
  rviz:=false start_rviz:=false ee_reference_mode:=marker
# Terminal 2: start the iPhone receiver and goal teleop node.
ros2 launch teleop_iphone teleop.launch.py
```

Configure ZigSim Pro to send JSON/UDP to the host and port in `config/osc_params.yaml`. The receiver publishes the corrected phone pose as `iphone_odom -> iphone`. On a running OCS2/WBC mode, press one or two fingers to clutch the current EE pose, move the phone, and release to stop. Three fingers disengage; release all fingers before pressing again.

The data path is:

```text
ZigSim Pro -> osc_node -> /iphone/touch + TF
teleop_iphone -> /go2_x5/ee/goal (geometry_msgs/PoseStamped)
teleop_iphone -> /mobile_manipulator_mpc_target (ocs2_msgs/MpcTargetTrajectories)
go2_x5_planning -> /mobile_manipulator_mpc_target (when the planner is running)
go2_x5_ocs2 -> /go2_x5/arm/command/request
```

`goal_frame` defaults to `world`, matching the OCS2 floating-base model. Set `phone_to_world_yaw` in `config/teleop_iphone.yaml` when the phone's ARKit horizontal axes are rotated relative to the SLAM world. The teleop node requires fresh `/go2_x5/ee/state` feedback and an `OCS2` or `WBC` arm mode before publishing goals.


## License

MIT
