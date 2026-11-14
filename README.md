# Teleop-iPhone

The ROS2 package is adapted for the Go2-X5 OCS2 stack: iPhone motion is converted into world-frame EE goals for OCS2, with an optional `go2_x5_planning` bridge. OCS2 remains the only joint-command producer. The legacy sensor receiver topics are still available for diagnostics.

The main teleoperation path follows the current EmbodiLab iOS App. Legacy ZigSim Pro sensor receivers remain available:

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

The ROS2 teleop node publishes world-frame end-effector goals, arm mode requests and percentage-controlled gripper width. It publishes both the planner-facing `PoseStamped` goal and the native OCS2 target, so the planner is optional. OCS2 remains the joint-command owner.

Start the robot's MPC graph separately, then launch the receiver:

```bash
ros2 launch teleop_iphone teleop_go2_x5.launch.py
```

In EmbodiLab → Teleoperation, select **Wi-Fi / UDP**, the computer's LAN IP, and port **3333** (matching `config/osc_params.yaml`). TCP, USB and Bluetooth control transports are not connected by this ROS receiver. Select AR or Gamepad, then start sending; mode changes in the App require starting again.

In AR, the transmit switch is the clutch by default: fresh tracked phone motion engages relative to the measured EE, and stopping, tracking loss or timeout disengages. Set `require_touch_clutch: true` to additionally require a finger outside controls. F1 percentage directly commands gripper width in either mode while the arm is in OCS2/WBC. Releasing F1 or stopping holds the last width. Marker estimates cannot override F1; set `gripper_control_source: marker` to explicitly restore the previous marker input. No finger-count HOME/gripper gestures are used.

| Visible key | Default action | Canonical slot (1-based) |
| --- | --- | --- |
| A | Tap to align phone heading to robot `base` +x, AR only | 1 |
| B | Request HOME | 2 |
| X / Y | Request OCS2 / WBC | 3 / 4 |
| F1 | Gripper opening: 0% closed, 100% configured maximum width | 5 |
| F2 | Shared AR position and rotation scale | 6 |
| F3 | AR axis lock: latched ≤50% translation only, >50% rotation only | 7 |
| F4 | Precision: ×0.3 AR motion and Gamepad base velocity | 8 |

F1/F2 work while held and retain their values when locked with the App's F1–F4 latch setting. F2 uses base scale × `3^(2*value-1)`: 50% is the base scale, 0% divides by three, 100% multiplies by three. F4 enables precision (×0.3) while pressed or latched; rotation scale is capped at one. F3 requires latch to enable axis lock. Changing scales reanchors the relative goal to avoid jumps. A aligns once per tap; F4 exclusively controls precision. Button indices in the YAML refer to the canonical slots above, not Gamepad wire order.

Gamepad left stick controls base forward/lateral velocity; right stick X controls yaw rate. Xbox and PlayStation use the same wire channels. In Steering wheel layout, RT controls forward speed, LT reduces it to `RT × (1-LT)`, and steering controls yaw rate; lateral velocity is zero. Full brake stops forward motion and never requests reverse. D-pad and L3/R3 are published without robot actions. Gamepad F1 commands gripper width and F4 enables precision; F2/F3 values remain published on `/iphone/gamepad`; AR scale/lock/alignment controls do not alter Gamepad base motion. B/X/Y retain their arm mode requests in both modes. Inactive packets and returning to AR immediately publish zero base velocity; missing, invalid, duplicate or reordered packets do not refresh its 0.2 s watchdog.

The data path is:

```text
EmbodiLab JSON/UDP -> osc_node -> /iphone/buttons + /iphone/gamepad + TF
teleop_iphone -> /go2_x5/ee/goal (geometry_msgs/PoseStamped)
teleop_iphone -> /mobile_manipulator_mpc_target (ocs2_msgs/MpcTargetTrajectories)
go2_x5_planning -> /mobile_manipulator_mpc_target (when the planner is running)
go2_x5_ocs2 -> /go2_x5/arm/command/request
```

`goal_frame` defaults to `world`, matching the OCS2 floating-base model. Set `phone_to_world_yaw` in `config/teleop_iphone.yaml` when the phone's ARKit horizontal axes are rotated relative to the SLAM world. The teleop node requires fresh `/go2_x5/ee/state` feedback and an `OCS2` or `WBC` arm mode before publishing goals.


## Robot feedback to iPhone

The normal `teleop.launch.py` and `teleop_go2_x5.launch.py` now start a display-only
`robot_state_feedback.py` bridge. In EmbodiLab Settings → **Robot state JSON**,
keep port **5567**. Start sending controls once: `osc_node` learns the source
phone's IPv4 address from accepted EmbodiLab packets, and the bridge returns
complete JSON snapshots over UDP at 5 Hz. The source UDP port is not reused.
After controls stop, feedback continues for up to 30 seconds; a silent peer then
expires. No address is guessed or broadcast before a phone has sent controls.

```bash
# Normal operation: automatic phone address, existing robot graph running separately.
ros2 launch teleop_iphone teleop_go2_x5.launch.py
# Fixed destination, including feedback before the phone starts sending controls.
ros2 launch teleop_iphone teleop_go2_x5.launch.py feedback_host:=192.168.1.20
# Disable feedback when desired.
ros2 launch teleop_iphone teleop_go2_x5.launch.py feedback:=false
```

Configuration: `config/robot_state_feedback.yaml`. The launch argument
`feedback_host` overrides its `host`; both default to empty (auto). The bridge can
also run independently with that YAML and an explicit `host` parameter.

| Phone fields | Meaning / source |
| --- | --- |
| `mode`, `mode_report_age_ms`, `mode_request` | Reported arm mode and its age; requested mode is separately pending/matched/unconfirmed. Mode state is latched, so its receipt age alone is not a hardware heartbeat. |
| `teleop` | Actual EE command engagement, pause reason, accepted-input and EE-feedback freshness. `engaged` does not mean motors are enabled. |
| `controls`, `alignment` | Effective F2 scale, F3 lock, F4 precision; A alignment result/count, yaw and actual reference frame (including world fallback). |
| `gripper.command_mm`, `gripper.measured_mm` | Sent command versus measured driver width from `/arx5_controller/gripper_state`. `gripper.valid` applies to the measurement. |
| `ee_error` | Distance in mm and shortest quaternion angle in degrees between this node's latest EE goal and measured `/go2_x5/ee/state`, only while fresh and following. |
| `base.command_mps_rps`, `base.measured_mps_rps` | [vx, vy, wz] command versus odometry twist from `/go2_x5/slam/odom`; `base.frame` identifies the measured twist frame. `base.valid` applies to the measurement. |
| `input_ack` | Latest packet stream/sequence accepted by the UDP receiver; confirms reception, not actuator execution or round-trip latency. |
| `battery`, `temperature`, `fault` | Optional standard ROS sources; unavailable until explicitly configured. No fault feedback is not a clean bill of health. |

Optional sources are `sensor_msgs/BatteryState` (percentage 0–1),
`std_msgs/Float64` (maximum drive temperature in °C), and a robot-specific
`diagnostic_msgs/DiagnosticArray`. The native Unitree low-state format is not
implicitly decoded. Configure/remap the standard topics supplied by your robot.
Diagnostic changes trigger an additional full snapshot. `fault.summary: "OK"`
means the configured diagnostics source reported all OK; absent/stale diagnostics
remain `valid: false` with a null summary.

Every measurement includes `valid` and `age_ms` (time since receipt). Stamped
odometry/battery/diagnostics additionally check source timestamps against the ROS
clock and expose `source_age_ms`; zero, future or stale timestamps are invalid.
Unstamped gripper/temperature inputs can only be checked by receipt time. Use the
same ROS clock across publishers. Fast feedback expires after 0.5 s; health
feedback after 3 s. Invalid/stale measurements become JSON `null`, not zero.
The bridge clears teleop commands/engagement if structured teleop status stops
for 1 s, even while the App continues to label the incoming bridge packets LIVE.
Snapshots fit within the App's 64-field and 32 KiB limits. Full-rate data remains
in ROS/logs rather than the phone monitor.

The implementation is covered by synthetic ROS → UDP loopback tests. Phone
screen delivery and real hardware behavior require a separate live check.


## License

MIT
