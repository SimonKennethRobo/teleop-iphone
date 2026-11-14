# ROS2 Topics

For Go2-X5 OCS2 teleoperation, the package consumes `/go2_x5/ee/state` (`ocs2_msgs/msg/MpcObservation`) and publishes `/go2_x5/ee/goal` (`geometry_msgs/msg/PoseStamped`) plus `/mobile_manipulator_mpc_target` (`ocs2_msgs/msg/MpcTargetTrajectories`). Phone buttons B/X/Y publish `HOME`/`OCS2`/`WBC` to `/go2_x5/arm/mode/request`; see `config/teleop_iphone.yaml` for the A/B/X/Y/F1–F4 mapping. The planner-facing goal remains available when `go2_x5_planning` is running; OCS2 remains the joint-command owner.

Complete list of all published topics and their message types.

## Published Topics

| Topic                       | Message Type                     | Rate  | Description                                        |
| --------------------------- | -------------------------------- | ----- | -------------------------------------------------- |
| `/iphone/imu`             | `sensor_msgs/Imu`              | ~60Hz | Combined IMU data (accelerometer + gyroscope)      |
| `/iphone/pose`            | `geometry_msgs/PoseStamped`    | ~60Hz | Device orientation (quaternion from device sensor) |
| `/iphone/arkit_pose`      | `geometry_msgs/PoseStamped`    | ~60Hz | ARKit 6DOF pose (position + orientation)           |
| `/iphone/gps`             | `sensor_msgs/NavSatFix`        | ~1Hz  | GPS location (latitude, longitude, altitude)       |
| `/iphone/magnetic_field`  | `sensor_msgs/MagneticField`    | ~60Hz | Magnetometer 3-axis data                           |
| `/iphone/gravity`         | `geometry_msgs/Vector3Stamped` | ~60Hz | Gravity vector                                     |
| `/iphone/pressure`        | `sensor_msgs/FluidPressure`    | ~1Hz  | Barometric pressure (in Pascals)                   |
| `/iphone/compass_heading` | `std_msgs/Float64`             | ~60Hz | Compass heading (degrees, 0-360)                   |
| `/go2_x5/ee/goal`         | `geometry_msgs/PoseStamped`    | ~50Hz | World-frame teleop EE goal for the planner         |
| `/mobile_manipulator_mpc_target` | `ocs2_msgs/MpcTargetTrajectories` | ~50Hz | Native OCS2 EE target |
| `/go2_x5/arm/mode/request` | `std_msgs/String`              | Event | B/X/Y tap `HOME`/`OCS2`/`WBC` mode request     |
| `/go2_x5/arm/gripper/command` | `std_msgs/Float64`         | Event | Gripper width (m) from F1 percentage; optional explicit marker source |
| `/iphone/touch`           | `geometry_msgs/PoseArray`      | ~40Hz | One pose per finger (primary touch x/y)            |
| `/iphone/buttons`         | `sensor_msgs/Joy`              | ~40Hz | Both modes, canonical A/B/X/Y/F1/F2/F3/F4. N=8: pressed / latched / counters in first 24 `buttons`; `axes[i]` value 0–1. Extra flags: [24] active, [25] stream/mode changed, [26] AR. Header frame_id identifies phone/mode/stream for counter resets |
| `/iphone/gamepad`         | `sensor_msgs/Joy`              | ~60Hz | Raw Gamepad: `axes` = [left x, left y, right x, right y, steering-layout flag, F1…F4 values]. `buttons` = [Up,Down,Left,Right,A,B,X,Y,F1…F4,L3,R3]. Sticks unit disc; steering right pair = independent LT/RT in [0,1]. Controls neutral while inactive |
| `/go2_x5/base/cmd_vel`    | `geometry_msgs/Twist`          | ~60Hz | Gamepad base velocity: sticks -> x/y/yaw; steering -> RT×(1−LT) forward, left.x yaw; F4 precision |
| `/iphone/tracking_ok`     | `std_msgs/Bool`                | ~40Hz | `AR && active && poseValid && tracking == "normal"`      |
| `/iphone/gripper/opening` | `std_msgs/Float64`             | ~14Hz | Marker gripper opening 0 (closed) - 1 (open), only while valid |
| `/iphone/peer` | `std_msgs/String` | ~60Hz | Accepted EmbodiLab sender IPv4, stream ID and sequence JSON; feedback auto-destination and reception acknowledgement |
| `/teleop_iphone/robot_state` | `std_msgs/String` | 5Hz + events | Structured teleop status consumed by robot_state_feedback.py |
| `/teleop_iphone/status`   | `std_msgs/String`              | 5Hz   | Effective scales, axis lock, precision, yaw, gripper |
| `/teleop_iphone/status_marker` | `visualization_msgs/Marker` | 5Hz  | RViz text above the EE                             |

## Topic Details

### IMU (`/iphone/imu`)

Contains:

- **Linear acceleration** (m/s²): `linear_acceleration.{x,y,z}`
- **Angular velocity** (rad/s): `angular_velocity.{x,y,z}`
- **Orientation** (optional): `orientation.{x,y,z,w}`

### Pose (`/iphone/pose`)

Quaternion orientation from device motion sensor:

- Device quaternion sensor data
- Format: `pose.orientation.{x,y,z,w}`
- Position is set to origin (0,0,0)

### ARKit Pose (`/iphone/arkit_pose`)

Full 6DOF pose from ARKit (position + orientation):

- **Position** (meters): `pose.position.{x,y,z}` - Camera position in world space
- **Orientation** (quaternion): `pose.orientation.{x,y,z,w}` - Camera rotation
- Only published when ARKit tracking is active
- Provides both translation and rotation in a single message

### GPS (`/iphone/gps`)

GPS location data:

- **Latitude** (degrees): `latitude`
- **Longitude** (degrees): `longitude`
- **Altitude** (meters): `altitude`
- **Status**: `status.status` (STATUS_FIX when valid)

### Magnetic Field (`/iphone/magnetic_field`)

3-axis magnetometer readings:

- **Magnetic field** (Tesla): `magnetic_field.{x,y,z}`

### Gravity (`/iphone/gravity`)

Gravity vector in device frame:

- **Gravity** (g-force): `vector.{x,y,z}`
- Normalized to ~1.0 magnitude

### Pressure (`/iphone/pressure`)

Barometric pressure:

- **Pressure** (Pascals): `fluid_pressure`
- Note: Converted from hPa (multiply by 100)

### Compass Heading (`/iphone/compass_heading`)

Magnetic heading:

- **Heading** (degrees): `data`
- Range: 0-360, where 0/360 is North

## Viewing Topics

### List all topics:

```bash
ros2 topic list
```

### Echo a specific topic:

```bash
# IMU data
ros2 topic echo /iphone/imu

# Device orientation
ros2 topic echo /iphone/pose

# ARKit 6DOF pose (position + orientation)
ros2 topic echo /iphone/arkit_pose

# GPS location
ros2 topic echo /iphone/gps

# Gravity vector
ros2 topic echo /iphone/gravity

# Compass heading
ros2 topic echo /iphone/compass_heading
```

### Check topic rates:

```bash
ros2 topic hz /iphone/imu
ros2 topic hz /iphone/pose
```

### Topic info:

```bash
ros2 topic info /iphone/imu -v
```

## Recording Data

### Record all ZigSim topics:

```bash
ros2 bag record /iphone/imu /iphone/pose /iphone/arkit_pose /iphone/gps /iphone/gravity /iphone/pressure /iphone/compass_heading /iphone/magnetic_field
```

### Record specific topics:

```bash
# Record only IMU and ARKit pose
ros2 bag record /iphone/imu /iphone/arkit_pose

# Record with custom name
ros2 bag record -o my_recording /iphone/imu /iphone/arkit_pose

# Record all pose data (device + ARKit)
ros2 bag record /iphone/pose /iphone/arkit_pose
```

## Visualization in RViz2

### Launch RViz2:

```bash
rviz2
```

### Configure:

1. Set **Fixed Frame** to `iphone` (or your custom frame_id)
2. Add displays:
   - **Imu**: Add → By topic → `/iphone/imu` → Imu
   - **Device Pose**: Add → By topic → `/iphone/pose` → PoseStamped
   - **ARKit Pose**: Add → By topic → `/iphone/arkit_pose` → PoseStamped
   - **GPS**: Add → By topic → `/iphone/gps` → NavSatFix
3. Configure ARKit Pose display:
   - Set **Axes Length** to 0.1 to visualize orientation
   - Set **Axes Radius** to 0.01
   - This shows both position and orientation of the camera

## Example: Monitor All Data

Create a simple monitor script:

```bash
#!/bin/bash
# monitor_zigsim.sh

# Open multiple terminals with topic echoing
gnome-terminal --tab --title="IMU" -- bash -c "ros2 topic echo /iphone/imu; bash"
gnome-terminal --tab --title="Pose" -- bash -c "ros2 topic echo /iphone/pose; bash"
gnome-terminal --tab --title="GPS" -- bash -c "ros2 topic echo /iphone/gps; bash"
gnome-terminal --tab --title="Gravity" -- bash -c "ros2 topic echo /iphone/gravity; bash"
```

## Frame ID

All topics use the same frame ID (default: `iphone`), configurable via the `frame_id` parameter:

```bash
ros2 launch teleop_iphone teleop.launch.py frame_id:=mobile_device
```

## Troubleshooting

### No topics appearing?

1. Check if node is running: `ros2 node list`
2. Verify ZigSim Pro is streaming (check phone)
3. Check firewall: `sudo ufw allow 3333/udp`

### Topics not updating?

```bash
# Check topic rates (should be >0 Hz)
ros2 topic hz /iphone/imu

# Check for errors in node
ros2 node info /osc_node
```

### Incorrect data?

```bash
# Enable debug logging
ros2 run teleop_iphone osc_node --ros-args --log-level debug
```


## Robot JSON feedback inputs

The display-only feedback bridge subscribes to `/arx5_controller/gripper_state`
(`std_msgs/Float64`, width in metres) and `/go2_x5/slam/odom`
(`nav_msgs/Odometry`, measured body twist). Optional battery, temperature and
robot-specific diagnostics topics are configured in
`config/robot_state_feedback.yaml`. It sends full UDP JSON snapshots to the
current iPhone on port 5567; see [README.md](README.md#robot-feedback-to-iphone)
for freshness, units, source semantics and launch arguments.
