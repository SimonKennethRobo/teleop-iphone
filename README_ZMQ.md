# Legacy ZMQ entry point

The old ZMQ implementation sent Cartesian and reset commands directly to an
ARX5 server. That path is not compatible with the Go2-X5 architecture, where
`go2_x5_ocs2` is the only joint-command producer.

`teleop_zmq.launch.py` and `teleop_iphone_zmq.py` remain as compatibility
aliases. They forward to the native `teleop_iphone` ROS node and do not open a
ZMQ connection. Use `teleop.launch.py` for new setups:

```bash
ros2 launch teleop_iphone teleop.launch.py
```
