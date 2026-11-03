# Jetson ↔ ArmPi Pro bridge

This directory contains the ROS-version boundary for an ArmPi Pro whose Pi runs
ROS 1 Noetic while a Jetson runs ROS 2 Jazzy.  Camera and VLA workloads stay on
the Jetson; the Pi remains the only process that talks to the ArmPi I2C motor
board and serial servo controller.

The connection is one TCP socket with newline-delimited JSON (`NDJSON`), port
`8765` by default.  Every command has an `id`, and replies/telemetry use the
same JSON envelope.  TCP is reliable and ordered; the id lets callers match
asynchronous results and makes logs debuggable.

## Layout

```
protocol/                 Pure-Python serializer, deserializer, validation
ros1_ws/src/armpi_jetson_bridge/   Pi TCP server + ROS 1 hardware adapter
ros2_ws/src/jetson_armpi_bridge/   Jetson TCP client + ROS 2 adapter
network/                  Example static Ethernet configuration
tests/                    Protocol-only tests; no ROS or hardware required
```

## Protocol

Commands sent Jetson → Pi:

| Type | Payload |
| --- | --- |
| `base_velocity` | `vx_mps`, `vy_mps`, `wz_radps` |
| `arm_pose` | `position_m: [x,y,z]`, `pitch_deg`, `duration_s` |
| `gripper` | `command: open/close` or `pulse`, `duration_s` |
| `stop` | `{}` |
| `get_state` | `{}` |
| `heartbeat` | `{}` |

The ArmPi Pro vendor IK supports Cartesian position plus **pitch**, not an
arbitrary 6-DOF orientation.  `arm_pose` is intentionally pitch-only.  Wrist
rotation and gripper calibration are separate hardware controls.

The Pi replies with `accepted`, `rejected`, `state`, and periodic `telemetry`.
`state.pose_is_estimated` is currently true: the vendor servo configuration
sets `fake_read: true`, which yields commanded rather than physically measured
positions.

## ROS topics

### Jetson (ROS 2)

- subscribes `/armpi/cmd_vel` (`geometry_msgs/Twist`): `linear.x/y`,
  `angular.z` are forwarded as chassis velocity
- subscribes `/armpi/arm_pose_cmd` (`geometry_msgs/PoseStamped`): position and
  quaternion pitch are forwarded; non-zero roll/yaw are warned because the arm
  cannot satisfy arbitrary orientation
- subscribes `/armpi/gripper_cmd` (`std_msgs/String`): `open` or `close`
- subscribes `/armpi/stop` (`std_msgs/Empty`)
- publishes `/armpi/status` (`std_msgs/String`), `/armpi/state`
  (`std_msgs/String`), and `/armpi/pose_estimated` (`geometry_msgs/PoseStamped`)
- provides `/armpi/get_state` (`std_srvs/Trigger`); the resulting state is
  published on `/armpi/state`

### Pi (ROS 1)

The Pi node publishes vendor messages to:

- `/chassis_control/set_velocity` (`chassis_control/SetVelocity`)
- `/servo_controllers/port_id_1/multi_id_pos_dur`
  (`hiwonder_servo_msgs/MultiRawIdPosDur`)

It subscribes to `/servo_controllers/port_id_1/servo_states` for telemetry.

## Static Ethernet example

The supplied networkd examples use an isolated cable network:

| Host | Address |
| --- | --- |
| Jetson | `192.168.50.10/24` |
| Pi | `192.168.50.20/24` |

Install the matching `.network` file as `/etc/systemd/network/20-armpi-ethernet.network`
on each host and adjust `Name=` to its Ethernet interface (often `eth0`). Do
not install both files on one machine. The node defaults match these values.

## Build and run

Copy this directory to both computers, or keep it in a shared repository.  The
small `protocol/` directory must be on `PYTHONPATH` for both nodes.

On the Pi (after sourcing its Noetic workspace and launching the vendor servo
controller and `chassis_control`):

```bash
cd /path/to/Jetson_ArmPi_Communication/ros1_ws && catkin_make
source devel/setup.bash
export PYTHONPATH=/path/to/Jetson_ArmPi_Communication/protocol:$PYTHONPATH
rosrun armpi_jetson_bridge armpi_to_jetson.py _bind_host:=192.168.50.20 _port:=8765
```

On the Jetson:

```bash
cd /path/to/Jetson_ArmPi_Communication/ros2_ws
colcon build --packages-select jetson_armpi_bridge
. install/setup.bash
export PYTHONPATH=/home/local/ros2_ws/src/Jetson_ArmPi_Communication/protocol:$PYTHONPATH
ros2 run jetson_armpi_bridge jetson_to_armpi --ros-args \
  -p pi_host:=192.168.50.20 -p port:=8765
```

Run `python3 -m unittest discover -s tests -v` from this directory to test
protocol framing and validation locally.

## Experimental wheel odometry

The Pi runs ROS 1 Melodic and Python 3. The wheel node and bridge changes are
local source only until copied or pulled onto the Pi and rebuilt. The motor
board protocol and calibration remain unverified; do not use for navigation.

The diagnostic selects candidate register `0x3c` on bus 1, address `0x34`,
then reads 16 bytes as four signed little-endian counts. Register selection is
an I2C write transaction, but no count-reset, configuration or motor commands
are sent. First confirm the board is compatible and powered. With the base
safely supported, start the diagnostic and manually turn one wheel at a time:

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/Jetson_ArmPi_Communication/ros1_ws/devel/setup.bash
rosrun armpi_jetson_bridge encoder_diagnostic.py
```

Record stationary readings, channel and sign for each wheel, counts per full
wheel turn in each direction, and whether counts persist across a power cycle.
Errors must not be interpreted as zero movement. Coordinate access with the
chassis driver; both use I2C bus 1. Do not write to 0x3c: some boards reset
counts when data is written to that register.

After physical calibration, configure the ROS 1 node with private parameters:
`calibration_verified: true`, `channel_order` (four 1-based channels in
front-left/front-right/rear-left/rear-right order), `wheel_signs` (four ±1
values for positive forward wheel travel), `wheel_radius_m`,
`ticks_per_revolution`, `wheelbase_sum_m` (centre-to-axle plus centre-to-wheel
spacing), and `max_ticks_per_second`. `rate_hz` defaults to 20 and `i2c_bus`
to 1. All geometry, channel assignment and signs require measurement. The node
starts disabled until calibration is explicitly verified; it stops on read or
count discontinuity and must be restarted/relocalized. It emits
`/wheel/odometry` with `wheel_odom` and `base_link` frames, but never TF.
Covariances are provisional placeholders, not measured uncertainties.

The bridge forwards the full odometry message with its Pi source timestamp.
Synchronize Pi/Jetson clocks and verify offset, rates, reconnect and stop
behavior before using it. TCP odometry shares the existing command connection;
its effect on latency and the existing watchdog remains untested. The Jetson
SLAM workspace publishes the wheel TF and must be its only publisher.
