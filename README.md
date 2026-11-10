# primitive-lynx_quadruped

Robonix chassis primitive for the DeepRobotics Lynx S10 ("山猫") wheeled
quadruped. Like `primitive-deep_robotics-lite3_quadruped` it exposes the
standard chassis contract set, but it drives the S10 through its **native ROS 2
`drdds` topics** instead of a UDP→ROS2 transfer bridge:

| S10 topic | type | direction |
|-----------|------|-----------|
| `/MOTION_STATE` | `drdds/msg/MotionState` | out (stand/sit/RL) |
| `/MOTION_INFO`  | `drdds/msg/MotionInfo`  | in (velocities + state @20 Hz) |
| `/NAV_CMD`      | `drdds/msg/NavCmd`      | out (x/y/yaw velocity) |
| `/GAIT`         | `drdds/msg/Gait`        | out (agile nav gait) |
| `/ODOM`         | `nav_msgs/Odometry`     | in (fused pose, re-stamped to `odom` frame) |

Exposed Robonix capabilities: `chassis/driver`, `chassis/move`,
`chassis/twist_in`, `chassis/odom`, `chassis/posture`, `chassis/get_posture`.

## Odometry

`chassis/odom` is a **forwarder**: it subscribes `odom_source_topic`
(default `/ODOM`) and republishes it on `odom_topic` (`/odom`) with `frame_id`
re-stamped to `odom` and the stamp moved to the host clock. It does not
estimate odometry itself.

The S10's native `/ODOM` is the robot's onboard LIO-fused pose: it is stamped
in the robot's `map` frame, carries the robot's own clock, and is only
meaningful once the robot is localised (while idle it reads garbage). For
navigation, point the primitive at a continuous external odometry source
instead. In the Lynx S10 deployment that is the FAST-LIO primitive:

```yaml
odom_source_topic: /fastlio/odom
```

`/fastlio/odom` is computed by the deployment's FAST-LIO primitive (front Airy
lidar + its built-in IMU, re-expressed as `odom -> base_link`). That primitive
is robot-specific and ships with the deployment; this package only forwards
whatever `odom_source_topic` carries.

## Deployment notes

- **DDS domains.** The S10's native graph runs on `ROS_DOMAIN_ID=0`, and its
  own `map -> base_link` TF is stamped with a robot clock that drifts from the
  host. The deployment runs the Robonix stack on a private domain and relays
  the chassis topics across the split with `scripts/chassis_domain_bridge.py`
  (started by `scripts/start.sh`; configured by `ROS_DOMAIN_ID` and
  `LYNX_ROBOT_DOMAIN_ID`).
- **Arm home gate (`arm_home_before_motion`).** When enabled, the primitive
  commands the arm to its home pose and refuses to move the chassis until
  `/arm/joint_states_single` reports the home pose; `twist_in` publishes zero
  velocity while the arm is not home. This keeps the robot from tipping over
  with the arm extended. Only `posture("damping")` (soft e-stop) bypasses the
  gate.
- `drdds` is the S10's custom ROS 2 message package, prebuilt on the robot at
  `/home/nvidia/drdds_build_ws`; the deploy manifest sets `LYNX_DRDDS_INSTALL`
  to its install prefix.

## Build & run

```sh
# build (codegen only; drdds is prebuilt on the robot)
rbnx codegen -p . --mcp

# run (sources /opt/ros/humble and the prebuilt drdds install)
bash scripts/start.sh
```

See `CAPABILITY.md` for the tool contract and the required stand → RL sequence
before movement.
