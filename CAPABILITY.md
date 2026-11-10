---
description: Lynx S10 底盘控制（移动、站/趴/导航模式姿态读写）
---

# lynx_quadruped — Lynx S10 wheeled-quadruped chassis primitive

Wraps the DeepRobotics Lynx S10 ("山猫") motion host over its native ROS 2
topics (`drdds/msg/*`) and exposes posture control plus burst-style movement
to the pilot. Use this capability whenever the user asks the robot to stand,
sit, enter navigation mode, or move (forward/backward/rotate).

Unlike the Lite3, the S10 has its own internal computer and onboard
navigation; this primitive only drives the chassis. Sensors (lidar/IMU/camera)
and services (mapping/nav2/speech) are out of scope here.

## Interface (3 tools + 2 topics)

### `robonix/primitive/chassis/posture` — set_posture

Set the S10 to a named motion state by publishing `drdds/msg/MotionState` on
`/MOTION_STATE`. Returns `{success, message}`. `success=false` means the target
state was not observed within `posture_wait_s` seconds.

| posture_name | state | description |
|--------------|-------|-------------|
| `"stand"` (站立) | 1 → 17 | stands up (站立, a transient half-crouch), then switches to RL控制 (17), the operational standing state |
| `"sit"` (趴下) | 4 | sitting posture |
| `"rl"` (RL控制) | 17 | enter RL control directly; also sets the agile nav gait (0x3002) |
| `"damping"` (关节阻尼/软急停) | 2 | joint damping / soft emergency stop |
| `"zero"` (标零) | 5 | zero calibration |

> **底盘运动前的机械臂保护（`arm_home_before_motion`）：** 只要底盘要动，机械臂必须先在
> home `[0,0,0,0,25,0]°`。本部署对以下入口统一生效：
> - `posture("stand"/"sit"/"rl"/"zero")`：先回 home 并确认到位，再切姿态；
> - `move`（MCP 底盘移动）：先回 home，再发 `/NAV_CMD`；
> - `twist_in`（`/cmd_vel`，Nav2 走这个）：臂不在 home 时**发零速挡住**，后台把臂收回
>   home，收不回去就一直不动（Nav2 因无进展失败，但不会翻车）。
>
> `posture("damping")`（软急停）是唯一例外：必须立刻生效，不等臂回 home。
> 机械臂 3 s 内拿不到关节角、或 12 s 内回不到 home 时，底盘运动会被**拒绝**并返回
> `success=false`。站立后如需抓取，请重新调用抓取技能的 `observe`/`grasp`
> （会重新观察；固定观察位是关节角，站/趴姿态下看到的世界位置不同）。

> **Before moving:** `/NAV_CMD` (used by `move`) is honoured only in **导航模式**,
> which is three things together: RL控制 (17) + agile nav gait (0x3002) +
> **使用模式 = 导航 (1)**. Under Robonix control the robot stays in 导航模式: the
> primitive tries to switch it on init and re-asserts it in `posture("stand")`/
> `posture("rl")`. The 使用模式 is switched over the 本体监控协议 — but on this
> robot that port is **TLS 1.3 + mTLS** (30003 answers `certificate required`,
> plain UDP 30004 is silent), so the channel is normally unusable without a
> client certificate. That is why the 使用模式 check is **best-effort**: it never
> blocks `move` (see `usage_mode_strict`). Confirm state with `get_posture`
> (`rl_control`); the 使用模式 itself has to be set on the robot.

### `robonix/primitive/chassis/get_posture` — get_posture

Read the robot's real motion state from `/MOTION_INFO` (read-only). Returns
`{posture, basic_state, available}`:

| field | meaning |
|-------|---------|
| `posture` | `standing` / `sitting` / `rl_control` / `damping` / `boot_damping` / `zeroing` / `idle` / `unknown` |
| `basic_state` | raw drdds `MotionState.state` (1/4/17/…); `-1` when unavailable |
| `available` | `false` if the S10 stream is stale (robot unreachable) |

**Recommended flow:** after `set_posture(...)`, call `get_posture` to confirm.
If `available` is `false`, tell the user posture acquisition failed instead of
guessing.

### `robonix/primitive/chassis/move` — move

Single-shot velocity burst on `/NAV_CMD`, then stop. **Not a continuous
stream.** Requires RL control mode (17).

| field | meaning |
|-------|---------|
| `command.forward_m` | forward distance (m), positive = forward |
| `command.rotate_deg` | in-place yaw rotation (deg), positive = CCW |
| `command.linear_x/y/z`, `angular_x/y/z` | raw velocity (m/s, rad/s) |
| `command.duration_sec` | raw-velocity duration (default 1.0 s) |

Modes (priority): `forward_m` → `rotate_deg` → raw velocity. `forward_m`
drives at `speed_mps` (default 0.2 m/s); `rotate_deg` closes the loop on the
dead-reckoned yaw. Returns JSON `{status, mode, duration_sec, …}` in `status`.

> ⚠️ **Precondition:** the robot must be in 导航模式 — RL控制 (17) **and** the
> agile nav gait (0x3002/0x3003) **and** 使用模式=导航(1). `posture("stand")`/
> `posture("rl")` handle the first two and try the third; `/NAV_CMD` is ignored
> outside 导航模式. `move` re-checks state+gait from `/MOTION_INFO` and returns
> `{status:"error"}` if the robot is not in RL control or the nav gait cannot be
> selected. The 使用模式 read over the 本体监控协议 is **best-effort**: when that
> channel is unreachable move still publishes `/NAV_CMD` and reports
> `"usage_mode": "unverified"`. The robot silently ignores the command if it is
> genuinely not in 导航模式, so this cannot cause unexpected motion. Set
> `usage_mode_strict: true` to turn the check back into a hard precondition.

### `robonix/primitive/chassis/twist_in` — continuous velocity (topic)

A standard `geometry_msgs/Twist` topic (default `/cmd_vel`) bridged to the
S10's `/NAV_CMD`. Future navigation services publish Twist here; each message
is converted to `drdds/NavCmd` (x_vel/y_vel/yaw_vel).

### `robonix/primitive/chassis/odom` — odometry (topic)

A `nav_msgs/Odometry` stream (default `/odom`) forwarded from the
`odom_source_topic` (default `/ODOM`, a standard `nav_msgs/Odometry` in the
robot's `map` frame). This capability does **not** compute odometry: it
re-stamps `frame_id` from the robot's LIO `map` frame to `odom_frame` (default
`odom`) and moves the stamp to the host clock, so consumers treat it as
continuous `odom -> base_link` odometry rather than a second `map` frame;
`child_frame_id` is filled with `base_frame` because the S10 leaves it empty.
The native pose is only meaningful once the robot is localised — while idle it
reads garbage (huge x/y/z). The source is configurable via
`odom_source_topic`; `/LIO_ODOM` and `/LIO_ODOM_HIGH_FREQUENCY` are
alternatives that only publish while the LIO stack is running.

For navigation use a continuous external source: the Lynx S10 deployment sets
`odom_source_topic: /fastlio/odom`, where `/fastlio/odom` is produced by the
deployment's FAST-LIO primitive (front Airy lidar + built-in IMU,
`odom -> base_link`). That primitive is robot-specific and ships with the
deployment, not with this package.

## Recommended pilot workflow

1. `posture("stand")`, confirm `get_posture` reports `rl_control`.
2. `move(command={"forward_m": 1.0})` / `move(command={"rotate_deg": 90})`.
3. `posture("sit")` when done, confirm `sitting`.

## Config (deploy manifest `config:`)

| key | default | meaning |
|-----|---------|---------|
| `motion_state_topic` | `/MOTION_STATE` | state transition command topic |
| `motion_info_topic` | `/MOTION_INFO` | motion state/velocity stream |
| `nav_cmd_topic` | `/NAV_CMD` | navigation velocity command topic |
| `gait_topic` | `/GAIT` | gait switch topic |
| `twist_in_topic` | `/cmd_vel` | standard Twist input |
| `odom_topic` | `/odom` | forwarded odometry output |
| `odom_source_topic` | `/ODOM` | native S10 odometry topic to forward |
| `odom_frame` | `odom` | `frame_id` stamped on the forwarded odometry |
| `base_frame` | `base_link` | fills the S10's empty child_frame_id |
| `sentinel_timeout_s` | `30.0` | wait for `/MOTION_INFO` on init |
| `arm_home_before_motion` | `false` | gate chassis motion on the arm being home (defaults to `arm_home_before_stand`) |
| `arm_home_before_stand` | `false` | only gate posture changes (stand/RL) on arm home |
| `arm_joint_command_topic` | `/arm/joint_command` | arm home command topic |
| `arm_joint_states_topic` | `/arm/joint_states_single` | arm joint feedback topic |
| `arm_home_deg` | `[0,0,0,0,25,0]` | arm home pose (deg) |
| `arm_home_tol_deg` | `3.0` | home tolerance |
| `arm_home_timeout_s` | `12.0` | wait for the arm to reach home |
| `arm_state_wait_s` | `3.0` | wait for a joint-state sample |
| `speed_mps` | `0.2` | forward/backward speed |
| `ang_speed_rps` | `0.6` | rotation speed |
| `default_duration_sec` | `1.0` | raw-velocity default duration |
| `rotate_tolerance_deg` | `3.0` | closed-loop yaw tolerance |
| `posture_wait_s` | `8.0` | wait for a posture transition |
| `nav_gait` | `12290` | gait set on RL entry (0x3002 平地敏捷) |
| `usage_mode_ip` | `10.21.33.103` | 本体监控协议 host (使用模式切换) |
| `usage_mode_port` | `30004` | monitor port. On this robot 30003 is TLS 1.3 + **mTLS** (`certificate required`) and plain UDP 30004 never answers, so the channel only works with a client certificate. |
| `usage_mode_transport` | `udp` | `udp` or `tcp` for the usage-mode switch (both need encryption on this robot) |
| `usage_mode` | `1` | 使用模式 set on RL entry (1 = 导航模式) |
| `require_usage_mode_check` | `true` | Try to read/switch 使用模式 over 本体监控协议. Failure is a warning, not an error (see below). Set `false` to skip the attempt entirely. |
| `usage_mode_strict` | `false` | `true` restores the old fail-closed behaviour: `move`/`posture` return an error when 使用模式 cannot be confirmed. Leave `false` when the monitor protocol needs mTLS and no client cert is available. |
| `usage_mode_retry_s` | `300.0` | in best-effort mode, after a failed read/switch do not retry for this many seconds (a dead channel would otherwise add ~4 s to every move). `init` resets the timer; `usage_mode_strict` always retries. |

## What this primitive does NOT do

- It does not correct the S10's odometry (it forwards `/ODOM` as-is, only
  re-stamping the frame name).
- It does not do obstacle avoidance or global path planning.
- It does not expose the S10's lidar/IMU/camera or UDP 本体监控协议.
