#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lynx_quadruped — DeepRobotics Lynx S10 wheeled-quadruped chassis primitive.

Owns ``robonix/primitive/chassis/*`` for the S10, mirroring ``lite3_quadruped``.

Unlike the Lite3 (which needs a UDP→ROS2 ``transfer`` bridge), the S10 exposes
its own ROS 2 topics natively through the custom ``drdds`` message package:

    /MOTION_STATE  (out) drdds/msg/MotionState — state 1=stand 4=sit 17=RL nav
    /MOTION_INFO   (in)  drdds/msg/MotionInfo  — velocities + state + gait @20 Hz
    /NAV_CMD       (out) drdds/msg/NavCmd      — x_vel/y_vel/yaw_vel (RL mode only)
    /GAIT          (out) drdds/msg/Gait        — 0x3002 agile nav gait
    /ODOM          (in)  nav_msgs/Odometry     — fused pose, forwarded as odom

The primitive bridges these to the standard Robonix chassis surface:

    twist_in   (geometry_msgs/Twist)  -> /NAV_CMD
    odom       (nav_msgs/Odometry)    <- /ODOM (re-stamped to odom frame)
    posture / get_posture / move      -> /MOTION_STATE + /NAV_CMD

The S10 already publishes its fused pose on ``/ODOM`` (standard
``nav_msgs/Odometry``, frame ``map``); the primitive forwards it to the
declared ``odom`` capability topic, re-stamping ``frame_id`` from the robot's
LIO ``map`` frame to ``odom_frame`` (default ``odom``) so downstream treats it
as continuous ``odom -> base_link`` odometry rather than a second ``map``
frame. The pose is only meaningful once the robot is localised (it reads
garbage while idle). ``/MOTION_INFO`` yaw rate is still
integrated for the internal yaw used by ``rotate_deg``. ``/NAV_CMD`` is only
honoured in 导航模式 — RL控制(17) + agile nav gait (0x3002) + 使用模式=导航(1) —
so ``move()`` checks state 17 + gait from ``/MOTION_INFO`` before moving. The
使用模式 itself is only reachable over the 本体监控协议, which on this robot
speaks TLS+mTLS (30003) or DTLS (30004): without a client certificate it can
neither be read nor set, so that check is best-effort — it warns and still
publishes ``/NAV_CMD`` (set ``usage_mode_strict`` to fail closed instead).
``posture()`` tries to switch it on RL entry for the case the channel works.
"""
from __future__ import annotations

import json
import logging
import math
import os
import socket
import struct
import threading
import time

from rclpy.clock import Clock
from robonix_api import Primitive, Ok, Err, Deferred

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_quadruped] %(message)s",
)
log = logging.getLogger("lynx_quadruped")

lynx_quadruped = Primitive(id="lynx_quadruped", namespace="robonix/primitive/chassis")

# Generated codegen artifacts (paths wired up by robonix_api at import time).
from quadruped_mcp import (  # noqa: E402
    SetPosture_Request,
    SetPosture_Response,
    GetPosture_Request,
    GetPosture_Response,
)
from chassis_mcp import (  # noqa: E402
    ExecuteMoveCommand_Request,
    ExecuteMoveCommand_Response,
)
from std_msgs_mcp import String  # noqa: E402

# S10 native messages, from the prebuilt drdds package on the Jetson.
from drdds.msg import MotionState, MotionInfo, NavCmd, Gait  # noqa: E402

# ── shared state (guarded by _state_cv) ─────────────────────────────────────
_state_cv = threading.Condition()
_motion_lock = threading.Lock()  # serialises move() bursts

_motion_state: int | None = None  # latest drdds MotionState.state
_motion_stamp = 0.0               # monotonic time of the last MotionInfo
_gait: int | None = None          # latest drdds GaitValue.gait (0x1001 std / 0x3002 nav)

_odom_yaw = 0.0                   # integrated yaw (from vel_yaw), for rotate_deg
_last_info_t: float | None = None  # previous sample time, for dt

# publishers / config filled in on_init
nav_cmd_pub = None
motion_state_pub = None
gait_pub = None
odom_pub = None

base_frame = "base_link"
odom_frame = "odom"

# ── 机械臂 home 保护 ────────────────────────────────────────────────────────
# 站立/切 RL 之前先把臂收回 home（[0,0,0,0,25,0]°）。臂伸在外面时站起来重心偏移，
# 实测整个机器人会翻。home 失败就拒绝站起来，不冒险。
_arm_cmd_pub = None
_arm_cv = threading.Condition()
_arm_joints_rad: list[float] | None = None
_arm_gripper_pos = 0.1
_arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_arm_home_before_stand = False
_arm_home_before_motion = False      # 任一底盘运动（posture/move/twist_in）前都先回 home
_arm_at_home = False                 # 由 /arm/joint_states_single 实时更新
_arm_home_deg = [0.0, 0.0, 0.0, 0.0, 25.0, 0.0]
_arm_home_tol_rad = math.radians(3.0)
_arm_home_timeout_s = 12.0
_arm_state_wait_s = 3.0
_motion_home_lock = threading.Lock()
_motion_home_thread = None
_motion_home_last_attempt = 0.0
_motion_home_retry_s = 2.0

_speed_mps = 0.2
_ang_speed_rps = 0.6
_default_dur = 1.0
_rotate_tolerance_deg = 3.0
_posture_wait_s = 8.0
_nav_gait = 0x3002  # 平地敏捷步态, recommended for navigation

# 本体监控协议 (使用模式切换): the S10's 使用模式 (常规0/导航1/辅助2) is not on
# any ROS 2 topic; /NAV_CMD only works in 导航模式, so posture() tries to switch
# it here. Measured on the lab robot: 30003 answers TLS 1.3 + mTLS ("certificate
# required"), 30004/30000-30002 plain UDP never answer — i.e. the channel needs
# a client certificate and is normally unusable without one.
_usage_mode_ip = "10.21.33.103"
_usage_mode_port = 30004          # DTLS; TCP 30003 = TLS + mTLS
_usage_mode_transport = "udp"
_nav_usage_mode = 1               # 使用模式: 1 = 导航模式 (required for /NAV_CMD)
# Whether to try reading/switching the 使用模式 at all. The channel is often
# unusable (this robot answers only TLS/mTLS on 30003, plain UDP 30004 is
# silent), and /NAV_CMD is ignored by the robot outside 导航模式 anyway, so a
# failed check is treated as a warning by default — set usage_mode_strict to
# turn it back into a hard precondition.
_require_usage_mode_check = True
_usage_mode_strict = False        # true: refuse to move when 使用模式 is unverified
# In best-effort mode, a failed attempt is not retried for this long: the
# channel is dead on this robot (mTLS) and each retry costs ~4 s inside a move.
# init() resets the timer, so a config/restart picks up a newly available channel.
_usage_mode_retry_s = 300.0
_usage_mode_failed_at: float | None = None


# ── posture / state tables ──────────────────────────────────────────────────
POSTURE_STATES = {
    "stand": 1, "站立": 1,
    "sit": 4, "趴下": 4, "坐": 4,
    "rl": 17, "rl控制": 17, "nav": 17, "导航": 17,
    "damping": 2, "软急停": 2,
    "zero": 5, "标零": 5,
}

STATE_NAMES = {
    0: "idle",
    1: "standing",
    2: "damping",
    3: "boot_damping",
    4: "sitting",
    5: "zeroing",
    17: "rl_control",
    0x100: "damped_sitting",
}


# ── ROS callbacks ───────────────────────────────────────────────────────────
def _on_motion_info(msg) -> None:
    """Track robot state + gait, and integrate yaw rate for rotate_deg."""
    global _motion_state, _motion_stamp, _odom_yaw, _last_info_t, _gait
    d = msg.data
    now = time.monotonic()
    with _state_cv:
        if _last_info_t is not None:
            dt = max(0.0, now - _last_info_t)
            _odom_yaw += float(d.vel_yaw) * dt
        _last_info_t = now
        _motion_state = int(d.motion_state.state)
        _gait = int(d.gait_state.gait)
        _motion_stamp = now
        _state_cv.notify_all()


def _on_native_odom(msg) -> None:
    """Forward the S10's own /ODOM (nav_msgs/Odometry) to the odom capability.

    The S10 already fuses its pose and publishes it on /ODOM stamped in its own
    LIO ``map`` frame. Downstream (the mapping service's map_to_odom_bridge)
    needs this as the continuous ``odom -> base_link`` navigation odometry, so
    we re-stamp the frame_id to ``odom_frame`` instead of passing the robot's
    ``map`` frame through. The pose is only meaningful once the robot is
    localised (it reads garbage while idle). The S10 leaves child_frame_id
    empty, so we fill it with base_frame for tf consumers.
    """
    if odom_pub is None:
        return
    msg.header.frame_id = odom_frame
    # Re-stamp to the HOST clock. The S10's /ODOM carries the robot's own
    # clock, which drifts ~0.3-0.5 s behind this host. map_to_odom_bridge
    # time-syncs this stream against RTAB-Map's ICP odometry with
    # max_sync_error_s=0.10 s, so a robot-clock stamp makes every sync fail
    # ("no chassis odom sample close enough to the ICP timestamp").
    msg.header.stamp = Clock().now().to_msg()
    if not msg.child_frame_id:
        msg.child_frame_id = base_frame
    odom_pub.publish(msg)


def _on_twist_in(msg) -> None:
    """Forward a standard Twist velocity command to the S10 /NAV_CMD topic.

    只要底盘要动，机械臂就必须先在 home：臂不在 home 时这里**发零速挡住**，
    后台线程把臂收回 home，收不回去就一直不动（Nav2 会因无进展失败，但不会翻车）。
    """
    if nav_cmd_pub is None:
        return
    moving = (abs(float(msg.linear.x)) + abs(float(msg.linear.y))
              + abs(float(msg.angular.z))) > 1e-6
    if moving and _arm_home_before_motion and not _arm_at_home:
        _publish_stop_nav()
        _request_arm_home_async("twist_in")
        return
    cmd = NavCmd()
    cmd.data.x_vel = float(msg.linear.x)
    cmd.data.y_vel = float(msg.linear.y)
    cmd.data.yaw_vel = float(msg.angular.z)
    nav_cmd_pub.publish(cmd)


# ── posture helpers ─────────────────────────────────────────────────────────
def _publish_stop_nav() -> None:
    """发一条零速 /NAV_CMD，让底盘立刻停住。"""
    if nav_cmd_pub is not None:
        nav_cmd_pub.publish(NavCmd())


def _request_arm_home_async(reason: str) -> None:
    """twist_in 高频回调里不能阻塞：起一个后台线程去把臂回 home。

    回 home 期间 twist_in 一律发零速（由调用方做），所以底盘不会动。
    """
    global _motion_home_thread, _motion_home_last_attempt
    now = time.monotonic()
    if _motion_home_thread is not None and _motion_home_thread.is_alive():
        return
    if now - _motion_home_last_attempt < _motion_home_retry_s:
        return
    _motion_home_last_attempt = now

    def worker() -> None:
        ok, msg = _ensure_arm_home(reason)
        if ok:
            log.info("arm home ok before chassis motion (%s): %s", reason, msg)
        else:
            log.warning("arm home FAILED before chassis motion (%s): %s",
                        reason, msg)

    _motion_home_thread = threading.Thread(target=worker, name="arm-home-gate", daemon=True)
    _motion_home_thread.start()


def _send_motion_state(state: int) -> None:
    """Publish a /MOTION_STATE transition a few times (the topic is transient)."""
    msg = MotionState()
    msg.data.state = state
    for _ in range(3):
        motion_state_pub.publish(msg)
        time.sleep(0.2)


def _wait_for_state(target: int, timeout: float) -> int | None:
    deadline = time.monotonic() + timeout
    with _state_cv:
        while time.monotonic() < deadline:
            if _motion_state == target:
                return target
            _state_cv.wait(timeout=min(0.2, deadline - time.monotonic()))
    return _motion_state


def _wait_for_gait(target: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    with _state_cv:
        while time.monotonic() < deadline:
            if _gait == target:
                return True
            _state_cv.wait(timeout=min(0.2, deadline - time.monotonic()))
    return _gait == target


def _publish_nav_gait() -> None:
    """Switch to the agile nav gait (0x3002 平地敏捷) — the navigation motion
    mode that /NAV_CMD requires. Published a few times like /MOTION_STATE, and
    only after the robot is already in RL控制(17)."""
    if gait_pub is None or not _nav_gait:
        return
    g = Gait()
    g.data.gait = _nav_gait
    for _ in range(3):
        gait_pub.publish(g)
        time.sleep(0.2)


# ── 本体监控协议 (usage-mode switch) ──────────────────────────────────────
# The S10 使用模式 (常规0/导航1/辅助2) is not exposed on any ROS 2 topic — it is
# switched over the 本体监控协议. Frame = 16-byte header + JSON ASDU (see the S10
# dev guide §1.1.5; reference impl in robot-deep_robotics-lynx_s10/.sdk/). A
# heartbeat first registers this client (the robot rejects commands from an
# unmatched client), then the mode-switch command, then its ErrorCode response.
_MONITOR_SYNC = b"\xeb\x91\xeb\x90"
_MONITOR_TYPE_MODE_SWITCH = 0x00100002
_MONITOR_CMD_MODE_SWITCH = 0x00500002
_MONITOR_TYPE_HEARTBEAT = 0x00100064
_MONITOR_CMD_HEARTBEAT = 0x00000005
_MONITOR_TYPE_BASIC_STATUS = 0x00100064
_MONITOR_CMD_BASIC_STATUS = 0x00f00000
_USAGE_MODE_READ_S = 2.0             # wait for one 2Hz BasicStatus report


def _monitor_frame(obj: dict, msg_id: int) -> bytes:
    payload = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    header = _MONITOR_SYNC
    header += struct.pack("<H", len(payload))   # ASDU length, little-endian
    header += struct.pack("<H", msg_id)          # 报文 ID, little-endian
    header += bytes([0x01, 0x00, 0x01])          # JSON | 包编号 | 版本
    header += b"\x00" * 5                        # 预留
    return header + payload


def _switch_usage_mode(mode: int) -> tuple[bool, str]:
    """Switch the S10 使用模式 to `mode` (0常规/1导航/2辅助) over the 本体监控协议.

    One-shot over a fresh socket: heartbeat, mode-switch, then read frames until
    the command response (Items.ErrorCode) arrives. Returns (ok, message).
    """
    if not _usage_mode_ip:
        return False, "usage-mode switch disabled (no usage_mode_ip)"
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    hb = _monitor_frame({
        "PatrolDevice": {"Type": _MONITOR_TYPE_HEARTBEAT, "Command": _MONITOR_CMD_HEARTBEAT,
                         "Time": now, "Items": {}}}, 0)
    cmd = _monitor_frame({
        "PatrolDevice": {"Type": _MONITOR_TYPE_MODE_SWITCH, "Command": _MONITOR_CMD_MODE_SWITCH,
                         "Time": now, "Items": {"Mode": mode}}}, 1)

    sock = None
    try:
        if _usage_mode_transport == "tcp":
            sock = socket.create_connection((_usage_mode_ip, _usage_mode_port), timeout=3.0)
            sock.settimeout(1.0)
            sock.sendall(hb)
            sock.sendall(cmd)
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            dst = (_usage_mode_ip, _usage_mode_port)
            sock.sendto(hb, dst)
            sock.sendto(cmd, dst)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                data = sock.recv(4096)
            except (socket.timeout, OSError):
                continue
            if len(data) < 16 or data[:4] != _MONITOR_SYNC:
                continue
            try:
                pd = json.loads(data[16:16 + struct.unpack("<H", data[4:6])[0]]
                                .decode("utf-8")).get("PatrolDevice", {})
            except Exception:
                continue
            items = pd.get("Items", {}) or {}
            if "ErrorCode" in items:
                ec = items.get("ErrorCode", -1)
                if ec == 0:
                    return True, f"usage mode={mode} ok"
                return False, (f"usage mode switch failed "
                               f"(ErrorCode=0x{ec:X} {items.get('ErrorMessage', '')})")
        return False, f"no response from {_usage_mode_ip}:{_usage_mode_port}"
    except OSError as e:
        return False, f"usage-mode switch failed: {e}"
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def _read_usage_mode(timeout: float = _USAGE_MODE_READ_S) -> tuple[int | None, str]:
    """Read the S10 使用模式 (0常规/1导航/2辅助) from its BasicStatus report.

    The robot pushes BasicStatus (2Hz) to whoever sent a heartbeat, so we send
    one heartbeat and read frames until a BasicStatus frame arrives. Returns
    (mode, message); mode is None when nothing was read in time.
    """
    if not _usage_mode_ip:
        return None, "usage-mode read disabled (no usage_mode_ip)"
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    hb = _monitor_frame({
        "PatrolDevice": {"Type": _MONITOR_TYPE_HEARTBEAT, "Command": _MONITOR_CMD_HEARTBEAT,
                         "Time": now, "Items": {}}}, 0)

    sock = None
    try:
        if _usage_mode_transport == "tcp":
            sock = socket.create_connection((_usage_mode_ip, _usage_mode_port), timeout=3.0)
            sock.settimeout(1.0)
            sock.sendall(hb)
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            sock.sendto(hb, (_usage_mode_ip, _usage_mode_port))

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                data = sock.recv(4096)
            except (socket.timeout, OSError):
                continue
            if len(data) < 16 or data[:4] != _MONITOR_SYNC:
                continue
            try:
                pd = json.loads(data[16:16 + struct.unpack("<H", data[4:6])[0]]
                                .decode("utf-8")).get("PatrolDevice", {})
            except Exception:
                continue
            if (pd.get("Type") != _MONITOR_TYPE_BASIC_STATUS
                    or pd.get("Command") != _MONITOR_CMD_BASIC_STATUS):
                continue
            bs = (pd.get("Items") or {}).get("BasicStatus")
            if isinstance(bs, dict) and "ControlUsageMode" in bs:
                return int(bs["ControlUsageMode"]), f"usage mode={bs['ControlUsageMode']}"
        return None, f"no BasicStatus from {_usage_mode_ip}:{_usage_mode_port}"
    except OSError as e:
        return None, f"usage-mode read failed: {e}"
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def _ensure_usage_mode() -> tuple[bool, str]:
    """Best-effort: confirm (and if needed switch) 使用模式 = 导航.

    Returns (confirmed, message). The 使用模式 only exists on the 本体监控协议;
    when that channel is unavailable — this robot only answers TLS+mTLS on 30003
    and stays silent on plain UDP 30004 — it cannot be read or set from here.
    That is NOT a reason to refuse motion: the robot ignores /NAV_CMD outside
    导航模式, so an unverified mode can never cause unexpected movement, only a
    silently ignored command. Callers warn and proceed unless
    ``usage_mode_strict`` is set.

    A failed attempt is remembered for ``usage_mode_retry_s`` so a dead channel
    does not add ~4 s to every single move command.
    """
    global _usage_mode_failed_at
    if not _require_usage_mode_check:
        return True, "usage-mode check disabled; assuming 导航"
    if (not _usage_mode_strict and _usage_mode_failed_at is not None
            and time.monotonic() - _usage_mode_failed_at < _usage_mode_retry_s):
        return False, "使用模式 unverified (monitor protocol unreachable; will retry)"
    mode, _ = _read_usage_mode()
    if mode == _nav_usage_mode:
        _usage_mode_failed_at = None
        return True, f"usage mode={mode}"
    ok, msg = _switch_usage_mode(_nav_usage_mode)
    if ok:
        _usage_mode_failed_at = None
        return True, msg
    _usage_mode_failed_at = time.monotonic()
    log.warning("使用模式 unverified: %s", msg)
    return False, f"使用模式 unverified ({msg})"


def _on_arm_joint_state(msg) -> None:
    """缓存机械臂关节角（弧度），供站立前回 home 用。"""
    global _arm_joints_rad, _arm_gripper_pos
    by_name = {str(n): float(v) for n, v in zip(msg.name, msg.position)}
    try:
        values = [by_name[name] for name in _arm_joint_names]
    except KeyError:
        if len(msg.position) < 6:
            return
        values = [float(v) for v in msg.position[:6]]
    gripper = by_name.get("gripper")
    if gripper is None and len(msg.position) >= 7:
        gripper = float(msg.position[6])
    global _arm_at_home
    home = [math.radians(float(v)) for v in _arm_home_deg]
    at_home = all(abs(c - h) <= _arm_home_tol_rad for c, h in zip(values, home))
    with _arm_cv:
        _arm_joints_rad = values
        if gripper is not None:
            _arm_gripper_pos = gripper
        _arm_at_home = at_home
        _arm_cv.notify_all()


def _ensure_arm_home(reason: str = "") -> tuple[bool, str]:
    """底盘要运动前把臂回 home；配置关闭时直接返回成功。

    Returns:
        (是否成功, 说明)。失败时调用方必须拒绝底盘运动。
    """
    global _arm_joints_rad, _arm_at_home
    if not _arm_home_before_motion:
        return True, ""
    if _arm_cmd_pub is None:
        return False, "arm_home_before_motion 开了，但机械臂命令 publisher 没建起来"

    deadline = time.monotonic() + _arm_state_wait_s
    with _arm_cv:
        while _arm_joints_rad is None and time.monotonic() < deadline:
            _arm_cv.wait(timeout=min(0.5, max(0.0, deadline - time.monotonic())))
        current = list(_arm_joints_rad) if _arm_joints_rad is not None else None
    if current is None:
        return False, ("%s 上 %ss 没收到机械臂关节角（piper_ctl 在跑吗？）——"
                       "不确定臂在哪就不站起来" % ("/arm/joint_states_single", _arm_state_wait_s))

    home = [math.radians(float(v)) for v in _arm_home_deg]
    if all(abs(c - h) <= _arm_home_tol_rad for c, h in zip(current, home)):
        _arm_at_home = True
        return True, "机械臂已经在 home（%s）" % reason

    from sensor_msgs.msg import JointState  # type: ignore
    steps = 240          # 4.8 s 回 home，和 arm_home.py 的节奏一致，别太猛
    dt = 0.02
    names = _arm_joint_names + ["gripper"]
    for i in range(1, steps + 1):
        a = i / float(steps)
        positions = [(1.0 - a) * c + a * h for c, h in zip(current, home)]
        msg = JointState()
        msg.name = names
        msg.position = positions + [float(_arm_gripper_pos)]
        _arm_cmd_pub.publish(msg)
        time.sleep(dt)

    deadline = time.monotonic() + _arm_home_timeout_s
    with _arm_cv:
        while time.monotonic() < deadline:
            if (_arm_joints_rad is not None
                    and all(abs(c - h) <= _arm_home_tol_rad
                            for c, h in zip(_arm_joints_rad, home))):
                _arm_at_home = True
                return True, "机械臂已回 home（%s）" % reason
            _arm_cv.wait(timeout=0.2)
    return False, ("机械臂 %.1fs 内没回到 home（姿态可能被挡住）——"
                   "拒绝站起来，先把臂摆顺再试" % _arm_home_timeout_s)


# ── posture control (MCP) ───────────────────────────────────────────────────
@lynx_quadruped.mcp(
    "robonix/primitive/chassis/posture",
    description=(
        'Switch the Lynx S10 motion state. posture_name: "stand"(站立, ends in '
        'RL控制), "sit"(趴下), "rl"(RL控制), "damping"(软急停), "zero"(标零). '
        'move only works after stand/rl. Any posture change that moves the '
        'chassis first sends the arm to home (an extended arm can tip the '
        'robot); if the arm cannot reach home the posture change is refused. '
        'damping (soft emergency stop) is the only exception and takes effect '
        'immediately. After standing, re-run observation (s10_grasp_dev/'
        'observe or grasp) before grasping.'
    ),
)
def set_posture(req: SetPosture_Request) -> SetPosture_Response:
    """Set the Lynx S10 to a named motion state.

    posture_name -> drdds MotionState.state:
      "stand"  (站立)   -> 1, then immediately 17. 站立(1) is only a transient
                           (标准运动模式, a half-crouch); the operational
                           standing state is RL控制(17). The dev guide requires
                           switching to RL控制 right after standing.
      "sit"    (趴下)   -> 4
      "rl"     (RL控制) -> 17   (also sets the agile nav gait 0x3002)
      "damping" (关节阻尼/软急停) -> 2
      "zero"   (标零)   -> 5

    /NAV_CMD (used by move) only works in 导航模式: RL控制(17) + nav gait
    0x3002 + 使用模式=导航(1). "stand"/"rl" end there — both switch the gait,
    then the 使用模式 over the 本体监控协议."""
    raw = (req.posture_name or "").strip()
    state = POSTURE_STATES.get(raw) or POSTURE_STATES.get(raw.lower())
    if state is None:
        return SetPosture_Response(
            success=False,
            message=(
                f"unknown posture {req.posture_name!r}; valid: "
                "stand(站立), sit(趴下), rl(RL控制), damping(软急停), zero(标零)"
            ),
        )

    if state != 2:  # damping(2) 是软急停，必须立刻生效，不能等臂回 home
        home_ok, home_msg = _ensure_arm_home("posture:%s" % raw)
        if not home_ok:
            return SetPosture_Response(
                success=False,
                message="拒绝底盘运动（%s）：%s" % (raw, home_msg),
            )
        if home_msg:
            log.info("pre-posture arm home: %s", home_msg)

    if state == 1:
        # 站立(1) only stands the robot up into 标准运动模式 (a half-crouch).
        # The real standing/operational state is RL控制(17); chain straight to it.
        _send_motion_state(1)
        _wait_for_state(1, _posture_wait_s)
        time.sleep(0.3)
        _send_motion_state(17)
        target = 17
    else:
        _send_motion_state(state)
        target = state

    observed = _wait_for_state(target, _posture_wait_s)

    gait_ok = True
    mode_ok = True
    mode_msg = ""
    if observed == 17:
        # /NAV_CMD (move) is ignored unless the robot is in RL控制(17) + an agile
        # nav gait AND its 使用模式 is 导航(1). Switch the gait once 17 is actually
        # reached (so /GAIT is not lost during the transition), then switch usage
        # mode over the 本体监控协议.
        _publish_nav_gait()
        gait_ok = _wait_for_gait(_nav_gait, _posture_wait_s)
        if gait_ok:
            mode_ok, mode_msg = _ensure_usage_mode()
            if not mode_ok and not _usage_mode_strict:
                # The 使用模式 cannot be observed, but that must not fail the
                # posture: state+gait already put the robot where move needs it.
                log.warning("%s: %.120s; continuing (usage_mode_strict=false)",
                            raw, mode_msg)
                mode_ok = True  # warning only
                mode_msg = "使用模式 unverified (warning only)"

    if observed == target and gait_ok and mode_ok:
        return SetPosture_Response(
            success=True,
            message=(
                f"{raw} ok (state={target} {STATE_NAMES.get(target, '')}"
                f", gait={hex(_gait) if _gait else 'unknown'}"
                f"{', ' + mode_msg if mode_msg else ''})"
            ),
        )
    if observed != target:
        return SetPosture_Response(
            success=False,
            message=(
                f"sent {raw} (state={target}); not reached within {_posture_wait_s}s "
                f"(observed={observed} {STATE_NAMES.get(observed, '')})"
            ),
        )
    if not gait_ok:
        return SetPosture_Response(
            success=False,
            message=(
                f"reached RL控制(17) but nav gait {hex(_nav_gait)} not confirmed "
                f"(gait={hex(_gait) if _gait else 'unknown'})"
            ),
        )
    return SetPosture_Response(
        success=False,
        message=(
            f"reached RL控制(17) + nav gait but {mode_msg} "
            "(usage_mode_strict=true); move (/NAV_CMD) may be ignored"
        ),
    )


@lynx_quadruped.mcp(
    "robonix/primitive/chassis/get_posture",
    description=(
        'Read the S10 current motion state (read-only). posture: standing/'
        'sitting/rl_control/damping/boot_damping/zeroing/idle/unknown; '
        'available=false means the stream is stale.'
    ),
)
def get_posture(req: GetPosture_Request) -> GetPosture_Response:
    """Read the robot's CURRENT motion state from the /MOTION_INFO stream.

    Read-only: never sends a command. Use this to confirm a posture change
    instead of re-sending set_posture. posture is one of 'standing' (站立),
    'sitting' (趴下), 'rl_control' (导航/RL控制), 'damping', 'boot_damping',
    'zeroing', 'idle', or 'unknown'. If available is false the S10 stream is
    stale — tell the user posture acquisition failed instead of guessing."""
    global _motion_state, _motion_stamp
    available = _motion_state is not None and (time.monotonic() - _motion_stamp) < 1.0
    if not available:
        return GetPosture_Response(posture="unknown", basic_state=-1, available=False)
    return GetPosture_Response(
        posture=STATE_NAMES.get(_motion_state, "unknown"),
        basic_state=_motion_state,
        available=True,
    )


# ── move (MCP) ─────────────────────────────────────────────────────────────
def _move_response(obj) -> ExecuteMoveCommand_Response:
    return ExecuteMoveCommand_Response(
        status=String(data=json.dumps(obj))
    )


def _publish_navcmd_for_duration(cmd, duration: float) -> None:
    """Publish /NAV_CMD at 10 Hz for the duration, then always send a stop."""
    try:
        deadline = time.monotonic() + max(0.1, duration)
        while time.monotonic() < deadline:
            nav_cmd_pub.publish(cmd)
            time.sleep(0.1)
    finally:
        nav_cmd_pub.publish(NavCmd())


def _wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _rotate_closed_loop(target_deg: float, max_speed: float):
    """Rotate in place, closing the loop on the integrated yaw."""
    target = math.radians(target_deg)
    tolerance = math.radians(_rotate_tolerance_deg)
    timeout = max(5.0, abs(target) / max(0.1, max_speed) * 2.5 + 2.0)
    started = time.monotonic()
    with _state_cv:
        if _motion_state is None or started - _motion_stamp > 1.0:
            _state_cv.wait(timeout=1.0)
        if _motion_state is None or time.monotonic() - _motion_stamp > 1.0:
            return False, 0.0, 0.0, "motion stream unavailable"
        previous = _odom_yaw

    accumulated = 0.0
    command = NavCmd()
    try:
        while time.monotonic() - started < timeout:
            with _state_cv:
                _state_cv.wait(timeout=0.1)
                if time.monotonic() - _motion_stamp > 1.0:
                    return False, math.degrees(accumulated), time.monotonic() - started, "motion stream became stale"
                current = _odom_yaw
            accumulated += _wrap_angle(current - previous)
            previous = current
            remaining = target - accumulated
            if abs(remaining) <= tolerance:
                return True, math.degrees(accumulated), time.monotonic() - started, ""
            speed = min(max_speed, max(0.15, abs(remaining) * 1.2))
            command.data.yaw_vel = math.copysign(speed, remaining)
            nav_cmd_pub.publish(command)
        return False, math.degrees(accumulated), time.monotonic() - started, "rotation timeout"
    finally:
        nav_cmd_pub.publish(NavCmd())


@lynx_quadruped.mcp(
    "robonix/primitive/chassis/move",
    description=(
        "Move or rotate the Lynx S10 base with a single bounded burst, then "
        "stop. Any chassis motion first sends the arm to home (an extended arm "
        "can tip the robot); if the arm cannot reach home, the move is refused. "
        "The robot must be in RL nav mode first (posture('stand') already ends "
        "there; confirm with get_posture)."
    ),
)
def move(req: ExecuteMoveCommand_Request) -> ExecuteMoveCommand_Response:
    """Move or rotate the Lynx S10 base with one bounded burst, then stop.

    `command` is an object whose fields are used in this priority order:

    - forward_m (float, metres): signed forward/backward distance, moved at the
      configured speed. Example: {"command": {"forward_m": 0.1}}.
    - rotate_deg (float, degrees): signed in-place yaw rotation; positive is
      CCW/left, negative is CW/right. Example: {"command": {"rotate_deg": -30}}.
    - linear_x/y/z + angular_x/y/z (m/s, rad/s) with duration_sec (s): raw
      velocity burst. Only used when forward_m and rotate_deg are both 0.

    Precondition: the robot must be in RL nav mode — state 17 plus the agile
    nav gait (0x3002/0x3003) plus 使用模式=导航(1). posture('stand')/posture('rl')
    set all three; move reads the 使用模式 over the 本体监控协议 when that channel
    answers, but an unreachable monitor protocol does not block /NAV_CMD: the
    robot ignores velocity commands outside 导航模式 anyway, so move publishes
    and reports ``usage_mode: "unverified"`` (set usage_mode_strict to refuse).
    """
    if nav_cmd_pub is None:
        return _move_response({"status": "error", "error": "ROS2 not initialized"})

    home_ok, home_msg = _ensure_arm_home("move")
    if not home_ok:
        return _move_response({"status": "error", "error":
                               "拒绝底盘运动（move）：%s" % home_msg})

    # /NAV_CMD only takes effect in navigation mode, which is three things
    # together: state 17 + agile nav gait (0x3002) + 使用模式=导航(1). The first
    # two come from /MOTION_INFO; the 使用模式 is not on any ROS 2 topic, so it is
    # read over the 本体监控协议 and only switched when it is not already 导航(1).
    if _motion_state != 17:
        return _move_response({
            "status": "error",
            "error": (
                f"robot not in RL control (state={_motion_state}); "
                "call posture('stand') or posture('rl') first"
            ),
        })
    if _gait != _nav_gait:
        # The S10 falls back to the standard gait (0x1001) on its own after a
        # stand/RL transition, so a move issued a while after posture('stand')
        # would otherwise be refused. Re-assert the agile nav gait here once.
        log.info("gait=%s != nav gait %s; re-asserting",
                 hex(_gait) if _gait else "unknown", hex(_nav_gait))
        _publish_nav_gait()
        if not _wait_for_gait(_nav_gait, 2.0):
            return _move_response({
                "status": "error",
                "error": (
                    f"could not switch to nav gait {hex(_nav_gait)} "
                    f"(gait={hex(_gait) if _gait else 'unknown'}); "
                    "call posture('stand') or posture('rl') first"
                ),
            })
    # 使用模式 is best-effort: Nav2 drives the same /NAV_CMD every cycle without
    # consulting it, so blocking move() on an unreachable monitor protocol would
    # only make the MCP path stricter than the navigation path. Warn and send;
    # the robot itself ignores /NAV_CMD outside 导航模式.
    usage_mode_ok, usage_mode_msg = _ensure_usage_mode()
    if not usage_mode_ok:
        if _usage_mode_strict:
            return _move_response({
                "status": "error",
                "error": (
                    f"not in navigation mode (usage_mode_strict: {usage_mode_msg}); "
                    "confirm 使用模式=导航 on the robot, or set usage_mode_strict=false"
                ),
            })
        log.warning("moving with %s", usage_mode_msg)

    msg = req.command
    with _motion_lock:
        cmd = NavCmd()
        actual_rotate_deg = 0.0
        if float(getattr(msg, "forward_m", 0.0)) != 0.0:
            forward_m = float(msg.forward_m)
            sign = 1.0 if forward_m > 0 else -1.0
            cmd.data.x_vel = sign * _speed_mps
            duration = abs(forward_m) / _speed_mps
            mode = "forward_m"
            _publish_navcmd_for_duration(cmd, duration)
        elif float(getattr(msg, "rotate_deg", 0.0)) != 0.0:
            rotate_deg = float(msg.rotate_deg)
            ok, actual_rotate_deg, duration, error = _rotate_closed_loop(
                rotate_deg, _ang_speed_rps
            )
            if not ok:
                return _move_response({
                    "status": "error",
                    "mode": "rotate_deg",
                    "error": error,
                    "requested_rotate_deg": rotate_deg,
                    "actual_rotate_deg": actual_rotate_deg,
                })
            mode = "rotate_deg"
        else:
            cmd.data.x_vel = float(getattr(msg, "linear_x", 0.0))
            cmd.data.y_vel = float(getattr(msg, "linear_y", 0.0))
            cmd.data.yaw_vel = float(getattr(msg, "angular_z", 0.0))
            duration = float(getattr(msg, "duration_sec", 0.0))
            if duration <= 0.0:
                duration = _default_dur
            mode = "velocity"
            _publish_navcmd_for_duration(cmd, duration)

    return _move_response({
        "status": "done",
        "mode": mode,
        "duration_sec": duration,
        "x_vel": cmd.data.x_vel,
        "y_vel": cmd.data.y_vel,
        "yaw_vel": cmd.data.yaw_vel,
        "actual_rotate_deg": actual_rotate_deg,
        "usage_mode": "nav" if usage_mode_ok else "unverified",
    })


# ── lifecycle ───────────────────────────────────────────────────────────────
@lynx_quadruped.on_init
def init(cfg):
    """INACTIVE: open the drdds publishers/subscriptions, await the S10 stream."""
    global nav_cmd_pub, motion_state_pub, gait_pub, odom_pub
    global base_frame, odom_frame
    global _speed_mps, _ang_speed_rps, _default_dur, _rotate_tolerance_deg, _posture_wait_s, _nav_gait
    global _usage_mode_ip, _usage_mode_port, _usage_mode_transport, _nav_usage_mode
    global _require_usage_mode_check, _usage_mode_strict, _usage_mode_retry_s
    global _usage_mode_failed_at

    motion_state_topic = cfg.get("motion_state_topic", "/MOTION_STATE")
    motion_info_topic = cfg.get("motion_info_topic", "/MOTION_INFO")
    nav_cmd_topic = cfg.get("nav_cmd_topic", "/NAV_CMD")
    gait_topic = cfg.get("gait_topic", "/GAIT")
    twist_in_topic = cfg.get("twist_in_topic", "/cmd_vel")
    odom_topic = cfg.get("odom_topic", "/odom")
    odom_source_topic = cfg.get("odom_source_topic", "/ODOM")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    base_frame = str(cfg.get("base_frame", "base_link"))
    odom_frame = str(cfg.get("odom_frame", "odom"))
    _speed_mps = float(cfg.get("speed_mps", os.environ.get("LYNX_SPEED_MPS", "0.2")))
    _ang_speed_rps = float(cfg.get("ang_speed_rps", os.environ.get("LYNX_ANG_SPEED_RPS", "0.6")))
    _default_dur = float(cfg.get("default_duration_sec", os.environ.get("LYNX_DEFAULT_DURATION_SEC", "1.0")))
    _rotate_tolerance_deg = float(cfg.get("rotate_tolerance_deg", os.environ.get("LYNX_ROTATE_TOLERANCE_DEG", "3.0")))
    _posture_wait_s = float(cfg.get("posture_wait_s", 8.0))
    _nav_gait = int(cfg.get("nav_gait", 0x3002))

    # 站立/切 RL 前的机械臂 home 保护（默认关闭；机器人部署里显式打开）
    global _arm_cmd_pub, _arm_home_before_stand, _arm_home_before_motion, _arm_home_deg
    global _arm_home_tol_rad, _arm_home_timeout_s, _arm_state_wait_s
    _arm_home_before_stand = bool(cfg.get("arm_home_before_stand", False))
    _arm_home_before_motion = bool(cfg.get("arm_home_before_motion",
                                           _arm_home_before_stand))
    _arm_home_deg = [float(v) for v in cfg.get("arm_home_deg", [0, 0, 0, 0, 25, 0])]
    _arm_home_tol_rad = math.radians(float(cfg.get("arm_home_tol_deg", 3.0)))
    _arm_home_timeout_s = float(cfg.get("arm_home_timeout_s", 12.0))
    _arm_state_wait_s = float(cfg.get("arm_state_wait_s", 3.0))
    if _arm_home_before_motion:
        from sensor_msgs.msg import JointState  # type: ignore
        arm_cmd_topic = str(cfg.get("arm_joint_command_topic", "/arm/joint_command"))
        arm_states_topic = str(cfg.get("arm_joint_states_topic", "/arm/joint_states_single"))
        _arm_cmd_pub = lynx_quadruped.create_publisher(
            "_arm_joint_command", topic=arm_cmd_topic, msg_type=JointState,
            qos="reliable", declare=False)
        lynx_quadruped.create_subscription(
            "_arm_joint_states", topic=arm_states_topic, msg_type=JointState,
            callback=_on_arm_joint_state, qos="reliable", declare=False)
        log.info("arm home protection ON: %s -> %s, home=%s°",
                 arm_states_topic, arm_cmd_topic, _arm_home_deg)

    # 本体监控协议 (使用模式切换) — plain UDP 30004 / TCP 30003 (encryption off).
    _usage_mode_ip = str(cfg.get("usage_mode_ip", os.environ.get("LYNX_USAGE_MODE_IP", "10.21.33.103")))
    _usage_mode_port = int(cfg.get("usage_mode_port", os.environ.get("LYNX_USAGE_MODE_PORT", "30004")))
    _usage_mode_transport = str(cfg.get("usage_mode_transport", "udp")).lower()
    _nav_usage_mode = int(cfg.get("usage_mode", 1))
    _require_usage_mode_check = bool(cfg.get("require_usage_mode_check", True))
    _usage_mode_strict = bool(cfg.get("usage_mode_strict", False))
    _usage_mode_retry_s = float(cfg.get("usage_mode_retry_s", 300.0))
    _usage_mode_failed_at = None

    # Publishers for the S10's native drdds topics (internal, not declared).
    nav_cmd_pub = lynx_quadruped.create_publisher(
        "_nav_cmd", topic=nav_cmd_topic, msg_type=NavCmd, qos="reliable", declare=False)
    motion_state_pub = lynx_quadruped.create_publisher(
        "_motion_state", topic=motion_state_topic, msg_type=MotionState, qos="reliable", declare=False)
    gait_pub = lynx_quadruped.create_publisher(
        "_gait", topic=gait_topic, msg_type=Gait, qos="reliable", declare=False)

    # Subscribe to the S10 motion state/velocity stream (reliable: the S10
    # publishes it reliable, and a best-effort subscriber would not match).
    lynx_quadruped.create_subscription(
        "_motion_info", topic=motion_info_topic, msg_type=MotionInfo,
        callback=_on_motion_info, qos="reliable", declare=False)

    # twist_in bridge: standard Twist -> drdds NavCmd.
    from geometry_msgs.msg import Twist  # type: ignore
    lynx_quadruped.create_subscription(
        "robonix/primitive/chassis/twist_in", topic=twist_in_topic, msg_type=Twist,
        callback=_on_twist_in, qos="reliable", declare=False)
    lynx_quadruped.declare_ros2_topic(
        "robonix/primitive/chassis/twist_in", twist_in_topic, qos="reliable")

    # odom: forward the S10's native /ODOM (nav_msgs/Odometry) to the declared
    # odom capability topic. The S10 publishes /ODOM reliable (like its drdds
    # topics), so subscribe reliable.
    from nav_msgs.msg import Odometry  # type: ignore
    odom_pub = lynx_quadruped.create_publisher(
        "robonix/primitive/chassis/odom", topic=odom_topic, msg_type=Odometry,
        qos="reliable", declare=False)
    lynx_quadruped.declare_ros2_topic(
        "robonix/primitive/chassis/odom", odom_topic, qos="reliable")
    lynx_quadruped.create_subscription(
        "_odom_source", topic=odom_source_topic, msg_type=Odometry,
        callback=_on_native_odom, qos="reliable", declare=False)

    # Await the S10's motion stream before reporting ready.
    with _state_cv:
        if _motion_state is None:
            _state_cv.wait(timeout=sentinel_timeout)
    if _motion_state is None:
        return Deferred(f"no {motion_info_topic} stream from the S10 within {sentinel_timeout}s")

    # Under Robonix control the S10 stays in 导航模式 (使用模式=1) the whole time
    # — /NAV_CMD (move) needs it and nothing else switches it. Set it now;
    # posture() re-asserts it after each stand/rl in case a transition reset it.
    if _require_usage_mode_check:
        ok, msg = _ensure_usage_mode()
        if ok:
            log.info("usage mode=%d (%s)", _nav_usage_mode, msg)
        else:
            log.warning("initial 使用模式 not confirmed (%s); move() will proceed%s",
                        msg, " — usage_mode_strict refuses" if _usage_mode_strict else "")
    else:
        log.info("usage-mode check disabled by config; assuming 使用模式=%d",
                 _nav_usage_mode)

    log.info(
        "lynx_quadruped ready: state=%s odom=%s<-%s nav_cmd=%s twist_in=%s",
        _motion_state, odom_topic, odom_source_topic, nav_cmd_topic, twist_in_topic,
    )
    return Ok()


@lynx_quadruped.on_shutdown
def shutdown():
    """Send a zero /NAV_CMD so the robot does not keep driving on exit."""
    if nav_cmd_pub is not None:
        try:
            nav_cmd_pub.publish(NavCmd())
        except Exception:  # noqa: BLE001
            pass
    return Ok()


if __name__ == "__main__":
    lynx_quadruped.run()
