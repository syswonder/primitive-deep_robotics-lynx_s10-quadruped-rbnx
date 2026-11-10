#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_quadruped runtime — start the ROS-aware chassis primitive.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# drdds is the S10's custom ROS 2 message package, prebuilt on the Jetson.
# Override with LYNX_DRDDS_INSTALL if it lives elsewhere.
DRDDS_INSTALL="${LYNX_DRDDS_INSTALL:-/home/nvidia/drdds_build_ws/install}"

# ROS setup scripts may reference unset variables; disable nounset temporarily.
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$DRDDS_INSTALL/setup.bash" ]; then
  source "$DRDDS_INSTALL/setup.bash"
fi
set -u

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

# Our ROS 2 stack runs on a private DDS domain (manifest env ROS_DOMAIN_ID);
# the robot's native graph — including a TF tree stamped by the robot's own,
# badly skewed clock — stays on LYNX_ROBOT_DOMAIN_ID (0). Relay the chassis
# topics across the split; see scripts/chassis_domain_bridge.py for why.
# Starts before the primitive so /MOTION_INFO is flowing when it waits for the
# sentinel.
#
# Kill a bridge left behind by an unclean previous run: two bridges publish
# duplicate /MOTION_INFO into the private domain and duplicate commands to the
# robot, which makes the chassis primitive see each sample twice.
kill_stale() {
  local pattern="$1"
  pkill -TERM -f -- "$pattern" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    pgrep -f -- "$pattern" >/dev/null 2>&1 || return 0
    sleep 0.2
  done
  pkill -KILL -f -- "$pattern" 2>/dev/null || true
}
kill_stale "$PKG/scripts/chassis_domain_bridge.py"

python3 "$PKG/scripts/chassis_domain_bridge.py" &
BRIDGE_PID=$!

cleanup() {
  kill "$BRIDGE_PID" 2>/dev/null || true
  wait "$BRIDGE_PID" 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 -m lynx_quadruped.main
