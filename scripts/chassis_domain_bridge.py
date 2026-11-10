#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""S10 chassis DDS domain bridge.

The S10's onboard computers publish their native ROS 2 graph on
``ROS_DOMAIN_ID=0``: the drdds chassis topics used by ``lynx_quadruped`` *and*
a TF tree (``map -> base_link``, ``base_link -> base_gravity``) stamped with
the robot's own clock, which currently runs tens of seconds ahead of this
host. Sharing that domain with our SLAM stack is not workable:

* tf2 keeps one transform cache per *child frame*. The robot's future-stamped
  ``base_link`` entries make tf2 reject every host-stamped ``odom ->
  base_link`` we publish as ``TF_OLD_DATA``, and once the cache is longer than
  its 10 s history the host-clock entries are pruned out entirely.
* Every RTAB-Map lookup then fails with "Lookup would require extrapolation
  into the past ... earliest data is at <future time>", so no node is ever
  added and ``/map`` stays empty (the live scan preview in the web UI is
  unaffected, which makes the failure look like "point cloud but no map").
* The same domain is also flooded by other machines on the lab network
  publishing their own ``map``/``odom``/``base_link`` TF over Wi-Fi.

The deployment therefore runs on a private domain (manifest env
``ROS_DOMAIN_ID``, default 42). This process is the only bridge between the
two domains. It hosts one rclpy context per domain and relays the chassis
topics ``lynx_quadruped`` needs:

    robot domain (LYNX_ROBOT_DOMAIN_ID, default 0)
        /MOTION_INFO   robot -> private
        /BATTERY_DATA  robot -> private   (for the optional lynx_health)
        /MOTION_STATE  private -> robot   (posture commands)
        /NAV_CMD       private -> robot   (velocity commands)
        /GAIT          private -> robot   (gait selection)

Everything else stays private: the Airy driver is reached over its own UDP
multicast, FAST-LIO consumes ``/airy_201/points`` and ``/airy_201/imu`` from
the local driver, and RTAB-Map / Nav2 only see our own TF tree.
"""
from __future__ import annotations

import logging
import os
import signal
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from drdds.msg import BatteryData, Gait, MotionInfo, MotionState, NavCmd

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[chassis_bridge] %(message)s",
)
log = logging.getLogger("chassis_domain_bridge")

ROBOT_DOMAIN = int(os.environ.get("LYNX_ROBOT_DOMAIN_ID", "0"))
PRIVATE_DOMAIN = int(os.environ.get("ROS_DOMAIN_ID", "42"))

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# (topic, message type, direction); "in" = robot -> private.
BRIDGED = (
    ("/MOTION_INFO", MotionInfo, "in"),
    ("/BATTERY_DATA", BatteryData, "in"),
    ("/MOTION_STATE", MotionState, "out"),
    ("/NAV_CMD", NavCmd, "out"),
    ("/GAIT", Gait, "out"),
)


def _relay(src: Node, dst: Node, topic: str, msg_type, first: set[str]) -> None:
    pub = dst.create_publisher(msg_type, topic, QOS)

    def _forward(msg) -> None:
        pub.publish(msg)
        if topic not in first:
            first.add(topic)
            log.info("relaying %s", topic)

    src.create_subscription(msg_type, topic, _forward, QOS)


def main() -> int:
    robot_ctx = Context()
    rclpy.init(context=robot_ctx, domain_id=ROBOT_DOMAIN)
    private_ctx = Context()
    rclpy.init(context=private_ctx, domain_id=PRIVATE_DOMAIN)

    robot_node = Node("chassis_bridge_robot", context=robot_ctx)
    private_node = Node("chassis_bridge_private", context=private_ctx)

    first: set[str] = set()
    for topic, msg_type, direction in BRIDGED:
        if direction == "in":
            _relay(robot_node, private_node, topic, msg_type, first)
        else:
            _relay(private_node, robot_node, topic, msg_type, first)

    executors = []
    for ctx, node in ((robot_ctx, robot_node), (private_ctx, private_node)):
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        threading.Thread(target=executor.spin, name=node.get_name(), daemon=True).start()
        executors.append(executor)

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    log.info(
        "bridging chassis topics robot domain %d <-> private domain %d",
        ROBOT_DOMAIN,
        PRIVATE_DOMAIN,
    )
    stop.wait()

    for executor in executors:
        executor.shutdown()
    robot_node.destroy_node()
    private_node.destroy_node()
    robot_ctx.shutdown()
    private_ctx.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
