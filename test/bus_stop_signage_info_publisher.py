#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug publisher for /v2x/bus_stop/signage_info.

signage は source 不要。pilot-auto のみ source して実行:

  source /path/to/pilot-auto/install/setup.bash
  python3 test/bus_stop_signage_info_publisher.py

配列は下の SIGNAGE_INFOS / PUBLISH_FRAMES を編集して使う。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tier4_v2x_msgs.msg import BusStopSignageInfoArray, BusStopStatus, BusStopState

DEFAULT_TOPIC = "/v2x/bus_stop/signage_info"

# ============================================================
# ここを編集して publish 内容を変える
# state: BusStopState.* を指定
#   UNSET, WILL_PASS, WILL_STOP, APPROACHING, STOPPING,
#   STOP_COMPLETED, PASSING, PASS_COMPLETED
# ============================================================

SIGNAGE_INFOS = [
    {
        "stop_id": 1,
        "name": "始発駅",
        "state": BusStopState.STOP_COMPLETED,
        "will_stop": True,
    },
    {
        "stop_id": 2,
        "name": "中央駅",
        "state": BusStopState.STOP_COMPLETED,
        "will_stop": True,
    },
    {
        "stop_id": 3,
        "name": "公園前",
        "state": BusStopState.WILL_PASS,
        "will_stop": False,
    },
    {
        "stop_id": 4,
        "name": "和泉多摩川駅",
        "state": BusStopState.WILL_STOP,
        "will_stop": True,
    },
]

# 未使用なら空リストのまま
BUS_STOPS = []

# 複数フレームを順に publish したい場合はここに追加（空なら SIGNAGE_INFOS のみ）
PUBLISH_FRAMES = [
    # 例: 中央駅に接近中
    # [
    #     {"stop_id": 1, "name": "始発駅", "state": BusStopState.STOP_COMPLETED, "will_stop": True},
    #     {"stop_id": 2, "name": "中央駅", "state": BusStopState.APPROACHING, "will_stop": True},
    #     {"stop_id": 3, "name": "公園前", "state": BusStopState.WILL_STOP, "will_stop": True},
    #     {"stop_id": 4, "name": "終点", "state": BusStopState.WILL_STOP, "will_stop": True},
    # ],
]


def build_bus_stop_status(item):
    status = BusStopStatus()
    status.stop_id = item["stop_id"]
    status.name = item["name"]
    status.state.value = item["state"]
    status.will_stop = item.get("will_stop", True)
    status.last_updated_at = item.get("last_updated_at", int(time.time()))
    return status


def build_message(signage_infos_data, bus_stops_data, stamp):
    msg = BusStopSignageInfoArray()
    msg.stamp = stamp
    msg.signage_infos = [build_bus_stop_status(item) for item in signage_infos_data]
    msg.bus_stops = [build_bus_stop_status(item) for item in bus_stops_data]
    return msg


def get_publish_frames():
    if PUBLISH_FRAMES:
        return PUBLISH_FRAMES
    return [SIGNAGE_INFOS]


class BusStopSignageInfoPublisher(Node):
    def __init__(self, topic, rate, once):
        super().__init__("bus_stop_signage_info_publisher")
        self._frames = get_publish_frames()
        self._frame_index = 0
        self._once = once

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(BusStopSignageInfoArray, topic, qos)
        self._publish()

        if once:
            return

        self._timer = self.create_timer(rate, self._on_timer)
        self.get_logger().info(
            "Publishing to {} ({} frame(s), rate={:.1f}s)".format(
                topic, len(self._frames), rate
            )
        )

    def _publish(self):
        frame = self._frames[self._frame_index]
        msg = build_message(frame, BUS_STOPS, self.get_clock().now().to_msg())
        self._publisher.publish(msg)
        names = [item["name"] for item in frame]
        self.get_logger().info(
            "Published frame {}/{}: {}".format(
                self._frame_index + 1, len(self._frames), names
            )
        )

    def _on_timer(self):
        if len(self._frames) > 1:
            self._frame_index = (self._frame_index + 1) % len(self._frames)
        self._publish()


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Publish hardcoded BusStopSignageInfoArray messages"
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help="Topic name (default: {})".format(DEFAULT_TOPIC),
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Publish interval [s] (default: 1.0)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish once and exit",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    rclpy.init(args=argv)

    node = BusStopSignageInfoPublisher(args.topic, args.rate, args.once)
    if args.once:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
