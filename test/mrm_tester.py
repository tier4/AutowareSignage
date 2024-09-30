# -*- coding: utf-8 -*-
# Copyright (c) 2022 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from autoware_adapi_v1_msgs.msg import OperationModeState, MrmState


class AutowareMRMTester(Node):
    def __init__(self):
        super().__init__("autoware_mrm_tester")
        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.operation_mode_publisher_ = self.create_publisher(
            OperationModeState, "/api/operation_mode/state", api_qos
        )
        self.mrm_publisher_ = self.create_publisher(MrmState, "/api/fail_safe/mrm_state", api_qos)
        self._mrm_behavior = MrmState.EMERGENCY_STOP
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        operation_mode = OperationModeState()
        operation_mode.mode = OperationModeState.AUTONOMOUS
        operation_mode.is_autoware_control_enabled = True
        self.operation_mode_publisher_.publish(operation_mode)

        mrm_state = MrmState()
        if self._mrm_behavior == MrmState.EMERGENCY_STOP:
            self._mrm_behavior = MrmState.NONE
        else:
            self._mrm_behavior = MrmState.EMERGENCY_STOP
        print(self._mrm_behavior)
        mrm_state.behavior = self._mrm_behavior
        self.mrm_publisher_.publish(mrm_state)


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(AutowareMRMTester())

    rclpy.shutdown()


if __name__ == "__main__":
    main()
