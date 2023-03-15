# !/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.duration import Duration
from tier4_debug_msgs.msg import Float64Stamped
from tier4_api_msgs.msg import AwapiAutowareStatus, AwapiVehicleStatus
from tier4_external_api_msgs.msg import DoorStatus
from autoware_auto_system_msgs.msg import HazardStatusStamped
from autoware_adapi_v1_msgs.msg import RouteState


class AutowareStateInterface:
    def __init__(self, node):
        self.autoware_state_callback_list = []
        self.control_mode_callback_list = []
        self.emergency_stopped_callback_list = []

        self.door_status_callback_list = []
        self.turn_signal_callback_list = []
        self.velocity_callback_list = []
        self.distance_callback_list = []
        self.routing_state_callback_list = []
        self._node = node

        self._sub_autoware_state = node.create_subscription(
            AwapiAutowareStatus, "/awapi/autoware/get/status", self.autoware_state_callback, 10
        )
        self._sub_vehicle_state = node.create_subscription(
            AwapiVehicleStatus, "/awapi/vehicle/get/status", self.vehicle_state_callback, 10
        )
        self._sub_vehicle_state = node.create_subscription(
            DoorStatus, "/api/external/get/door", self.vehicle_door_callback, 10
        )
        self._sub_hazard_status = node.create_subscription(
            HazardStatusStamped,
            "/system/emergency/hazard_status",
            self.sub_hazard_status_callback,
            10,
        )
        self._sub_path_distance = node.create_subscription(
            Float64Stamped,
            "/autoware_api/utils/path_distance_calculator/distance",
            self.path_distance_callback,
            10,
        )
        self._sub_routing_state = node.create_subscription(
            RouteState, "/api/routing/state", self.sub_routing_state_callback, 10
        )
        self._autoware_status_time = self._node.get_clock().now()
        self._vehicle_status_time = self._node.get_clock().now()
        self._distance_time = self._node.get_clock().now()

        self._topic_checker = self._node.create_timer(1, self.topic_checker_callback)

    def topic_checker_callback(self):
        if self._node.get_clock().now() - self._distance_time > Duration(seconds=1):
            for callback in self.distance_callback_list:
                callback(1000)

        if self._node.get_clock().now() - self._autoware_status_time > Duration(seconds=5):
            for callback in self.autoware_state_callback_list:
                callback("")

            for callback in self.control_mode_callback_list:
                callback(0)

            for callback in self.emergency_stopped_callback_list:
                callback(False)

        if self._node.get_clock().now() - self._vehicle_status_time > Duration(seconds=5):
            for callback in self.velocity_callback_list:
                callback(0)

    # set callback
    def set_autoware_state_callback(self, callback):
        self.autoware_state_callback_list.append(callback)

    def set_control_mode_callback(self, callback):
        self.control_mode_callback_list.append(callback)

    def set_emergency_stopped_callback(self, callback):
        self.emergency_stopped_callback_list.append(callback)

    def set_turn_signal_callback(self, callback):
        self.turn_signal_callback_list.append(callback)

    def set_velocity_callback(self, callback):
        self.velocity_callback_list.append(callback)

    def set_door_status_callback(self, callback):
        self.door_status_callback_list.append(callback)

    def set_distance_callback(self, callback):
        self.distance_callback_list.append(callback)

    def set_routing_state_callback(self, callback):
        self.routing_state_callback_list.append(callback)

    # ros subscriber
    # autoware stateをsubしたときの処理
    def autoware_state_callback(self, topic):
        try:
            self._autoware_status_time = self._node.get_clock().now()
            autoware_state = topic.autoware_state
            control_mode = topic.control_mode
            emergency_stopped = topic.emergency_stopped

            for callback in self.control_mode_callback_list:
                callback(control_mode)

            for callback in self.autoware_state_callback_list:
                callback(autoware_state)
        except Exception as e:
            self._node.get_logger().error("Unable to get the autoware state, ERROR: " + str(e))

    # vehicle doorをsubしたときの処理
    def vehicle_door_callback(self, topic):
        try:
            door_status = topic.status

            for callback in self.door_status_callback_list:
                callback(door_status)
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle door status, ERROR: " + str(e))

    # vehicle stateをsubしたときの処理
    def vehicle_state_callback(self, topic):
        try:
            self._vehicle_status_time = self._node.get_clock().now()
            turn_signal = topic.turn_signal
            velocity = topic.velocity
            steering = topic.steering

            for callback in self.turn_signal_callback_list:
                callback(turn_signal)
            for callback in self.velocity_callback_list:
                callback(velocity)
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def path_distance_callback(self, topic):
        try:
            self._distance_time = self._node.get_clock().now()
            distance = topic.data
            for callback in self.distance_callback_list:
                callback(distance)
        except Exception as e:
            self._node.get_logger().error("Unable to get the distance, ERROR: " + str(e))

    def sub_hazard_status_callback(self, topic):
        try:
            emergency_stopped = topic.status.emergency
            for callback in self.emergency_stopped_callback_list:
                callback(emergency_stopped)
        except Exception as e:
            self._node.get_logger().error("Unable to get the hazard_status, ERROR: " + str(e))

    def sub_routing_state_callback(self, topic):
        try:
            routing_state = topic.state
            for callback in self.routing_state_callback_list:
                callback(routing_state)
        except Exception as e:
            self._node.get_logger().error("Unable to get the routing state, ERROR: " + str(e))
