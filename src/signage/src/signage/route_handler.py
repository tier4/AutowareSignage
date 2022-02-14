# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import os
import requests
import json
from rclpy.duration import Duration
import signage.signage_utils as utils
from tier4_external_api_msgs.msg import DoorStatus

EMPTY_CURRENT_TASK = {
    "departure_station": ["", ""],
    "arrival_station": ["", ""],
    "depart_time": 0,
}


class RouteHandler:
    def __init__(self, node, viewController, announceController, autoware_state_interface):
        self._node = node
        self._viewController = viewController
        self._announce_interface = announceController
        self.AUTOWARE_IP = os.getenv("AUTOWARE_IP", "localhost")
        self._fms_payload = {
            "method": "get",
            "url": "https://"
            + os.getenv("FMS_URL", "fms.web.auto")
            + "/v1/projects/{project_id}/environments/{environment_id}/vehicles/{vehicle_id}/active_schedule",
            "body": {},
        }
        self._schedule_updated_time = ""
        self._schedule_id = ""
        self._schedule_type = ""
        self._route_name = ["", ""]
        self._previous_station_name = ["", ""]
        self._next_station_list = [["", ""], ["", ""], ["", ""], ["", ""], ["", ""]]
        self._remain_arrive_time_text = ""
        self._remain_depart_time_text = ""
        self._display_time = False
        self._is_auto_mode = False
        self._is_emergency_mode = False
        self._in_emergency_state = False
        self._emergency_trigger_time = 0
        self._is_stopping = False
        self._is_driving = False
        self._previous_driving_status = False
        self._reach_final = False
        self._prev_autoware_state = ""
        self._prev_prev_autoware_state = ""
        self._announced_going_to_depart = False
        self._announced_going_to_arrive = False
        self._distance = 1000
        self._pre_door_announce_status = DoorStatus.UNKNOWN
        self._current_task_details = {
            "departure_station": ["", ""],
            "arrival_station": ["", ""],
            "depart_time": 0,
        }
        self._fms_check_time = 0
        self.init_seperated_task_lists()

        self._node.declare_parameter("ignore_manual_driving", False)
        self._ignore_manual_driving = (
            self._node.get_parameter("ignore_manual_driving").get_parameter_value().bool_value
        )
        self._node.declare_parameter("check_fms_time", 5)
        self._check_fms_time = (
            self._node.get_parameter("check_fms_time").get_parameter_value().integer_value
        )
        self._node.declare_parameter("ignore_emergency_stoppped", False)
        self._ignore_emergency_stoppped = (
            self._node.get_parameter("ignore_emergency_stoppped").get_parameter_value().bool_value
        )
        self._node.declare_parameter("set_goal_by_distance", False)
        self._set_goal_by_distance = (
            self._node.get_parameter("set_goal_by_distance").get_parameter_value().bool_value
        )
        self._node.declare_parameter("goal_distance", 1)
        self._goal_distance = (
            self._node.get_parameter("goal_distance").get_parameter_value().integer_value
        )
        self._node.declare_parameter("emergency_repeat_period", 180)
        self._emergency_repeat_period = (
            self._node.get_parameter("emergency_repeat_period").get_parameter_value().integer_value
        )

        self.process_station_list_from_fms()

        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        autoware_state_interface.set_control_mode_callback(self.sub_control_mode)
        autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)
        autoware_state_interface.set_distance_callback(self.sub_distance)
        autoware_state_interface.set_door_status_callback(self.sub_door_status)

        route_checker = self._node.create_timer(1, self.route_checker_callback)
        view_mode_update = self._node.create_timer(1, self.view_mode_callback)
        calculate_time = self._node.create_timer(1, self.calculate_time_callback)

    def init_seperated_task_lists(self):
        self._seperated_task_lists = {
            "doing_list": [],
            "todo_list": [],
            "done_list": [],
        }

    # ============== Subsciber callback ==================

    def sub_autoware_state(self, autoware_state):
        if not self._is_auto_mode:
            return

        stop_condition = [
            "WaitingForRoute",
            "ArrivedGoal",
            "Planning",
        ]

        buffer_flag = (
            autoware_state == self._prev_prev_autoware_state
            and autoware_state == self._prev_autoware_state
        )

        self._is_stopping = (autoware_state in stop_condition) or (
            buffer_flag and autoware_state == "WaitingForEngage"
        )
        self._is_driving = autoware_state == "Driving"

        self._prev_prev_autoware_state = self._prev_autoware_state
        self._prev_autoware_state = autoware_state

    def sub_control_mode(self, control_mode):
        self._is_auto_mode = control_mode == 1

    def sub_emergency(self, emergency_stopped):
        if self._ignore_emergency_stoppped:
            self._is_emergency_mode = False
        else:
            self._is_emergency_mode = emergency_stopped

        if self._is_emergency_mode and not self._in_emergency_state:
            self._announce_interface.announce_emergency("emergency")
            self._in_emergency_state = True
        elif not self._is_emergency_mode and self._in_emergency_state:
            self._in_emergency_state = False
        elif self._is_emergency_mode and self._in_emergency_state:
            if not self._emergency_trigger_time:
                self._emergency_trigger_time = self._node.get_clock().now()
            elif self._node.get_clock().now() - self._emergency_trigger_time > Duration(
                seconds=self._emergency_repeat_period
            ):
                self._announce_interface.announce_emergency("in_emergency")
                self._emergency_trigger_time = 0

    def sub_distance(self, distance):
        self._distance = distance

    def sub_door_status(self, door_status):
        if self._pre_door_announce_status == door_status:
            # same announce, return
            return

        if door_status == DoorStatus.OPENING:
            # Should able to give warning everytime the door is opening
            self._announce_interface.send_announce("door_open")
            self._pre_door_announce_status = door_status
        elif door_status == DoorStatus.CLOSING:
            # Should able to give warning everytime the door is closing
            self._announce_interface.send_announce("door_close")
            self._pre_door_announce_status = door_status
        else:
            self._pre_door_announce_status = door_status

    # ============== Subsciber callback ==================

    def process_station_list_from_fms(self):
        try:
            respond = requests.post(
                "http://{}:4711/v1/services/order".format(self.AUTOWARE_IP),
                json=self._fms_payload,
                timeout=5,
            )

            data = json.loads(respond.text)

            if not data:
                raise Exception("No data from fms")
            elif (
                self._schedule_updated_time == data["updated_at"]
                and self._schedule_id == data["schedule_id"]
            ):
                self._fms_check_time = self._node.get_clock().now()
                raise Exception("same schedule, skip")

            # depends on the type of schedule, the route will change
            self._schedule_type = data["schedule_type"]

            self._route_name = utils.get_route_name(
                data.get("tags", []),
            )

            self.init_seperated_task_lists()
            utils.seperate_task_list(self._seperated_task_lists, data.get("tasks", []))

            if not self._seperated_task_lists["doing_list"]:
                raise Exception("doing_list is not found, skip")

            for task in self._seperated_task_lists["doing_list"]:
                utils.process_current_task(self._current_task_details, task)

            if self._previous_station_name == ["", ""] and self._seperated_task_lists["done_list"]:
                self._previous_station_name = utils.get_prevous_station_name_from_fms(
                    self._seperated_task_lists["done_list"]
                )

            self._next_station_list = utils.create_next_station_list(
                self._current_task_details, self._seperated_task_lists, "fms", self._schedule_type
            )
            if self._seperated_task_lists["todo_list"]:
                self._reach_final = False

            self._schedule_updated_time = data["updated_at"]
            self._schedule_id = data["schedule_id"]
            self._fms_check_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().warning("Unable to get the task from FMS, ERROR: " + str(e))

    def arrived_goal(self):
        try:
            if self._current_task_details == EMPTY_CURRENT_TASK:
                raise Exception("No current task details")

            self._previous_station_name = self._current_task_details["departure_station"]

            if not self._seperated_task_lists["todo_list"]:
                # Reach final station
                self._current_task_details["departure_station"] = self._current_task_details[
                    "arrival_station"
                ]
                self._current_task_details["arrival_station"] = ["", ""]
                self._reach_final = True
                return

            next_task = self._seperated_task_lists["todo_list"].pop(0)
            # Get the next task from todo_list
            utils.process_current_task(self._current_task_details, next_task)

            self._next_station_list = utils.create_next_station_list(
                self._current_task_details, self._seperated_task_lists, "local", self._schedule_type
            )
            self._announce_interface.announce_arrived()
        except Exception as e:
            self._node.get_logger().error("Unable to update the goal, ERROR: " + str(e))

    # ========== Timer Callback =============

    def route_checker_callback(self):
        try:
            if not self._fms_check_time:
                self.process_station_list_from_fms()
            elif self._node.get_clock().now() - self._fms_check_time > Duration(
                seconds=self._check_fms_time
            ):
                self.process_station_list_from_fms()

            if self._is_emergency_mode:
                return

            if (
                not self._is_auto_mode
                and self._ignore_manual_driving
                and self._set_goal_by_distance
            ):
                if self._distance < self._goal_distance:
                    self._is_stopping = True
                    self._is_driving = False

            if self._reach_final:
                self._previous_driving_status = False
                return

            if self._is_stopping and self._previous_driving_status:
                self.arrived_goal()
                self._previous_driving_status = False

            if self._is_driving:
                self._previous_driving_status = self._is_driving
        except Exception as e:
            self._node.get_logger().error("Error unable to check the route: " + str(e))

    def calculate_time_callback(self):
        try:
            if self._current_task_details == EMPTY_CURRENT_TASK:
                return

            remain_minute = 100
            remain_minute = int((self._current_task_details["depart_time"]
                                - self._node.get_clock().now().to_msg().sec) / 60)
            if remain_minute > 0:
                self._remain_depart_time_text = "このバスはあと{}分程で出発します".format(str(remain_minute))
            else:
                self._remain_depart_time_text = "間もなく出発します"

            if self._reach_final:
                self._remain_depart_time_text = "終点です。\nご乗車ありがとうございました"
            elif self._is_driving:
                self._announced_going_to_depart = False
                if self._distance < 100 and not self._announced_going_to_arrive:
                    self._announce_interface.announce_going_to_depart_and_arrive("going_to_arrive")
                    self._announced_going_to_arrive = True
                    self._remain_arrive_time_text = "間もなく到着します"
            elif self._is_stopping:
                self._announced_going_to_arrive = False
                if remain_minute < 1 and not self._announced_going_to_depart:
                    self._announce_interface.announce_going_to_depart_and_arrive("going_to_depart")
                    self._announced_going_to_depart = True

            if remain_minute < 5 or self._distance < 100 or self._reach_final:
                self._display_time = True
            else:
                self._display_time = False
        except Exception as e:
            self._node.get_logger().error("Error in getting calculate the time: " + str(e))

    def view_mode_callback(self):
        try:
            self._viewController.route_name = self._route_name
            self._viewController.departure_station_name = self._current_task_details[
                "departure_station"
            ]
            self._viewController.arrival_station_name = self._current_task_details[
                "arrival_station"
            ]
            self._viewController.previous_station_name = self._previous_station_name
            self._viewController.next_station_list = self._next_station_list
            self._viewController.remain_arrive_time_text = self._remain_arrive_time_text
            self._viewController.remain_depart_time_text = self._remain_depart_time_text
            self._viewController.display_time = self._display_time

            if self._is_emergency_mode:
                view_mode = "emergency_stopped"
            elif not self._is_auto_mode and not self._ignore_manual_driving:
                view_mode = "manual_driving"
            elif self._is_stopping and self._current_task_details["departure_station"] != ["", ""]:
                view_mode = "stopping"
            elif self._is_driving and self._current_task_details["arrival_station"] != ["", ""]:
                view_mode = "driving"
            elif self._is_driving:
                view_mode = "auto_driving"
            elif self._current_task_details["departure_station"] != ["", ""]:
                view_mode = "stopping"
            else:
                view_mode = "out_of_service"

            self._viewController.view_mode = view_mode
        except Exception as e:
            self._node.get_logger().error("Error in updating the view mode: " + str(e))
