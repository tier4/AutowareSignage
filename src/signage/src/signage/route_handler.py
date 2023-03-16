# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import os
import requests
import json
from datetime import datetime
from rclpy.duration import Duration
import signage.signage_utils as utils
from tier4_external_api_msgs.msg import DoorStatus
from autoware_adapi_v1_msgs.msg import RouteState, MrmState, OperationModeState


class RouteHandler:
    def __init__(
        self, node, viewController, announceController, autoware_interface, parameter_interface
    ):
        self._node = node
        self._viewController = viewController
        self._announce_interface = announceController
        self._autoware = autoware_interface
        self._parameter = parameter_interface.parameter
        self.AUTOWARE_IP = os.getenv("AUTOWARE_IP", "localhost")
        self._fms_payload = {
            "method": "get",
            "url": "https://"
            + os.getenv("FMS_URL", "fms.web.auto")
            + "/v1/projects/{project_id}/environments/{environment_id}/vehicles/{vehicle_id}/active_schedule",
            "body": {},
        }
        self._schedule_details = utils.init_ScheduleDetails()
        self._display_details = utils.init_DisplayDetails()
        self._current_task_details = utils.init_CurrentTask()
        self._task_list = utils.init_TaskList()
        self._remain_arrive_time_text = ""
        self._remain_depart_time_text = ""
        self._display_time = False
        self._is_emergency_mode = False
        self._in_emergency_state = False
        self._emergency_trigger_time = 0
        self._is_stopping = False
        self._is_driving = False
        self._previous_driving_status = False
        self._reach_final = False
        self._announced_going_to_depart = False
        self._announced_going_to_arrive = False
        self._pre_door_announce_status = DoorStatus.UNKNOWN
        self._fms_check_time = 0

        self.process_station_list_from_fms()

        self._node.create_timer(1, self.route_checker_callback)
        self._node.create_timer(1, self.emergency_checker_callback)
        self._node.create_timer(1, self.view_mode_callback)
        self._node.create_timer(1, self.calculate_time_callback)
        self._node.create_timer(1, self.door_status_callback)

    def emergency_checker_callback(self):
        if self._parameter.ignore_emergency:
            self._is_emergency_mode = False
        else:
            self._is_emergency_mode = (
                self._autoware.information.mrm_behavior == MrmState.EMERGENCY_STOP
            )

        if self._is_emergency_mode and not self._in_emergency_state:
            self._announce_interface.announce_emergency("emergency")
            self._in_emergency_state = True
        elif not self._is_emergency_mode and self._in_emergency_state:
            self._in_emergency_state = False
        elif self._is_emergency_mode and self._in_emergency_state:
            if not self._emergency_trigger_time:
                self._emergency_trigger_time = self._node.get_clock().now()
            elif self._node.get_clock().now() - self._emergency_trigger_time > Duration(
                seconds=self._parameter.emergency_repeat_period
            ):
                self._announce_interface.announce_emergency("in_emergency")
                self._emergency_trigger_time = 0

    def door_status_callback(self):
        door_status = self._autoware.information.door_status
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
            elif utils.check_schedule_update(self._schedule_details, data):
                self._fms_check_time = self._node.get_clock().now()
                raise Exception("same schedule, skip")

            self._schedule_details = utils.update_schedule_details(data)

            self._display_details.route_name = utils.get_route_name(
                data.get("tags", []),
            )

            self.task_list = utils.seperate_task_list(data.get("tasks", []))

            if not self.task_list.doing_list:
                raise Exception("doing_list is not found, skip")

            for task in self.task_list.doing_list:
                self._current_task_details = utils.process_current_task(task)

            if self._display_details.previous_station == ["", ""] and self.task_list.done_list:
                self._display_details.previous_station = utils.get_prevous_station_name_from_fms(
                    self.task_list.done_list
                )

            self._display_details.next_station_list = utils.create_next_station_list(
                self._current_task_details,
                self.task_list.todo_list,
                "fms",
                self._schedule_details.schedule_type,
            )

            # Reset previous station when reach goal
            if self._reach_final and self.task_list.doing_list:
                self._reach_final = False
                self._display_details.previous_station = ["", ""]

            self._fms_check_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().warning(
                "Unable to get the task from FMS, ERROR: " + str(e), throttle_duration_sec=5
            )

    def arrived_goal(self):
        try:
            if self._current_task_details == utils.init_CurrentTask():
                raise Exception("No current task details")

            self._display_details.previous_station = self._current_task_details.departure_station

            if not self.task_list.todo_list:
                # Reach final station
                self._current_task_details.departure_station = (
                    self._current_task_details.arrival_station
                )
                self._current_task_details.arrival_station = ["", ""]
                self._reach_final = True
                return

            next_task = self.task_list.todo_list.pop(0)
            # Get the next task from todo_list
            self._current_task_details = utils.process_current_task(next_task)

            self._display_details.next_station_list = utils.create_next_station_list(
                self._current_task_details,
                self.task_list.todo_list,
                "local",
                self._schedule_details.schedule_type,
            )
            self._announce_interface.announce_arrived()
        except Exception as e:
            self._node.get_logger().error("Unable to update the goal, ERROR: " + str(e))

    # ========== Timer Callback =============

    def route_checker_callback(self):
        try:
            if self._autoware.information.operation_mode == OperationModeState.AUTONOMOUS:
                # Check whether the vehicle is move in autonomous
                self._is_driving = True
                self._is_stopping = False
            elif self._autoware.information.route_state == RouteState.ARRIVED:
                # Flip the flag when the vehicle arrive to goal
                self._is_stopping = True
                self._is_driving = False

            if not self._fms_check_time:
                self.process_station_list_from_fms()
            elif self._node.get_clock().now() - self._fms_check_time > Duration(
                seconds=self._parameter.check_fms_time
            ):
                self.process_station_list_from_fms()

            if self._is_emergency_mode:
                return

            if (
                not self._autoware.information.autoware_control
                and self._parameter.ignore_manual_driving
                and self._parameter.set_goal_by_distance
            ):
                if self._autoware.information.goal_distance < self._parameter.goal_distance:
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
            if self._current_task_details == utils.init_CurrentTask():
                return

            remain_minute = 100
            remain_minute = int(
                (self._current_task_details.depart_time - self._node.get_clock().now().to_msg().sec)
                / 60
            )
            if remain_minute > 0:
                self._remain_depart_time_text = "このバスはあと{}分程で出発します".format(str(remain_minute))
            else:
                self._remain_depart_time_text = "間もなく出発します"

            if self._reach_final:
                self._remain_depart_time_text = "終点です。\nご乗車ありがとうございました"
            elif self._is_driving:
                self._announced_going_to_depart = False
                if (
                    self._autoware.information.goal_distance < 100
                    and not self._announced_going_to_arrive
                ):
                    self._announce_interface.announce_going_to_depart_and_arrive("going_to_arrive")
                    self._announced_going_to_arrive = True
                    self._remain_arrive_time_text = "間もなく到着します"
            elif self._is_stopping:
                self._remain_arrive_time_text = ""
                self._announced_going_to_arrive = False
                if remain_minute < 1 and not self._announced_going_to_depart:
                    self._announce_interface.announce_going_to_depart_and_arrive("going_to_depart")
                    self._announced_going_to_depart = True

            if (
                remain_minute < 5
                or self._autoware.information.goal_distance < 100
                or self._reach_final
            ):
                self._display_time = True
            else:
                self._display_time = False
        except Exception as e:
            self._node.get_logger().error("Error in getting calculate the time: " + str(e))

    def view_mode_callback(self):
        try:
            self._viewController.clock_string = datetime.now().strftime("%H:%M")
            self._viewController.route_name = self._display_details.route_name
            self._viewController.departure_station_name = (
                self._current_task_details.departure_station
            )
            self._viewController.arrival_station_name = self._current_task_details.arrival_station
            self._viewController.previous_station_name = self._display_details.previous_station
            self._viewController.next_station_list = self._display_details.next_station_list
            self._viewController.remain_arrive_time_text = self._remain_arrive_time_text
            self._viewController.remain_depart_time_text = self._remain_depart_time_text
            self._viewController.display_time = self._display_time

            if self._is_emergency_mode:
                view_mode = "emergency_stopped"
            elif (
                not self._autoware.information.autoware_control
                and not self._parameter.ignore_manual_driving
            ):
                view_mode = "manual_driving"
            elif self._is_stopping and self._current_task_details.departure_station != ["", ""]:
                view_mode = "stopping"
            elif self._is_driving and self._current_task_details.arrival_station != ["", ""]:
                view_mode = "driving"
            elif self._is_driving:
                view_mode = "auto_driving"
            elif self._current_task_details.departure_station != ["", ""]:
                view_mode = "stopping"
            else:
                view_mode = "out_of_service"

            self._viewController.view_mode = view_mode
        except Exception as e:
            self._node.get_logger().error("Error in updating the view mode: " + str(e))
