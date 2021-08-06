# This Python file uses the following encoding: utf-8

from PyQt5.QtCore import pyqtProperty
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject

import rclpy
from rclpy.node import Node
import os

import requests
import json
from datetime import datetime
from pytz import timezone
from dateutil import parser
from itertools import cycle
import collections
from autoware_debug_msgs.msg import Float64Stamped

class ViewControllerProperty(QObject):
    _view_mode_changed_signal = pyqtSignal(str)
    _route_name_signal = pyqtSignal(str)
    _get_departure_station_name_signal = pyqtSignal(str)
    _get_arrival_station_name_signal = pyqtSignal(str)
    _get_next_station_list_signal = pyqtSignal(list)
    _get_previous_station_list_signal = pyqtSignal(list)
    _get_remain_time_text_signal = pyqtSignal(str)
    _get_display_time_signal = pyqtSignal(bool)

    def __init__(self, node=None, autoware_state_interface=None, announce_interface=None):
        super(ViewControllerProperty, self).__init__()
        node.get_logger().info("%s initializing..." % "signage")
        self._node = node

        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        autoware_state_interface.set_control_mode_callback(self.sub_control_mode)
        autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)
        self._announce_interface = announce_interface

        self.is_auto_mode = False
        self.is_emergency_mode = False
        self.is_stopping = False
        self.is_driving = False
        self._view_mode = ""
        self._route_name = ""
        self._departure_station_name = ""
        self._arrival_station_name = ""
        self._next_station_list = ["", "", ""]
        self._previous_station_deque = collections.deque(3*[""], 3)
        self._previous_station_list = ["", "", ""]
        self._remain_time_text = ""
        self._distance = 1000
        self._display_time = False
        self._announced_going_to_depart = False
        self._announced_going_to_arrive = False
        self._previous_driving_status = False
        self._checked_route_fms = False
        self._checked_route_local = False
        self._current_task_list = []
        self._depart_time = 0
        self._reach_final = False
        self._schedule_id = ""
        self._schedule_updated_time = ""
        self._fms_payload = {
             "method": "get",
             "url": "https://" + os.getenv('FMS_URL', 'fms.web.auto') + "/v1/projects/{project_id}/environments/{environment_id}/vehicles/{vehicle_id}/active_schedule",
             "body": {}
            }
        self._sub_path_distance = node.create_subscription(
            Float64Stamped,
            '/path_distance_calculator/distance',
            self.path_distance_callback,
            10
        )
        self._cycle_view_control_timer = self._node.create_timer(
            0.1,
            self.update_view_state)
        try:
            self.process_station_list_from_fms()
        except:
            pass
        self._route_checker = self._node.create_timer(
            1,
            self.route_checker_callback)

    # view mode
    @pyqtProperty(str, notify=_view_mode_changed_signal)
    def view_mode(self):
        return self._view_mode

    @view_mode.setter
    def view_mode(self, view_mode):
        if self._view_mode == view_mode:
            return
        self._view_mode = view_mode
        self._view_mode_changed_signal.emit(view_mode)

    def update_view_state(self):
        if self.is_emergency_mode:
            self.view_mode = "emergency_stopped"
        elif not self.is_auto_mode:
            self.view_mode = "manual_driving"
        elif self.is_stopping and self._departure_station_name:
            self.view_mode = "stopping"
        elif self.is_driving and self._arrival_station_name:
            self._previous_driving_status = True
            self.view_mode = "driving"
        else:
            self.view_mode = "out_of_service"

        # self._node.get_logger().info('view mode %r' % (self._view_mode))

    def sub_autoware_state(self, autoware_state):
        self.is_stopping = autoware_state in ["WaitingForRoute", "WaitingForEngage", "ArrivedGoal", "Planning"]
        self.is_driving = autoware_state == "Driving"

    def sub_control_mode(self, control_mode):
        self.is_auto_mode = control_mode == 1

    def sub_emergency(self, emergency_stopped):
        self.is_emergency_mode = emergency_stopped

    # QMLへroute_nameを反映させる
    @pyqtProperty("QString", notify=_route_name_signal)
    def route_name(self):
        return self._route_name

    @route_name.setter
    def route_name(self, route_name):
        if self._route_name == route_name:
            return
        self._route_name = route_name
        self._route_name_signal.emit(route_name)

    # QMLへ出発地バス停名を反映させる
    @pyqtProperty("QString", notify=_get_departure_station_name_signal)
    def departure_station_name(self):
        return self._departure_station_name

    @departure_station_name.setter
    def departure_station_name(self, departure_station_name):
        if self._departure_station_name == departure_station_name:
            return
        self._departure_station_name = departure_station_name
        self._get_departure_station_name_signal.emit(departure_station_name)

    # QMLへ到着地バス停名を反映させる
    @pyqtProperty("QString", notify=_get_arrival_station_name_signal)
    def arrival_station_name(self):
        return self._arrival_station_name

    @arrival_station_name.setter
    def arrival_station_name(self, arrival_station_name):
        if self._arrival_station_name == arrival_station_name:
            return
        self._arrival_station_name = arrival_station_name
        self._get_arrival_station_name_signal.emit(arrival_station_name)

    # QMLへのsignal
    @pyqtProperty(list, notify=_get_next_station_list_signal)
    def next_station_list(self):
        return self._next_station_list

    @next_station_list.setter
    def next_station_list(self, next_station_list):
        if self._next_station_list == next_station_list:
            return
        self._next_station_list = next_station_list
        self._get_next_station_list_signal.emit(next_station_list)

    # QMLへのsignal
    @pyqtProperty(list, notify=_get_previous_station_list_signal)
    def previous_station_list(self):
        return self._previous_station_list

    @previous_station_list.setter
    def previous_station_list(self, previous_station_list):
        if self._previous_station_list == previous_station_list:
            return
        self._previous_station_list = previous_station_list
        self._get_previous_station_list_signal.emit(previous_station_list)

    # QMLへroute_nameを反映させる
    @pyqtProperty("QString", notify=_get_remain_time_text_signal)
    def remain_time_text(self):
        return self._remain_time_text

    @remain_time_text.setter
    def remain_time_text(self, remain_time_text):
        if self._remain_time_text == remain_time_text:
            return
        self._remain_time_text = remain_time_text
        self._get_remain_time_text_signal.emit(remain_time_text)

    # QMLへのsignal
    @pyqtProperty(bool, notify=_get_display_time_signal)
    def display_time(self):
        return self._display_time

    @display_time.setter
    def display_time(self, display_time):
        if self._display_time == display_time:
            return
        self._display_time = display_time
        self._get_display_time_signal.emit(display_time)

    def route_checker_callback(self):
        if not self.is_auto_mode or self.is_emergency_mode:
            return

        if not self._current_task_list:
            # if not found current task, reconnect to fms
            try:
                self.process_station_list_from_fms()
            except:
                return

        if self._reach_final and self._current_task_list:
            self._reach_final = False
            self._previous_driving_status = False
            self._previous_station_deque = collections.deque(3*[""], 3)
            self.previous_station_list = ["", "", ""]

        if self.is_stopping and not self._checked_route_local and self._previous_driving_status:
            self.process_station_list_from_local()
        elif self.is_driving and not self._checked_route_fms:
            try:
                self.process_station_list_from_fms()
            except:
                return
        if not self._current_task_list:
            return

        ## TODO: use time?
        current_time = self._node.get_clock().now().to_msg().sec
        try:
            remain_minute = 100
            if self.is_driving:
                self._announced_going_to_depart = False
                if self._distance < 100 and not self._announced_going_to_arrive:
                    self._announce_interface.announce_going_to_depart_and_arrive("going_to_arrive")
                    self._announced_going_to_arrive = True
                    self.remain_time_text = "間もなく到着します"

            if self.is_stopping:
                self._announced_going_to_arrive = False
                remain_minute = int((self._depart_time - current_time)/60)
                if remain_minute > 0:
                    self.remain_time_text = "このバスはあと{}分程で出発します".format(str(remain_minute))
                else:
                    self.remain_time_text = "間もなく出発します"

                if remain_minute < 1 and not self._announced_going_to_depart:
                    self._announce_interface.announce_going_to_depart_and_arrive("going_to_depart")
                    self._announced_going_to_depart = True

            if remain_minute < 5 or self._distance < 100 :
                self.display_time = True
            else:
                self.display_time = False
        except Exception as e:
            self._node.get_logger().error("Error in getting calculate the time: " + str(e))

    def process_depart_arrive_station_details(self, task):
        if task["origin_point_name"]:
            self.departure_station_name = task["origin_point_name"]
        else:
            self.departure_station_name = "start"
        self.arrival_station_name = task["destination_point_name"]
        date_time_obj = parser.parse(task["plan_start_time"])
        self._depart_time = datetime.timestamp(date_time_obj)

    def process_previous_station_list(self):
        self._previous_station_deque.appendleft(self.departure_station_name)
        self.previous_station_list = list(self._previous_station_deque)

    def create_next_station_list(self, call_type):
        station_list = []

        for task in self._current_task_list:
            if task["task_type"] == "move" and task["status"] in ["doing", "todo"]:
                station_list.append(task["destination_point_name"])

        if self.departure_station_name != "start" and call_type == "local" and self._schedule_type == "loop":
            station_list.append(self.departure_station_name)

        if len(station_list) < 4 and self._schedule_type == "loop":
            station_cycle = cycle(station_list)
            for _ in range(4 - len(station_list)):
                station_list.append(next(station_cycle))
        else:
            for _ in range(4 - len(station_list)):
                station_list.append("")

        self.next_station_list = list(station_list[:3])

    def process_station_list_from_local(self):
        try:
            if not self._current_task_list:
                # empty current task list
                return

            self.process_previous_station_list()

            # remove the first task
            self._current_task_list.pop(0)

            if not self._current_task_list:
                # Reach final station
                self.departure_station_name = self.arrival_station_name
                self.arrival_station_name  = ""
                self.remain_time_text = "終点です。\nご乗車ありがとうございました"
                self._reach_final = True
                return

            for task in self._current_task_list:
                if task["task_type"] == "move" and task["status"] == "todo":
                    self.process_depart_arrive_station_details(task)
                    break

            self.create_next_station_list("local")
            self._checked_route_local = True
            self._checked_route_fms = False
        except Exception as e:
            self._node.get_logger().error("Unable to get the task from local, ERROR: " + str(e))

    def process_station_list_from_fms(self):
        try:
            respond = requests.post("http://{}:4711/v1/services/order".format(os.getenv('AUTOWARE_IP', 'localhost')), json=self._fms_payload, timeout=5)
            data = json.loads(respond.text)

            if not data:
                self._node.get_logger().error("No data from fms")
                return

            elif self._schedule_updated_time == data["updated_at"] and self._schedule_id == data["schedule_id"]:
                self._node.get_logger().error("same schedule, skip")
                return

            self._schedule_type = data["schedule_type"]
            self.route_name  = data["project_id"]

            fms_task_list = []
            fms_done_list = []
            for task in data.get("tasks", {}):
                if task["task_type"] == "move" and task["status"] in ["doing", "todo"]:
                    fms_task_list.append(task)
                elif task["task_type"] == "move" and task["status"] in ["done"]:
                    fms_done_list.append(task)
            self._current_task_list = list(fms_task_list)

            if not self._current_task_list:
                return

            for task in self._current_task_list:
                if task["status"] == "doing":
                    self.process_depart_arrive_station_details(task)

            if self._previous_station_list == ["", "", ""]:
                for task in fms_done_list:
                    self._previous_station_deque.appendleft(task["origin_point_name"] if task["origin_point_name"] else "start")
                self.previous_station_list = list(self._previous_station_deque)

            self.create_next_station_list("fms")
            self._schedule_updated_time = data["updated_at"]
            self._schedule_id = data["schedule_id"]
            self._checked_route_local = False
            self._checked_route_fms = True
        except Exception as e:
            self._node.get_logger().error("Unable to get the task from FMS, ERROR: " + str(e))
            raise Exception("Unable to get the task from FMS, ERROR: " + str(e))

    def path_distance_callback(self, topic):
        if topic:
            self._distance = topic.data
