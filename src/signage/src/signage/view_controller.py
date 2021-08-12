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
from itertools import cycle
import collections

class ViewControllerProperty(QObject):
    _view_mode_changed_signal = pyqtSignal(str)
    _route_name_signal = pyqtSignal(str)
    _get_departure_station_name_signal = pyqtSignal(str)
    _get_arrival_station_name_signal = pyqtSignal(str)
    _get_next_station_list_signal = pyqtSignal(list)
    _get_previous_station_list_signal = pyqtSignal(list)
    _get_remain_time_text_signal = pyqtSignal(str)

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
        self._announce_depart = False
        self._previous_driving_status = False
        self._checked_route_fms = False
        self._checked_route_local = False
        self._current_task_list = []
        self._depart_time = 0
        self._fms_payload = {
             "method": "get",
             "url": os.getenv('FMS_URL'),
             "body": {}
            }
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
        elif self.is_stopping:
            self.view_mode = "stopping"
        elif self.is_driving:
            self._previous_driving_status = True
            self.view_mode = "driving"
        else:
            self.view_mode = "out_of_service"

        # self._node.get_logger().info('view mode %r' % (self._view_mode))

    def sub_autoware_state(self, autoware_state):
        self.is_stopping = autoware_state != "Driving" and  autoware_state != "InitializingVehicle"
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

    def route_checker_callback(self):
        if not self.is_auto_mode or self.is_emergency_mode:
            return

        if not self._current_task_list:
            # if not found current task, reconnect to fms
            try:
                self.process_station_list_from_fms()
            except:
                return

        if self.is_stopping and not self._checked_route_local and self._previous_driving_status:
            self.process_station_list_from_local()
        elif self.is_driving and not self._checked_route_fms:
            try:
                self.process_station_list_from_fms()
            except:
                return

        ## TODO: use time?
        current_time = self._node.get_clock().now().to_msg().sec
        try:
            if self.is_driving:
                self._announce_depart = False

            if self.is_stopping:
                remain_minute = int((self._depart_time - current_time)/60)
                if remain_minute > 0:
                    self.remain_time_text = "このバスはあと{}分程で出発します".format(str(remain_minute))
                else:
                    self.remain_time_text = "間もなく出発します"

                if remain_minute < 1 and not self._announce_depart:
                    self._announce_interface.announce_going_to_depart_and_arrive("going_to_depart")
                    self._announce_depart = True
        except Exception as e:
            self._node.get_logger().error("Error in getting calculate the time: " + str(e))

    def process_depart_arrive_station_details(self, task):
        if task["origin_point_name"]:
            self.departure_station_name = task["origin_point_name"]
        else:
            self.departure_station_name = "start"
        self.arrival_station_name = task["destination_point_name"]
        date_time_obj = datetime.strptime(task["plan_start_time"], '%Y-%m-%dT%H:%M:%S.%f%z')
        self._depart_time = datetime.timestamp(date_time_obj)

    def process_previous_station_list(self):
        self._previous_station_deque.appendleft(self.departure_station_name)
        self.previous_station_list = list(self._previous_station_deque)

    def create_next_station_list(self, call_type):
        station_list = []

        for task in self._current_task_list:
            if task["task_type"] == "move":
                station_list.append(task["destination_point_name"])

        if self.departure_station_name != "start" and call_type == "local":
            station_list.append(self.departure_station_name)

        if len(station_list) < 4 and self._schedule_type == "loop":
            station_cycle = cycle(station_list)
            for _ in range(4 - len(station_list)):
                station_list.append(next(station_cycle))

        self.next_station_list = list(station_list[:3])

    def process_station_list_from_local(self):
        try:
            self.process_previous_station_list()
            if not self._current_task_list:
                # empty current task list
                return

            # remove the first task
            self._current_task_list.pop(0)

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
            ip_address = os.getenv('AUTOWARE_IP')
            if not ip_address:
                ip_address = "localhost"

            respond = requests.post("http://{}:4711/v1/services/order".format(ip_address), json=self._fms_payload, timeout=5)
            data = json.loads(respond.text)
            self._schedule_type = data["schedule_type"]
            self.route_name  = data["project_id"]
            self._current_task_list = data.get("tasks", self._current_task_list)
            for task in self._current_task_list:
                if task["task_type"] == "move" and task["status"] == "doing":
                    self.process_depart_arrive_station_details(task)

            self.create_next_station_list("fms")
            self._checked_route_local = False
            self._checked_route_fms = True
        except Exception as e:
            self._node.get_logger().error("Unable to get the task from FMS, ERROR: " + str(e))
            raise Exception("Unable to get the task from FMS, ERROR: " + str(e))
