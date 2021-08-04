# This Python file uses the following encoding: utf-8

from PyQt5.QtCore import pyqtProperty
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSlot

import rclpy
from rclpy.node import Node

import requests
import json

from autoware_api_msgs.msg import AwapiAutowareStatus
from autoware_api_msgs.msg import AwapiVehicleStatus

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

        # Changet to fms
        # rospy.Subscriber("/api/get/stations", ApiDummyStation, self.sub_route_station)
        # rospy.Subscriber("/api/get/route", RouteStation, self.sub_route)

        self.is_auto_mode = False
        self.is_emergency_mode = False
        self.is_stopping = False
        self.is_driving = False
        self._view_mode = ""
        self._route_name = ""
        self._departure_station_name = ""
        self._arrival_station_name = ""
        self._next_station_list = ["", "", ""]
        self._previous_station_list = ["", "", ""]
        self._remain_time_text = ""
        self._display_time = False
        self._announce_depart_arrive = False
        self._current_task_list = {}
        self._FMS_payload = {
             "method": "get",
             "url": "https://fms.dev.web.auto/v1/projects/{project_id}/environments/{environment_id}/vehicles/{vehicle_id}/active_schedule",
             "body": {}
            }
        self._cycle_view_control_timer = self._node.create_timer(
            0.1,
            self.update_view_state)
        # TODO: fix timer
        # self._calculate_route_time_timer = self._node.create_timer(
        #     1,
        #     self.calculate_time)
        # TODO: change to only call when in needed
        self.post_request()
        self._FMS_request_timer = self._node.create_timer(
            60,
            self.post_request)

    # view mode
    @pyqtProperty(str, notify=_view_mode_changed_signal)
    def view_mode(self):
        return self._view_mode

    @view_mode.setter
    def view_mode(self, view_mode):
        # if self._view_mode == view_mode:
        #     return
        self._view_mode = view_mode
        self._view_mode_changed_signal.emit(view_mode)

    def update_view_state(self):
        if self.is_emergency_mode:
            self.view_mode = "emergency_stopped"
        elif not self.is_auto_mode:
            self.view_mode = "manual_driving"
        elif self.is_stopping and self._departure_station_name:
            self.view_mode = "stopping"
        elif self.is_driving and self._departure_station_name:
            self.view_mode = "driving"
        else:
            self.view_mode = "out_of_service"

        # self._node.get_logger().info('view mode %r' % (self._view_mode))

    def sub_autoware_state(self, autoware_state):
        self.is_stopping = autoware_state != "Driving" and autoware_state != "InitializingVehicle"
        self.is_driving = autoware_state == "Driving"

    def sub_control_mode(self, control_mode):
        self.is_auto_mode = control_mode == 1

    def sub_emergency(self, emergency_stopped):
        self.is_emergency_mode = emergency_stopped

    def generate_next_previous_station_list(self, station_id):
        list_index = self._station_arrangement.index(station_id)
        station_list = ["", "", ""]
        n = 0
        for next_station in self._station_arrangement[list_index:][1:4]:
            station_list[n] = self._stations[next_station]["name"]
            n += 1
        self.next_station_list = list(station_list)

        station_list = ["", "", ""]
        n = 0
        for next_station in self._station_arrangement[:list_index][-3:][::-1]:
            station_list[n] = self._stations[next_station]["name"]
            n += 1
        self.previous_station_list = list(station_list)

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

    def calculate_time(self):
        if not self.is_auto_mode or self.is_emergency_mode:
            return

        if self._arrival_station_id and self._departure_station_id:
            current_time = rospy.get_time()
            try:
                if self.is_driving:
                    remain_minute = int((int(self._stations[self._arrival_station_id]["eta"].secs) - current_time)/60)
                    if remain_minute > 0:
                        self.remain_time_text = "あと{}分で到着します".format(str(remain_minute))
                    else:
                        self.remain_time_text = "間もなく到着します"

                    if remain_minute < 1 and self._announce_depart_arrive:
                        self._announce_interface.announce_going_to_depart_and_arrive("going_to_arrive")
                        self._announce_depart_arrive = False
                elif self.is_stopping:
                    remain_minute = int((int(self._stations[self._departure_station_id]["etd"].secs) - current_time)/60)
                    if remain_minute > 0:
                        self.remain_time_text = "このバスはあと{}分程で出発します".format(str(remain_minute))
                    else:
                        self.remain_time_text = "間もなく出発します"

                    if remain_minute < 1 and not self._announce_depart_arrive:
                        self._announce_interface.announce_going_to_depart_and_arrive("going_to_depart")
                        self._announce_depart_arrive = True
                if remain_minute <= 5:
                    self.display_time = True
                else:
                    self.display_time = False
            except Exception as e:
                rospy.logerr("Error in getting calculate the time: " + str(e))

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

    def post_request(self):
        # TODO: change to use os env
        try:
            respond = requests.post("http://localhost:4711/v1/services/order", json=self._FMS_payload)
            data = json.loads(respond.text)
            self._node.get_logger().info('view mode: ' + str(data))
            schedule_type = data.get("schedule_type", "")
            self.route_name  = data.get("schedule_id", "")
            self._current_task_list = data.get("tasks", self._current_task_list)
            station_list = []
            for task in self._current_task_list:
                if task["task_type"] == "move" and task["status"] == "doing":
                    self.departure_station_name = task["origin_point_name"]
                    self.arrival_station_name = task["destination_point_name"]
                    station_list.append(self.arrival_station_name)
                ## Generate next station list
                elif task["task_type"] == "move" and task["status"] == "todo":
                    station_list.append(task["destination_point_name"])

            if len(station_list) < 3:
                if schedule_type == "loop":
                    # Auto fill in the lopp
                    station_list = station_list

            # limit to 3
            self.next_station_list = list(station_list[:3])
        except Exception as e:
            self._node.get_logger().error("Unable to get the task from FMS, ERROR: " + str(e))