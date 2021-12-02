# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8


from PyQt5.QtCore import pyqtProperty
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject

class ViewControllerProperty(QObject):
    _view_mode_changed_signal = pyqtSignal(str)
    _route_name_signal = pyqtSignal(list)
    _get_departure_station_name_signal = pyqtSignal(list)
    _get_arrival_station_name_signal = pyqtSignal(list)
    _get_next_station_list_signal = pyqtSignal(list)
    _get_previous_station_name_signal = pyqtSignal(list)
    _get_remain_arrive_time_text_signal = pyqtSignal(str)
    _get_remain_depart_time_text_signal = pyqtSignal(str)
    _get_display_time_signal = pyqtSignal(bool)

    def __init__(self, node=None, autoware_state_interface=None, announce_interface=None):
        super(ViewControllerProperty, self).__init__()

        self._announce_interface = announce_interface
        self._view_mode = ""
        self._route_name = ["", ""]
        self._departure_station_name = ["", ""]
        self._arrival_station_name = ["", ""]
        self._next_station_list = [["",""]*5]
        self._previous_station_name = ["", ""]
        self._remain_arrive_time_text = ""
        self._remain_depart_time_text = ""
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
        
        
        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        autoware_state_interface.set_control_mode_callback(self.sub_control_mode)
        autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)
        autoware_state_interface.set_distance_callback(self.sub_distance)

        self._fms_check_time = self._node.get_clock().now()


        self._cycle_view_control_timer = self._node.create_timer(1.0, self.update_view_state)
        try:
            self.process_station_list_from_fms(True)
        except:
            pass

    @pyqtProperty(str, notify=_view_mode_changed_signal)
    def view_mode(self):
        return self._view_mode

    @view_mode.setter
    def view_mode(self, view_mode):
        if self._view_mode == view_mode:
            return
        self._view_mode = view_mode
        self._view_mode_changed_signal.emit(view_mode)

    # QMLへroute_nameを反映させる
    @pyqtProperty(list, notify=_route_name_signal)
    def route_name(self):
        return self._route_name

    @route_name.setter
    def route_name(self, route_name):
        if self._route_name == route_name:
            return
        self._route_name = route_name
        self._route_name_signal.emit(route_name)

    # QMLへ出発地バス停名を反映させる
    @pyqtProperty(list, notify=_get_departure_station_name_signal)
    def departure_station_name(self):
        return self._departure_station_name

    @departure_station_name.setter
    def departure_station_name(self, departure_station_name):
        if self._departure_station_name == departure_station_name:
            return
        self._departure_station_name = departure_station_name
        self._get_departure_station_name_signal.emit(departure_station_name)

    # QMLへ到着地バス停名を反映させる
    @pyqtProperty(list, notify=_get_arrival_station_name_signal)
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
    @pyqtProperty(list, notify=_get_previous_station_name_signal)
    def previous_station_name(self):
        return self._previous_station_name

    @previous_station_name.setter
    def previous_station_name(self, previous_station_name):
        if self._previous_station_name == previous_station_name:
            return
        self._previous_station_name = previous_station_name
        self._get_previous_station_name_signal.emit(previous_station_name)

    # QMLへroute_nameを反映させる
    @pyqtProperty("QString", notify=_get_remain_arrive_time_text_signal)
    def remain_arrive_time_text(self):
        return self._remain_arrive_time_text

    @remain_arrive_time_text.setter
    def remain_arrive_time_text(self, remain_arrive_time_text):
        if self._remain_arrive_time_text == remain_arrive_time_text:
            return
        self._remain_arrive_time_text = remain_arrive_time_text
        self._get_remain_arrive_time_text_signal.emit(remain_arrive_time_text)

    # QMLへroute_nameを反映させる
    @pyqtProperty("QString", notify=_get_remain_depart_time_text_signal)
    def remain_depart_time_text(self):
        return self._remain_depart_time_text

    @remain_depart_time_text.setter
    def remain_depart_time_text(self, remain_depart_time_text):
        if self._remain_depart_time_text == remain_depart_time_text:
            return
        self._remain_depart_time_text = remain_depart_time_text
        self._get_remain_depart_time_text_signal.emit(remain_depart_time_text)

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
