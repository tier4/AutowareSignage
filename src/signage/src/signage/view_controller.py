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
    _get_clock_string_signal = pyqtSignal(str)
    _get_monitor_height_signal = pyqtSignal(int)
    _get_monitor_width_signal = pyqtSignal(int)
    _get_size_ratio_signal = pyqtSignal(float)

    def __init__(self, node, parameter_interface):
        super(ViewControllerProperty, self).__init__()
        self._node = node
        self._view_mode = ""
        self._route_name = ["", ""]
        self._departure_station_name = ["", ""]
        self._arrival_station_name = ["", ""]
        self._next_station_list = [["", ""], ["", ""], ["", ""], ["", ""], ["", ""]]
        self._previous_station_name = ["", ""]
        self._remain_arrive_time_text = ""
        self._remain_depart_time_text = ""
        self._monitor_width = 1920
        self._monitor_height = 540
        self._size_ratio = 1
        self.monitor_width = parameter_interface.parameter.monitor_width
        self.monitor_height = parameter_interface.parameter.monitor_height
        self.size_ratio = (self._monitor_height / 360.0) * (self._monitor_width / 1920) * 0.8
        self._clock_string = ""

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

    # QMLへroute_nameを反映させる
    @pyqtProperty("QString", notify=_get_clock_string_signal)
    def clock_string(self):
        return self._clock_string

    @clock_string.setter
    def clock_string(self, clock_string):
        if self._clock_string == clock_string:
            return
        self._clock_string = clock_string
        self._get_clock_string_signal.emit(clock_string)

    @pyqtProperty(int, notify=_get_monitor_width_signal)
    def monitor_width(self):
        return self._monitor_width

    @monitor_width.setter
    def monitor_width(self, monitor_width):
        if self._monitor_width == monitor_width:
            return
        self._monitor_width = monitor_width
        self._get_monitor_width_signal.emit(monitor_width)

    @pyqtProperty(int, notify=_get_monitor_height_signal)
    def monitor_height(self):
        return self._monitor_height

    @monitor_height.setter
    def monitor_height(self, monitor_height):
        if self._monitor_height == monitor_height:
            return
        self._monitor_height = monitor_height
        self._get_monitor_height_signal.emit(monitor_height)

    @pyqtProperty(float, notify=_get_size_ratio_signal)
    def size_ratio(self):
        return self._size_ratio

    @size_ratio.setter
    def size_ratio(self, size_ratio):
        if self._size_ratio == size_ratio:
            return
        self._size_ratio = size_ratio
        self._get_size_ratio_signal.emit(size_ratio)
