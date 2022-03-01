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
    _get_monitor_height_signal = pyqtSignal(int)
    _get_font_ratio_signal = pyqtSignal(float)

    def __init__(self, node=None):
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
        self._display_time = False
        self._node.declare_parameter("monitor_height", 360)
        self._monitor_height = (
            self._node.get_parameter("monitor_height").get_parameter_value().integer_value
        )
        self._font_ratio = (self._monitor_height / 360.0) * 0.8

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

    @pyqtProperty(int, notify=_get_monitor_height_signal)
    def monitor_height(self):
        return self._monitor_height

    @monitor_height.setter
    def monitor_height(self, monitor_height):
        if self._monitor_height == monitor_height:
            return
        self._monitor_height = monitor_height
        self._get_monitor_height_signal.emit(monitor_height)

    @pyqtProperty(float, notify=_get_font_ratio_signal)
    def font_ratio(self):
        return self._font_ratio

    @font_ratio.setter
    def font_ratio(self, font_ratio):
        if self._font_ratio == font_ratio:
            return
        self._font_ratio = font_ratio
        self._get_font_ratio_signal.emit(font_ratio)
