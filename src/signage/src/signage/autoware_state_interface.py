#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from autoware_api_msgs.msg import AwapiAutowareStatus, AwapiVehicleStatus

class AutowareStateInterface():

    def __init__(self):
        self.autoware_state_callback_list = []
        self.control_mode_callback_list = []
        self.emergency_stopped_callback_list = []

        self.turn_signal_callback_list = []
        self.velocity_callback_list = []

        self._status_sub = rospy.Subscriber("/awapi/autoware/get/status", AwapiAutowareStatus, self.sub_autoware_state)
        self._vehicle_status_sub = rospy.Subscriber("/awapi/vehicle/get/status", AwapiVehicleStatus, self.sub_vehicle_state)
        rospy.Timer(rospy.Duration(1), self.check_sub)

    def check_sub(self, event):
        # Reconnect to the subcriber if the roscore is down
        try:
            if self._status_sub.get_num_connections() <= 0:
                self._status_sub.unregister()
                self._status_sub = rospy.Subscriber("/awapi/autoware/get/status", AwapiAutowareStatus, self.sub_autoware_state)
                rospy.logwarn("Trying to reset the status sub, connect: " + str(self._status_sub.get_num_connections()))
            if self._vehicle_status_sub.get_num_connections() <= 0:
                self._vehicle_status_sub.unregister()
                self._vehicle_status_sub = rospy.Subscriber("/awapi/vehicle/get/status", AwapiVehicleStatus, self.sub_vehicle_state)
                rospy.logwarn("Trying to reset the vehicle status sub, connect: " + str(self._vehicle_status_sub.get_num_connections()))
        except Exception as e:
            rospy.logerr("Unable to reset the subcriber, ERROR: {}".format(str(e)))

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

    # ros subscriber
    # autoware stateをsubしたときの処理
    def sub_autoware_state(self, topic):
        try:
            autoware_state = topic.autoware_state
            control_mode = topic.control_mode
            emergency_stopped = topic.emergency_stopped

            for callback in self.autoware_state_callback_list:
                callback(autoware_state)

            for callback in self.control_mode_callback_list:
                callback(control_mode)

            for callback in self.emergency_stopped_callback_list:
                callback(emergency_stopped)
        except Exception as e:
            rospy.logerr("Unable to get the autoware state, ERROR: " + str(e))

    # vehicle stateをsubしたときの処理
    def sub_vehicle_state(self, topic):
        try:
            turn_signal = topic.turn_signal
            velocity = topic.velocity
            steering = topic.steering

            for callback in self.turn_signal_callback_list:
                callback(turn_signal)
            for callback in self.velocity_callback_list:
                callback(velocity)
        except Exception as e:
            rospy.logerr("Unable to get the vehicle state, ERROR: " + str(e))