# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

DEFAULT_ROUTE_NAME = ["行き先案内", "Route Information"]
V2X_FIXED_ROUTE_NAME = ["行き先案内", ""]
DEFAULT_DEPARTURE_NAME = "出発点; Start"
DEFAULT_ARRIVAL_NAME = "終点; Last Stop"
PREVIOUS_STATION_INDEX = -1  # TODO: check whether is -1 or 0
NEXT_STATION_DISPLAY_AMOUNT = 6

from dataclasses import dataclass
from datetime import datetime
from dateutil import parser
from itertools import cycle
from rclpy.duration import Duration


@dataclass
class TaskList:
    doing_list: list
    todo_list: list
    done_list: list


@dataclass
class CurrentTask:
    departure_station: list
    arrival_station: list
    depart_time: int


@dataclass
class ScheduleDetails:
    updated_time: str
    schedule_id: str
    schedule_type: str


@dataclass
class DisplayDetails:
    route_name: list
    previous_station: list
    next_station_list: list


def init_TaskList():
    return TaskList([], [], [])


def init_CurrentTask():
    return CurrentTask(["", ""], ["", ""], 0)


def init_ScheduleDetails():
    return ScheduleDetails("", "", "")


def init_DisplayDetails():
    return DisplayDetails(DEFAULT_ROUTE_NAME, ["", ""], [["", ""] * 5])


def check_schedule_update(schedule_details, data):
    return (
        schedule_details.updated_time == data["updated_at"]
        and schedule_details.schedule_id == data["schedule_id"]
    )


def update_schedule_details(data):
    return ScheduleDetails(data["updated_at"], data["schedule_id"], data["schedule_type"])


def process_tag(tags_list, key):
    for item in tags_list:
        if item["key"] == key:
            return item["value"]
    return ""


def split_name(name_string):
    name_list = name_string.split(";")
    if len(name_list) < 2:
        name_list.append("")
    return name_list


def get_route_name(tag_list):
    route_name = process_tag(tag_list, "route_name")
    if not route_name:
        return DEFAULT_ROUTE_NAME
    return split_name(route_name)


def separate_task_list(task_list):
    doing_list = []
    todo_list = []
    done_list = []
    for task in task_list:
        if task["task_type"] == "move":
            if task["status"] in ["doing"]:
                doing_list.append(task)
            elif task["status"] in ["todo"]:
                todo_list.append(task)
            elif task["status"] in ["done"]:
                done_list.append(task)
    return TaskList(doing_list, todo_list, done_list)


def process_current_task(task):
    if task.get("origin", "").get("name", ""):
        departure_station = split_name(task.get("origin", "").get("name", ""))
    else:
        departure_station = split_name(DEFAULT_DEPARTURE_NAME)

    if task.get("destination", "").get("name", ""):
        arrival_station = split_name(task.get("destination", "").get("name", ""))
    else:
        arrival_station = split_name(DEFAULT_ARRIVAL_NAME)

    try:
        date_time_obj = parser.parse(task["plan_start_time"])
        depart_time = datetime.timestamp(date_time_obj)
    except:
        depart_time = 0

    return CurrentTask(departure_station, arrival_station, depart_time)


def get_previous_station_name_from_fms(done_list):
    previous_station_task = done_list[PREVIOUS_STATION_INDEX]
    return split_name(previous_station_task.get("origin", "").get("name", DEFAULT_DEPARTURE_NAME))


def repeat_task_for_loop(station_list):
    station_cycle = cycle(station_list)
    for _ in range(NEXT_STATION_DISPLAY_AMOUNT - len(station_list)):
        station_list.append(next(station_cycle))
    return station_list


def auto_add_empty_list(station_list):
    for _ in range(NEXT_STATION_DISPLAY_AMOUNT - len(station_list)):
        station_list.append(["", ""])


def create_next_station_list(current_task_details, todo_list, call_type, schedule_type=""):
    station_list = [current_task_details.arrival_station]

    for task in todo_list:
        station_list.append(
            split_name(task.get("destination", "").get("name", DEFAULT_ARRIVAL_NAME))
        )

    if call_type == "local" and schedule_type == "loop":
        # Need this condition, because FMS will not include the departure_station in the task
        if current_task_details.departure_station[1] != "Start":
            station_list.append(current_task_details.departure_station)

    if len(station_list) < NEXT_STATION_DISPLAY_AMOUNT and schedule_type == "loop":
        station_list = repeat_task_for_loop(station_list)

    auto_add_empty_list(station_list)

    return station_list[: NEXT_STATION_DISPLAY_AMOUNT - 1]


def get_remain_minute(depart_time, current_time):
    return (depart_time - current_time) / 60


def handle_phrase(phrase_type, remain_minute=0):
    return {
        "final": "終点です。\nご乗車ありがとうございました",
        "remain_minute": "このバスはあと{}分程で出発します".format(str(remain_minute)),
        "departing": "間もなく発車時刻です",
        "arriving": "間もなく到着します",
    }.get(phrase_type, "")


def check_timeout(current_time, trigger_time, duration):
    return current_time - trigger_time > Duration(seconds=duration)


def to_japanese_station_name(name):
    """V2X name is Japanese only; keep English slot empty for QML compatibility."""
    if not name:
        return ["", ""]
    return [name, ""]


def normalize_bus_stop_state(state_value):
    """Map OR_* variants (20+) to base BusStopState values."""
    if state_value >= 20:
        return state_value - 20
    return state_value


def process_station_list_from_v2x(signage_infos):
    """
    Build display fields from BusStopStatus[] ordered by stop sequence.

    Returns (previous_station, current_task, next_station_list, reach_final)
    """
    from tier4_v2x_msgs.msg import BusStopState

    if not signage_infos:
        return (["", ""], init_CurrentTask(), [["", ""]] * 5, False)

    names = [to_japanese_station_name(info.name) for info in signage_infos]
    states = [normalize_bus_stop_state(info.state.value) for info in signage_infos]

    stopping_idx = None
    approaching_idx = None
    last_completed_idx = None
    for i, state in enumerate(states):
        if state == BusStopState.STOPPING and stopping_idx is None:
            stopping_idx = i
        if state == BusStopState.APPROACHING and approaching_idx is None:
            approaching_idx = i
        if state == BusStopState.STOP_COMPLETED:
            last_completed_idx = i

    if stopping_idx is not None:
        dep_idx = stopping_idx
        arr_idx = stopping_idx + 1
    elif approaching_idx is not None:
        dep_idx = approaching_idx - 1
        arr_idx = approaching_idx
    elif last_completed_idx is not None:
        dep_idx = last_completed_idx
        arr_idx = last_completed_idx + 1
    else:
        # Before departure: treat first stop as current
        dep_idx = 0
        arr_idx = 1 if len(names) > 1 else -1

    previous_station = names[dep_idx - 1] if dep_idx > 0 else ["", ""]
    departure_station = names[dep_idx] if dep_idx >= 0 else ["", ""]

    reach_final = False
    if arr_idx < 0 or arr_idx >= len(names):
        arrival_station = ["", ""]
        reach_final = dep_idx >= 0 and (
            last_completed_idx == dep_idx or stopping_idx == dep_idx
        )
        station_list = []
    else:
        arrival_station = names[arr_idx]
        station_list = list(names[arr_idx:])

    auto_add_empty_list(station_list)
    next_station_list = station_list[: NEXT_STATION_DISPLAY_AMOUNT - 1]

    current_task = CurrentTask(departure_station, arrival_station, 0)
    return previous_station, current_task, next_station_list, reach_final


def get_v2x_bus_stop_states(signage_infos):
    return {
        info.stop_id: normalize_bus_stop_state(info.state.value) for info in signage_infos
    }


def get_v2x_approaching(signage_infos):
    from tier4_v2x_msgs.msg import BusStopState

    if not signage_infos:
        return False
    for info in signage_infos:
        if normalize_bus_stop_state(info.state.value) == BusStopState.APPROACHING:
            return True
    return False


def detect_will_stop_to_approaching(signage_infos, prev_states):
    """
    Detect WILL_STOP/OR_WILL_STOP -> APPROACHING/OR_APPROACHING.
    OR_* is normalized to base values before comparison.

    Returns (became_approaching, current_states)
    """
    from tier4_v2x_msgs.msg import BusStopState

    current_states = get_v2x_bus_stop_states(signage_infos)
    became_approaching = False
    for stop_id, state in current_states.items():
        prev = prev_states.get(stop_id)
        if prev == BusStopState.WILL_STOP and state == BusStopState.APPROACHING:
            became_approaching = True
            break
    return became_approaching, current_states
