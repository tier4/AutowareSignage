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


def _is_signage_target_state(state):
    """States that keep the stop on the signage until the bus departs."""
    from tier4_v2x_msgs.msg import BusStopState

    return state in (
        BusStopState.WILL_PASS,
        BusStopState.WILL_STOP,
        BusStopState.APPROACHING,
        BusStopState.STOPPING,
        BusStopState.PASSING,
    )


def find_next_bus_stop_index(signage_infos):
    """First stop from the front that is still ahead of departure."""
    for index, info in enumerate(signage_infos):
        if _is_signage_target_state(normalize_bus_stop_state(info.state.value)):
            return index
    return None


def process_station_list_from_v2x(signage_infos):
    """
    Next stop is the first 停車予定/通過予定/バス停直前/停車中/通過中.
    The display moves on after STOP_COMPLETED or PASS_COMPLETED.

    Marker roles match FMS:
    - blue marker (departure) is the origin of the current leg,
      the stop immediately before the next stop
    - gray marker (previous) is the stop before that departure
    - upcoming markers start at the next stop

    Returns (previous_station, current_task, next_station_list, reach_final)
    """
    if not signage_infos:
        return (["", ""], init_CurrentTask(), [["", ""]] * 5, False)

    names = [to_japanese_station_name(info.name) for info in signage_infos]
    next_idx = find_next_bus_stop_index(signage_infos)

    if next_idx is None:
        departure_index = len(names) - 1
        arrival_station = ["", ""]
        station_list = []
        reach_final = True
    else:
        departure_index = next_idx - 1
        arrival_station = names[next_idx]
        station_list = list(names[next_idx:])
        reach_final = False

    departure_station = names[departure_index] if departure_index >= 0 else ["", ""]
    previous_station = names[departure_index - 1] if departure_index >= 1 else ["", ""]
    auto_add_empty_list(station_list)
    next_station_list = station_list[: NEXT_STATION_DISPLAY_AMOUNT - 1]
    current_task = CurrentTask(departure_station, arrival_station, 0)
    return previous_station, current_task, next_station_list, reach_final


def detect_v2x_signage_changes(signage_infos, prev_status):
    """
    Compare signage_infos with the previous snapshot.

    prev_status: {stop_id: (normalized_state, will_stop)}
    will_stop.wav plays when a stop changes 通過予定 -> 停車予定,
    or its will_stop changes false -> true.
    going_to_arrive plays only when the displayed next stop changes from
    停車予定 (WILL_STOP) to バス停直前 (APPROACHING) and will_stop is true.
    「間もなく到着します」 is shown only while that stop will actually stop.
    A 通過予定 stop does not play going_to_arrive or show the phrase,
    including when it becomes バス停直前 with will_stop still false.
    thank_you plays when that stop then changes to 停車 (STOPPING=4).
    OR_* states are normalized before comparison.

    Returns (
        play_will_stop,
        became_approaching,
        is_approaching,
        show_arriving,
        became_stopping,
        current_status,
    )
    """
    from tier4_v2x_msgs.msg import BusStopState

    current_status = {}
    play_will_stop = False
    became_approaching = False
    is_approaching = False
    show_arriving = False
    became_stopping = False

    for info in signage_infos:
        state = normalize_bus_stop_state(info.state.value)
        will_stop = bool(info.will_stop)
        current_status[info.stop_id] = (state, will_stop)

        prev = prev_status.get(info.stop_id) if prev_status else None
        if prev is None:
            continue
        prev_state, prev_will_stop = prev
        if prev_state == BusStopState.WILL_PASS and state == BusStopState.WILL_STOP:
            play_will_stop = True
        if not prev_will_stop and will_stop:
            play_will_stop = True

    next_idx = find_next_bus_stop_index(signage_infos)
    if next_idx is not None:
        info = signage_infos[next_idx]
        state = normalize_bus_stop_state(info.state.value)
        prev = prev_status.get(info.stop_id) if prev_status else None
        prev_state = prev[0] if prev is not None else None
        if state == BusStopState.APPROACHING:
            is_approaching = True
            # 通過予定（WILL_PASS、または will_stop が false）のまま直前に入った
            # 停留所は「間もなく到着します」も going_to_arrive も出さない
            if bool(info.will_stop):
                show_arriving = True
                if prev_state == BusStopState.WILL_STOP:
                    became_approaching = True
        elif state == BusStopState.STOPPING and prev_state not in (None, BusStopState.STOPPING):
            became_stopping = True

    return (
        play_will_stop,
        became_approaching,
        is_approaching,
        show_arriving,
        became_stopping,
        current_status,
    )
