# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

DEFAULT_ROUTE_NAME = "行き先案内; Route Information"
DEFAULT_DEPARTURE_NAME = "出発点; Start"
DEFAULT_ARRIVAL_NAME = "終点; Last Stop"
PREVIOUS_STATION_INDEX = -1  # TODO: check whether is -1 or 0
NEXT_STATION_DISPLAY_AMOUNT = 6

from datetime import datetime
from dateutil import parser
from itertools import cycle


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
        route_name = DEFAULT_ROUTE_NAME
    return split_name(route_name)


def seperate_task_list(seperated_task_list, task_list):
    for task in task_list:
        if task["task_type"] == "move":
            if task["status"] in ["doing"]:
                seperated_task_list["doing_list"].append(task)
            elif task["status"] in ["todo"]:
                seperated_task_list["todo_list"].append(task)
            elif task["status"] in ["done"]:
                seperated_task_list["done_list"].append(task)
    return seperated_task_list


def process_current_task(current_task_details, task):
    if task.get("origin_point_name", ""):
        current_task_details["departure_station"] = split_name(task.get("origin_point_name", ""))
    else:
        current_task_details["departure_station"] = split_name(DEFAULT_DEPARTURE_NAME)

    if task.get("destination_point_name", ""):
        current_task_details["arrival_station"] = split_name(task.get("destination_point_name", ""))
    else:
        current_task_details["arrival_station"] = split_name(DEFAULT_ARRIVAL_NAME)

    try:
        date_time_obj = parser.parse(task["plan_start_time"])
        current_task_details["depart_time"] = datetime.timestamp(date_time_obj)
    except:
        current_task_details["depart_time"] = 0


def get_prevous_station_name_from_fms(done_list):
    previous_station_task = done_list[PREVIOUS_LIST_INDEX]
    return split_name(previous_station_task.get("origin_point_name", DEFAULT_DEPARTURE_NAME))


def repeat_task_for_loop(station_list):
    station_cycle = cycle(station_list)
    for _ in range(NEXT_STATION_DISPLAY_AMOUNT - len(station_list)):
        station_list.append(next(station_cycle))
    return station_list


def auto_add_empty_list(station_list):
    for _ in range(NEXT_STATION_DISPLAY_AMOUNT - len(station_list)):
        station_list.append(["", ""])
    return station_list


def create_next_station_list(
    current_task_details, seperated_task_list, call_type, schedule_type=""
):
    station_list = [current_task_details["arrival_station"]]

    for task in seperated_task_list["todo_list"]:
        station_list.append(split_name(task.get("destination_point_name", DEFAULT_ARRIVAL_NAME)))

    if call_type == "local" and schedule_type == "loop":
        if current_task_details["departure_station"][1] != "Start":
            station_list.append(split_name(current_task_details["departure_station"]))

    if len(station_list) < NEXT_STATION_DISPLAY_AMOUNT and schedule_type == "loop":
        station_list = repeat_task_for_loop(station_list)

    auto_add_empty_list(station_list)

    return station_list[: NEXT_STATION_DISPLAY_AMOUNT - 1]
