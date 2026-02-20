# tests/signage/test_signage_utils.py
import pytest
from signage.signage_utils import (
    DEFAULT_ROUTE_NAME,
    DEFAULT_DEPARTURE_NAME,
    DEFAULT_ARRIVAL_NAME,
    NEXT_STATION_DISPLAY_AMOUNT,
    TaskList,
    CurrentTask,
    ScheduleDetails,
    DisplayDetails,
    init_TaskList,
    init_CurrentTask,
    init_ScheduleDetails,
    init_DisplayDetails,
    check_schedule_update,
    update_schedule_details,
    process_tag,
    split_name,
    get_route_name,
    separate_task_list,
    process_current_task,
    get_previous_station_name_from_fms,
    repeat_task_for_loop,
    auto_add_empty_list,
    create_next_station_list,
    get_remain_minute,
    handle_phrase,
)


class TestInitFunctions:
    def test_init_task_list(self):
        result = init_TaskList()
        assert result == TaskList([], [], [])

    def test_init_current_task(self):
        result = init_CurrentTask()
        assert result == CurrentTask(["", ""], ["", ""], 0)

    def test_init_schedule_details(self):
        result = init_ScheduleDetails()
        assert result == ScheduleDetails("", "", "")

    def test_init_display_details(self):
        result = init_DisplayDetails()
        assert result.route_name == DEFAULT_ROUTE_NAME
        assert result.previous_station == ["", ""]


class TestCheckScheduleUpdate:
    def test_same_schedule(self):
        details = ScheduleDetails("2026-01-01", "schedule-1", "loop")
        data = {"updated_at": "2026-01-01", "schedule_id": "schedule-1"}
        assert check_schedule_update(details, data) is True

    def test_different_time(self):
        details = ScheduleDetails("2026-01-01", "schedule-1", "loop")
        data = {"updated_at": "2026-01-02", "schedule_id": "schedule-1"}
        assert check_schedule_update(details, data) is False

    def test_different_id(self):
        details = ScheduleDetails("2026-01-01", "schedule-1", "loop")
        data = {"updated_at": "2026-01-01", "schedule_id": "schedule-2"}
        assert check_schedule_update(details, data) is False


class TestUpdateScheduleDetails:
    def test_basic(self):
        data = {
            "updated_at": "2026-01-01",
            "schedule_id": "schedule-1",
            "schedule_type": "loop",
        }
        result = update_schedule_details(data)
        assert result == ScheduleDetails("2026-01-01", "schedule-1", "loop")


class TestProcessTag:
    def test_found(self):
        tags = [{"key": "route_name", "value": "Line A"}]
        assert process_tag(tags, "route_name") == "Line A"

    def test_not_found(self):
        tags = [{"key": "other", "value": "value"}]
        assert process_tag(tags, "route_name") == ""

    def test_empty_list(self):
        assert process_tag([], "route_name") == ""


class TestSplitName:
    def test_with_semicolon(self):
        assert split_name("Tokyo;東京") == ["Tokyo", "東京"]

    def test_without_semicolon(self):
        assert split_name("Tokyo") == ["Tokyo", ""]

    def test_multiple_semicolons(self):
        result = split_name("A;B;C")
        assert len(result) == 3


class TestGetRouteName:
    def test_with_route_name(self):
        tags = [{"key": "route_name", "value": "Route A;ルートA"}]
        assert get_route_name(tags) == ["Route A", "ルートA"]

    def test_without_route_name(self):
        tags = [{"key": "other", "value": "value"}]
        assert get_route_name(tags) == DEFAULT_ROUTE_NAME

    def test_empty_route_name(self):
        tags = [{"key": "route_name", "value": ""}]
        assert get_route_name(tags) == DEFAULT_ROUTE_NAME


class TestSeparateTaskList:
    def test_mixed_tasks(self):
        tasks = [
            {"task_type": "move", "status": "doing"},
            {"task_type": "move", "status": "todo"},
            {"task_type": "move", "status": "done"},
            {"task_type": "charge", "status": "doing"},
        ]
        result = separate_task_list(tasks)
        assert len(result.doing_list) == 1
        assert len(result.todo_list) == 1
        assert len(result.done_list) == 1

    def test_empty_list(self):
        result = separate_task_list([])
        assert result == TaskList([], [], [])

    def test_only_move_tasks(self):
        tasks = [
            {"task_type": "move", "status": "doing"},
            {"task_type": "move", "status": "doing"},
        ]
        result = separate_task_list(tasks)
        assert len(result.doing_list) == 2


class TestProcessCurrentTask:
    def test_with_origin_and_destination(self):
        task = {
            "origin": {"name": "Station A;駅A"},
            "destination": {"name": "Station B;駅B"},
            "plan_start_time": "2026-01-15T10:00:00Z",
        }
        result = process_current_task(task)
        assert result.departure_station == ["Station A", "駅A"]
        assert result.arrival_station == ["Station B", "駅B"]
        assert result.depart_time > 0

    def test_without_origin(self):
        task = {
            "origin": {"name": ""},
            "destination": {"name": "Station B;駅B"},
            "plan_start_time": "2026-01-15T10:00:00Z",
        }
        result = process_current_task(task)
        assert result.departure_station == split_name(DEFAULT_DEPARTURE_NAME)

    def test_invalid_time(self):
        task = {
            "origin": {"name": "A;B"},
            "destination": {"name": "C;D"},
            "plan_start_time": "invalid",
        }
        result = process_current_task(task)
        assert result.depart_time == 0


class TestGetPreviousStationNameFromFms:
    def test_basic(self):
        done_list = [
            {"origin": {"name": "First;最初"}},
            {"origin": {"name": "Second;2番目"}},
        ]
        result = get_previous_station_name_from_fms(done_list)
        assert result == ["Second", "2番目"]


class TestRepeatTaskForLoop:
    def test_short_list(self):
        stations = [["A", "a"], ["B", "b"]]
        result = repeat_task_for_loop(stations)
        assert len(result) == NEXT_STATION_DISPLAY_AMOUNT

    def test_already_full(self):
        stations = [["S", "s"]] * NEXT_STATION_DISPLAY_AMOUNT
        result = repeat_task_for_loop(stations)
        assert len(result) == NEXT_STATION_DISPLAY_AMOUNT


class TestAutoAddEmptyList:
    def test_pads_to_display_amount(self):
        stations = [["A", "a"]]
        auto_add_empty_list(stations)
        assert len(stations) == NEXT_STATION_DISPLAY_AMOUNT
        assert stations[-1] == ["", ""]

    def test_already_full(self):
        stations = [["S", "s"]] * NEXT_STATION_DISPLAY_AMOUNT
        auto_add_empty_list(stations)
        assert len(stations) == NEXT_STATION_DISPLAY_AMOUNT


class TestCreateNextStationList:
    def test_basic(self):
        current = CurrentTask(["Dep", "出発"], ["Arr", "到着"], 0)
        todo = [{"destination": {"name": "Next;次"}}]
        result = create_next_station_list(current, todo, "fms")
        assert result[0] == ["Arr", "到着"]
        assert len(result) == NEXT_STATION_DISPLAY_AMOUNT - 1

    def test_empty_todo(self):
        current = CurrentTask(["Dep", "出発"], ["Arr", "到着"], 0)
        result = create_next_station_list(current, [], "fms")
        assert result[0] == ["Arr", "到着"]


class TestGetRemainMinute:
    def test_positive(self):
        assert get_remain_minute(660, 60) == 10.0

    def test_zero(self):
        assert get_remain_minute(100, 100) == 0.0

    def test_negative(self):
        assert get_remain_minute(0, 60) == -1.0


class TestHandlePhrase:
    def test_final(self):
        result = handle_phrase("final")
        assert "終点" in result

    def test_remain_minute(self):
        result = handle_phrase("remain_minute", 5)
        assert "5" in result

    def test_departing(self):
        result = handle_phrase("departing")
        assert "発車" in result

    def test_arriving(self):
        result = handle_phrase("arriving")
        assert "到着" in result

    def test_unknown(self):
        assert handle_phrase("unknown") == ""
