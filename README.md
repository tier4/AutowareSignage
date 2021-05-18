# FieldOperatorApplication

## 確認環境
- Ubuntu18.04
- Python2.7
- Qt5.9
- ros melodic

## setup

### Install .IV
下記参照
https://github.com/tier4/autoware.proj

### setup FOA
```
cd FirldOperatorApplication
source {AUTOWARE_PATH}/install/setup.bash
bash setup.sh
```

### setup Autoware.IV
route配信時に自動的にdisengageを行う
autoware.proj/src/autoware/autoware.iv/system/autoware_state_monitor/launch/autoware_state_monitor.launch
```
-    <param name="disengage_on_route" value="false" />
+    <param name="disengage_on_route" value="true" />
```

lane change approvalの配信を停止する
autoware.proj/src/autoware/launcher/planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch
```
-  <node pkg="rostopic" type="rostopic" name="auto_approve_lane_change" args="pub /lane_change_approval std_msgs/Bool 'data: true' -r 10" />
```

信号の上書きtopicを反映する

autoware.proj/src/autoware/autoware.iv/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/launch/behavior_velocity_planner.launch
```
-    <remap from="~input/traffic_light_states" to="/perception/traffic_light_recognition/traffic_light_states" />
+    <remap from="~input/traffic_light_states" to="/overwrite/traffic_light_recognition/traffic_light_states" />
```

## rebuild FOA
```
cd FirldOperatorApplication
colcon build
```

## start FOA
Autoware.IVを起動しておく

```
source {AUTOWARE_PATH}/install/setup.bash
bash start.sh
```

