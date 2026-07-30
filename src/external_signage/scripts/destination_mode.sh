#!/bin/bash
# 車外サイネージ 行先表示モード (SYS-HMI-03) の切替・確認ヘルパー。
#
# 使い方:
#   ./destination_mode.sh on       # 行先表示モードON  (active_schedule から行先td5を表示)
#   ./destination_mode.sh off      # 行先表示モードOFF (既存の experiment/auto/mrm/null 表示へ復帰)
#   ./destination_mode.sh status   # 現在の設定・モード状態を確認
#
# 前提: ROS 2 環境と対象ワークスペースを source 済みで、external_signage ノードが
#       起動していること (signage.launch.xml を use_external_signage:=true で起動)。
set -euo pipefail

SERVICE="/signage/destination_mode"

usage() {
  echo "usage: $0 {on|off|status}" >&2
  exit 1
}

[ $# -eq 1 ] || usage

case "$1" in
  on|true)
    echo "[destination_mode] ON にします..."
    ros2 service call "$SERVICE" std_srvs/srv/SetBool "{data: true}"
    ;;
  off|false)
    echo "[destination_mode] OFF にします..."
    ros2 service call "$SERVICE" std_srvs/srv/SetBool "{data: false}"
    ;;
  status)
    echo "[destination_mode] 現在の設定 (/signage/external/settings) を1件取得:"
    timeout 5 ros2 topic echo --once /signage/external/settings || \
      echo "  (settings を取得できませんでした。ノード起動を確認してください)"
    echo "[destination_mode] mode_status (/signage/mode_status) を1件取得:"
    timeout 5 ros2 topic echo --once /signage/mode_status || \
      echo "  (mode_status を取得できませんでした)"
    ;;
  *)
    usage
    ;;
esac
