#!/bin/bash
# 車外サイネージ 表示モード (OFF / 自動運転状態表示 / 行先表示) の切替・確認ヘルパー。
#
# 使い方:
#   ./display_mode.sh off           # OFF        (全ディスプレイを空白にして固定)
#   ./display_mode.sh autonomous    # 自動運転状態表示 (experiment/auto/mrm/null の既存表示)
#   ./display_mode.sh destination   # 行先表示   (active_schedule から行先td5を表示)
#   ./display_mode.sh status        # 現在の設定・モード状態を確認
#
# モードは settings.json の display_mode 1 キーに永続化され、再起動しても保持される。
# どのモードへの切替も service 1 コールで完結する。
#
# 前提: ROS 2 環境と対象ワークスペースを source 済みで、external_signage ノードが
#       起動していること (signage.launch.xml を use_external_signage:=true で起動)。
set -euo pipefail

ENABLE_SERVICE="/signage/display/enable"
DESTINATION_SERVICE="/signage/display/destination"

usage() {
  echo "usage: $0 {off|autonomous|destination|status}" >&2
  exit 1
}

[ $# -eq 1 ] || usage

case "$1" in
  off)
    echo "[display_mode] OFF にします..."
    ros2 service call "$ENABLE_SERVICE" std_srvs/srv/SetBool "{data: false}"
    ;;
  autonomous|auto)
    echo "[display_mode] 自動運転状態表示にします..."
    ros2 service call "$DESTINATION_SERVICE" std_srvs/srv/SetBool "{data: false}"
    ;;
  destination|dest)
    echo "[display_mode] 行先表示にします..."
    ros2 service call "$DESTINATION_SERVICE" std_srvs/srv/SetBool "{data: true}"
    ;;
  status)
    echo "[display_mode] 現在の設定 (/signage/external/settings) を1件取得:"
    timeout 5 ros2 topic echo --once /signage/external/settings || \
      echo "  (settings を取得できませんでした。ノード起動を確認してください)"
    echo "[display_mode] mode_status (/signage/mode_status) を1件取得:"
    timeout 5 ros2 topic echo --once /signage/mode_status || \
      echo "  (mode_status を取得できませんでした)"
    ;;
  *)
    usage
    ;;
esac
