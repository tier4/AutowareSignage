#!/bin/bash
# 車外サイネージ 行先表示 (SYS-HMI-03) テスト用ヘルパー。
# active_schedule を各パターンで publish、または web.auto agent から実スケジュールを
# 取得して publish / 表示する。
#
# 前提:
#   - ROS 2 環境と対象ワークスペースを source 済み
#   - external_signage ノードが起動済み (signage.launch.xml use_external_signage:=true)
#   - 行先表示モードが ON (./destination_mode.sh on)  ※OFF だと描画に反映されない
#
# 使い方:
#   ./publish_test_schedule.sh doing [POINT_ID]    # 走行中(doing): 登録済み point_id -> 行先td5
#   ./publish_test_schedule.sh todo  [POINT_ID]    # 次タスク(todo): 同上
#   ./publish_test_schedule.sh done  [POINT_ID]    # 全タスク完了     -> 回送中 (kaiso)
#   ./publish_test_schedule.sh none                # スケジュール空   -> 回送中 (kaiso)
#   ./publish_test_schedule.sh unmapped [POINT_ID] # 未登録 point_id  -> 空白 (null)
#   ./publish_test_schedule.sh no-id               # point_id 無し     -> 空白 (null)
#   ./publish_test_schedule.sh real                # web.auto agent の実スケジュールを publish
#   ./publish_test_schedule.sh show                # 実スケジュールを整形表示 (publish しない)
#
# POINT_ID 省略時は destination_mapping.yaml に登録済みの実 point_id を使う:
#   doing -> 91737  (日本科学未来館 / miraikan)
#   todo  -> 118489 (東京テレポート駅 / teleport)
#   done  -> doing/todo の move タスクが無くなるため回送中 (kaiso)。point_id は表示に影響しない
#   unmapped -> 999999 (未登録想定)
# ※ destination_mapping.yaml を更新したら、下の *_POINT_ID も併せて更新すること。
#    未登録の point_id のままだと doing/todo が空白 (null) 表示になり、テストの意図と食い違う。
#
# 期待結果はノードログ "destination display -> <key> (<detail>)" で確認する。
# td5 が一部サイズのみの場合 (現状: 行先/回送 td5 は 128x16 のみ) は、そのサイズを持つ
# front/back に表示され、80x24 未用意の side は "td5 not available ... showing null" で空白表示。
# 該当サイズの td5 が全く無い key は全ディスプレイ空白 (null) となる。
set -euo pipefail

TOPIC="/signage/active_schedule"
# ベンチ登録スケジュール (area_map_id 1155 お台場エリア) の move タスク2件の実 point_id。
# doing/todo で別の行先を出すことで、_resolve_destination の doing[0] -> todo[0] フォールバックが
# どちらの経路で解決したかをノードログの行先名で見分けられる。
DOING_POINT_ID=91737
TODO_POINT_ID=118489
UNMAPPED_POINT_ID=999999

AGENT_URL="http://${AUTOWARE_IP:-localhost}:${AUTOWARE_PORT:-4711}/v1/services/order"
FMS_ACTIVE_SCHEDULE_URL="https://${FMS_URL:-fms.dev.web.auto}/v1/projects/{project_id}/environments/{environment_id}/vehicles/{vehicle_id}/active_schedule"

usage() {
  sed -n '2,32p' "$0" >&2
  exit 1
}

# YAML 単一引用符スカラー内に埋め込むため、JSON 中の ' を '' へエスケープして publish する。
publish() {
  local json="$1"
  local esc="${json//\'/\'\'}"
  echo "[publish] ${TOPIC} <- ${json}"
  ros2 topic pub --times 3 --rate 2 "${TOPIC}" std_msgs/msg/String "{data: '${esc}'}"
}

# move タスク1件のスケジュール JSON を組み立てる。
#   $1 = task status (doing/todo/done ...)
#   $2 = point_id (空文字なら destination に point_id を入れない = no-id ケース)
move_schedule() {
  local status="$1" pid="${2-}"
  if [ -z "${pid}" ]; then
    printf '{"tasks":[{"task_type":"move","status":"%s","destination":{"name":"TestStop"}}]}' "${status}"
  else
    printf '{"tasks":[{"task_type":"move","status":"%s","destination":{"point_id":%s,"stop_point_id":1,"name":"TestStop"}}]}' "${status}" "${pid}"
  fi
}

fetch_real() {
  curl -s --max-time 12 -X POST "${AGENT_URL}" -H 'Content-Type: application/json' \
    -d "{\"method\":\"get\",\"url\":\"${FMS_ACTIVE_SCHEDULE_URL}\",\"body\":{}}"
}

[ $# -ge 1 ] || usage
cmd="$1"; shift || true

case "${cmd}" in
  doing)
    publish "$(move_schedule doing "${1:-${DOING_POINT_ID}}")"
    ;;
  todo)
    publish "$(move_schedule todo "${1:-${TODO_POINT_ID}}")"
    ;;
  done)
    # 全タスク完了 -> doing/todo の move タスクなし = 回送中 (kaiso)。point_id は表示に影響しない
    publish "$(move_schedule "done" "${1:-${DOING_POINT_ID}}")"
    ;;
  none)
    # 空スケジュール -> _resolve_destination は "kaiso" (no schedule)
    publish '{}'
    ;;
  unmapped)
    publish "$(move_schedule doing "${1:-${UNMAPPED_POINT_ID}}")"
    ;;
  no-id)
    # destination に point_id を入れない -> "null" (no destination point_id)
    publish "$(move_schedule doing "")"
    ;;
  real)
    echo "[real] web.auto agent (${AGENT_URL}) から実スケジュールを取得して publish します..."
    raw="$(fetch_real)"
    [ -n "${raw}" ] || { echo "  取得できませんでした (agent 未起動/認証/ID 未解決の可能性)" >&2; exit 1; }
    publish "${raw}"
    ;;
  show)
    echo "[show] web.auto agent (${AGENT_URL}) の実スケジュール:"
    raw="$(fetch_real)"
    [ -n "${raw}" ] || { echo "  取得できませんでした" >&2; exit 1; }
    if command -v python3 >/dev/null 2>&1; then
      echo "${raw}" | python3 -m json.tool --no-ensure-ascii 2>/dev/null || echo "${raw}"
    else
      echo "${raw}"
    fi
    ;;
  *)
    usage
    ;;
esac
