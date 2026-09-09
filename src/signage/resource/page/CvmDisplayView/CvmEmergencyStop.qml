import QtQuick 2.9
import QtQuick.Controls 2.2
import "../EmergencyStopView"

// 遠隔緊急 (CVM remote_emergency_display) の表示は、autoware MRM の停止後表示
// EmergencyStopping と完全に同一にする (#3 デザイン統一)。文言・配色を一元管理
// するため独自レイアウトを持たず EmergencyStopping コンポーネントを再利用する。
EmergencyStopping {
}
