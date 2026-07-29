import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: emergencyStopView
    width: viewController.monitor_width
    height: viewController.monitor_height

    // comfortable stop (slowing/slow_stop) と揃えて、減速中と停止後で表示を固定する。
    // 減速中 (emergency_slowing) は EmergencyStop、停止後 (emergency_stopped) は
    // EmergencyStopping を表示する (10 秒ごとの切り替えは廃止)。
    EmergencyStop {
        visible: viewController.view_mode === "emergency_slowing"
    }

    EmergencyStopping {
        visible: viewController.view_mode === "emergency_stopped"
    }
}
