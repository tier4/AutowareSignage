import QtQuick 2.9
import QtQuick.Window 2.9
import QtQuick.Controls 2.2

import "BusStopView"
import "BusMoveView"
import "EmergencyStopView"
import "Common"
import "Door"

Window {
    id: window
    visible: true
    title: qsTr("案内板")

    width: viewController.monitor_width
    height: viewController.monitor_height

    FrontDoorOpen {
        id: frontDoorOpenView
        visible: viewController.view_mode === "front_door"
    }

    MiddleDoorOpen {
        id: middleDoorOpenView
        visible: viewController.view_mode === "middle_door"
    }

    BothDoorsOpen {
        id: bothDoorsOpenView
        visible: viewController.view_mode === "both_doors"
    }

    EmergencyStopView {
        id: emergencyStopView
        visible: viewController.view_mode === "emergency_stopped"
    }

    Disconnected {
        id: disconnectedView
        visible: viewController.view_mode === "disconnected"
    }

    SlowStopView {
        id: slowStopView
        visible: viewController.view_mode === "slow_stop"
    }

    SlowingView {
        id: slowingView
        visible: viewController.view_mode === "slowing"
    }

    ManualDriving {
        id: manualDriving
        visible: viewController.view_mode === "manual_driving"
    }

    AutoDriving {
        id: autoDriving
        visible: viewController.view_mode === "auto_driving"
    }

    OutOfService {
        id: outOfService
        visible: viewController.view_mode === "out_of_service"
    }

    BusStopView {
        id: busStopView
        visible: viewController.view_mode === "stopping"
    }

    BusMoveView {
        id: busMoveView
        visible: viewController.view_mode === "driving"
    }

    Item {
        focus: true
        Keys.onPressed: {
            if (event.key === Qt.Key_Escape) {
                window.visibility = "Windowed"
            } else if (event.key === Qt.Key_Return) {
                window.visibility = "FullScreen"
            }
        }
        Component.onCompleted: {
            window.visibility = "FullScreen"
        }
    }
}
