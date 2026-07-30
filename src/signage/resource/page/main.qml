import QtQuick 2.9
import QtQuick.Window 2.9
import QtQuick.Controls 2.2

import "BusStopView"
import "BusMoveView"
import "EmergencyStopView"
import "CvmDisplayView"

Window {
    id: window
    visible: true
    title: qsTr("案内板")

    width: viewController.monitor_width
    height: viewController.monitor_height

    EmergencyStopView {
        id: emergencyStopView
        visible: viewController.view_mode === "emergency_slowing" || viewController.view_mode === "emergency_stopped"
    }

    CvmDisplayView {
        id: cvmDisplayView
        visible: viewController.view_mode === "cvm_display"
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

    StandingWarningView {
        id: standingWarningView
        visible: viewController.view_mode === "standing_sudden_warning"
              || viewController.view_mode === "standing_depart_warning"
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

    ArrivedView {
        id: arrivedView
        visible: viewController.view_mode === "arrived"
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
