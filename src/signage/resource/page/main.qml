import QtQuick 2.9
import QtQuick.Window 2.9
import QtQuick.Controls 2.2

import "BusStopView"
import "BusMoveView"

Window {
    id: window
    visible: true
    title: qsTr("案内板")

    width: 1920
    height: 360

    EmergencyStopView {
        id: emergencyStopView
        visible: viewController.view_mode === "emergency_stopped"
    }

    ManualDriving {
        id: manualDriving
        visible: viewController.view_mode === "manual_driving"
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

/*##^## Designer {
    D{i:0;height:360;width:1920}
}
 ##^##*/

