import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: emergencyStopView
    width: viewController.monitor_width
    height: viewController.monitor_height

    property int counter: 0

    Timer {
        interval: 10000
        running: true
        repeat: true
        onTriggered: {
            emergencyStopView.counter = emergencyStopView.counter + 1
        }
    }

    EmergencyStop {
        visible: emergencyStopView.counter % 2 === 0
    }

    EmergencyStopping {
        visible: emergencyStopView.counter % 2 === 1
    }

    states: [
        State {
            name: "init"
            when: viewController.view_mode === "emergency_stopped"
            StateChangeScript {
                name: "init value"
                script: {
                    emergencyStopView.counter = 0
                }
            }
        }
    ]
}
