import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: busStopView
    width: viewController.monitor_width
    height: viewController.monitor_height

    property int counter: 0

    Timer {
        interval: 10000
        running: true
        repeat: true
        onTriggered: {
            busStopView.counter = busStopView.counter + 1
        }
    }

    BusRouteName {
        visible: busStopView.counter % 2 === 0
    }

    BusStopList {
        visible: busStopView.counter % 2 === 1
    }

    states: [
        State {
            name: "init"
            when: viewController.view_mode === "stopping"
            StateChangeScript {
                name: "init value"
                script: {
                    busStopView.counter = 0
                }
            }
        }
    ]
}
