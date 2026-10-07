import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: busStopView
    width: viewController.monitor_width
    height: viewController.monitor_height

    property int counter: 0
    readonly property int pageCount: viewController.show_time_remaining ? 3 : 2

    Timer {
        interval: 10000
        running: true
        repeat: true
        onTriggered: {
            busStopView.counter = busStopView.counter + 1
        }
    }

    BusStopName {
        visible: busStopView.counter % busStopView.pageCount === 0
    }

    BusRouteName {
        visible: busStopView.counter % busStopView.pageCount === 1
    }

    TimeRemaining {
        visible: viewController.show_time_remaining && busStopView.counter % busStopView.pageCount === 2
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
