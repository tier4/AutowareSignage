import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: busStopView
    width: 1920
    height: 360

    property int counter: 0

    Timer {
        interval: 10000
        running: true
        repeat: true
        onTriggered: {
            busStopView.counter = busStopView.counter + 1
        }
    }

    function getCount() {
        if (viewController.display_time) {
            return 3
        } else {
            return 2
        }
    }

    BusStopName {
        visible: busStopView.counter % getCount() === 0
    }

    BusRouteName {
        visible: busStopView.counter % getCount() === 1
    }

    TimeRemaining {
        visible: viewController.display_time && busStopView.counter % getCount() === 2
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

/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/

