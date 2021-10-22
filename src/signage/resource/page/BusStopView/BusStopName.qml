import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: busStopName
    width: 1920
    height: 360
    color: "#ffffff"
    Text {
        id: currentBusText
        color: "#000000"
        text: viewController.departure_station_name[0]
        anchors.verticalCenterOffset: -100
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 50
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
    }

    Text {
        id: currentBusTextEn
        color: "#000000"
        text: viewController.departure_station_name[1]
        anchors.top: currentBusText.bottom
        anchors.topMargin: -5
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 28
        font.bold: true
        verticalAlignment: Text.AlignVCenter
    }

    Item {
        id: beforeAndAfterBusStop
        height: 100
        anchors.right: parent.right
        anchors.rightMargin: 0
        anchors.left: parent.left
        anchors.leftMargin: 0
        anchors.top: currentBusText.bottom
        anchors.topMargin: 30

        Rectangle {
            id: beforeAndAfterBar
            width: 1700
            height: 44
            color: "#69bfd2"
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Rectangle {
            id: rectangle2
            width: 90
            height: 90
            color: "#ffffff"
            radius: 45
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            border.color: "#717171"
            border.width: 5

            Rectangle {
                id: rectangle1
                width: 70
                height: 70
                color: "#0068b6"
                radius: 35
                border.width: 0
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Rectangle {
            id: beforeBusStopMarker
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.left: parent.left
            anchors.leftMargin: 300
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            border.color: "#717171"
            border.width: 10

            Text {
                id: beforeBusStopName
                width: 500
                color: "#717171"
                text: viewController.previous_station_list[0][0]
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 40
                font.bold: true
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
            }
            Text {
                id: beforeBusStopNameEn
                width: 500
                color: "#717171"
                text: viewController.previous_station_list[0][1]
                anchors.top: beforeBusStopName.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: beforeBusStopName.horizontalCenter
                font.pixelSize: 30
                font.bold: true
                elide: Text.ElideRight
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            visible : (viewController.previous_station_list[0][0] || viewController.previous_station_list[0][0].length !== 0)
        }

        Rectangle {
            id: nextBusStopMarker
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.right: parent.right
            anchors.rightMargin: 300
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            border.color: "#0068b6"
            border.width: 10

            Text {
                id: nextBusStopName
                width: 500
                text: viewController.arrival_station_name[0]
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                font.bold: true
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 40
            }
            Text {
                id: nextBusStopNameEn
                width: 500
                color: "#000000"
                text: viewController.arrival_station_name[1]
                anchors.horizontalCenter: nextBusStopName.horizontalCenter
                anchors.top: nextBusStopName.bottom
                anchors.topMargin: 5
                horizontalAlignment: Text.AlignHCenter
                font.bold: true
                font.pixelSize: 30
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
            }
            visible : (viewController.arrival_station_name[0] || viewController.arrival_station_name[0].length !== 0)
        }
    }
    function getDepatureStationName() {
        var station_name = viewController.departure_station_name
        if(!station_name || station_name.length === 0)
        {
            station_name = qsTr("バス停名")
        }
        return station_name
    }
}




/*##^## Designer {
    D{i:2;anchors_x:179;anchors_y:"-45"}D{i:8;anchors_x:179;anchors_y:82}D{i:9;anchors_x:179;anchors_y:82}
D{i:7;anchors_y:89}D{i:10;anchors_x:179;anchors_y:82}
}
 ##^##*/
