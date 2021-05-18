import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: busStopName
    width: 1920
    height: 360
    color: "#ffffff"
    Text {
        id: busRouteText
        x: 288
        y: 207
        color: "#000000"
        text: qsTr("バス停名")
        anchors.verticalCenterOffset: -100
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 68
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
    }

    Text {
        id: busRouteTextEn
        x: 853
        y: 131
        width: 213
        height: 39
        color: "#000000"
        text: qsTr("Bus Stop Name")
        anchors.top: busRouteText.bottom
        anchors.topMargin: -5
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 36
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
        anchors.top: busRouteText.bottom
        anchors.topMargin: 30

        Rectangle {
            id: beforeAndAfterBar
            x: 173
            width: 1700
            height: 44
            color: "#69bfd2"
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Rectangle {
            id: rectangle2
            x: 870
            y: 231
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
                x: -15
                y: 25
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
            y: -26
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
                x: 179
                y: 62
                width: 294
                height: 49
                color: "#717171"
                text: qsTr("前のバス停名")
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 48
                font.bold: true
                verticalAlignment: Text.AlignVCenter
            }

            Text {
                id: beforeBusStopNameEn
                x: 179
                y: 116
                width: 213
                height: 39
                color: "#717171"
                text: qsTr("Last Bus Stop")
                anchors.top: beforeBusStopName.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: beforeBusStopName.horizontalCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 24
                font.bold: true
                verticalAlignment: Text.AlignVCenter
            }
        }

        Rectangle {
            id: nextBusStopMarker
            x: 1502
            y: -21
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.right: parent.right
            anchors.rightMargin: 300
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            border.color: "#717171"
            border.width: 10

            Text {
                id: nextBusStopName
                x: -117
                y: 62
                width: 294
                height: 49
                text: qsTr("次のバス停名")
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 48
            }

            Text {
                id: nextBusStopNameEn
                x: -77
                y: 116
                width: 213
                height: 39
                color: "#000000"
                text: qsTr("Next Bus Stop")
                anchors.horizontalCenter: nextBusStopName.horizontalCenter
                anchors.top: nextBusStopName.bottom
                anchors.topMargin: 5
                horizontalAlignment: Text.AlignHCenter
                font.bold: true
                font.pixelSize: 24
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
}

/*##^## Designer {
    D{i:2;anchors_x:179;anchors_y:-45}D{i:7;anchors_y:89}D{i:8;anchors_x:179;anchors_y:82}
D{i:9;anchors_x:179;anchors_y:82}D{i:10;anchors_x:179;anchors_y:82}
}
 ##^##*/

