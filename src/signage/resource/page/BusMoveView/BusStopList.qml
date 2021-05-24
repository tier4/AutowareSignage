import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: busStopName
    width: 1920
    height: 360
    color: "#ffffff"

    Item {
        id: beforeAndAfterBusStop
        height: 100
        anchors.verticalCenter: parent.verticalCenter
        anchors.right: parent.right
        anchors.rightMargin: 0
        anchors.left: parent.left
        anchors.leftMargin: 0

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
            id: beforeBusStopMarker3
            x: 1507
            y: -26
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.right: beforeBusStopMarker2.left
            anchors.rightMargin: 180
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            border.color: "#717171"
            border.width: 10

            Text {
                id: beforeBusStopName
                x: 179
                y: 62
                width: 150
                height: 35
                color: "#717171"
                text: qsTr("前のバス停名")
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 24
                font.bold: true
                verticalAlignment: Text.AlignVCenter
            }

            Text {
                id: beforeBusStopNameEn
                x: 179
                y: 116
                width: 100
                height: 20
                color: "#717171"
                text: qsTr("Last Bus Stop")
                anchors.top: beforeBusStopName.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: beforeBusStopName.horizontalCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 14
                font.bold: true
                verticalAlignment: Text.AlignVCenter
            }
        }

        Rectangle {
            id: nextBusStopMarker3
            y: -21
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.left: nextBusStopMarker2.right
            anchors.leftMargin: 180
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            border.color: "#717171"
            border.width: 10

            Text {
                id: nextBusStopName
                x: 8
                y: 62
                width: 150
                height: 35
                text: qsTr("次のバス停名")
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 24
            }

            Text {
                id: nextBusStopNameEn
                x: -77
                y: 116
                width: 100
                height: 20
                color: "#000000"
                text: qsTr("Next Bus Stop")
                anchors.horizontalCenter: nextBusStopName.horizontalCenter
                anchors.top: nextBusStopName.bottom
                anchors.topMargin: 5
                horizontalAlignment: Text.AlignHCenter
                font.bold: true
                font.pixelSize: 14
                verticalAlignment: Text.AlignVCenter
            }
        }

        Rectangle {
            id: currentBusStopMarker
            x: 937
            y: 20
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            border.width: 10
            border.color: "#717171"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            Text {
                id: nextBusStopName1
                x: 8
                y: 62
                width: 150
                height: 35
                text: qsTr("次のバス停名")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 24
                anchors.topMargin: 2
                anchors.top: parent.bottom
            }

            Text {
                id: nextBusStopNameEn1
                x: -77
                width: 100
                height: 20
                color: "#000000"
                text: qsTr("Next Bus Stop")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 14
                anchors.topMargin: 5
                anchors.top: nextBusStopName1.bottom
            }
        }

        Rectangle {
            id: nextBusStopMarker1
            y: 27
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: currentBusStopMarker.right
            anchors.leftMargin: 180
            border.width: 10
            border.color: "#717171"
            Text {
                id: nextBusStopName2
                x: 8
                y: 62
                width: 150
                height: 35
                text: qsTr("次のバス停名")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 24
                font.bold: true
                anchors.topMargin: 2
                anchors.top: parent.bottom
            }

            Text {
                id: nextBusStopNameEn2
                x: -77
                width: 100
                height: 20
                color: "#000000"
                text: qsTr("Next Bus Stop")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 14
                font.bold: true
                anchors.topMargin: 5
                anchors.top: nextBusStopName2.bottom
            }
        }

        Rectangle {
            id: nextBusStopMarker2
            y: 20
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.verticalCenter: parent.verticalCenter
            border.width: 10
            anchors.leftMargin: 180
            border.color: "#717171"
            anchors.left: nextBusStopMarker1.right
            Text {
                id: nextBusStopName3
                x: 8
                y: 62
                width: 150
                height: 35
                text: qsTr("次のバス停名")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 24
                anchors.topMargin: 2
                anchors.top: parent.bottom
            }

            Text {
                id: nextBusStopNameEn3
                x: -77
                width: 100
                height: 20
                color: "#000000"
                text: qsTr("Next Bus Stop")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 14
                anchors.topMargin: 5
                anchors.top: nextBusStopName3.bottom
            }
        }

        Rectangle {
            id: beforeBusStopMarker1
            x: 1507
            y: -120
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.right: currentBusStopMarker.left
            anchors.rightMargin: 180
            border.width: 10
            border.color: "#717171"
            anchors.verticalCenter: parent.verticalCenter
            Text {
                id: beforeBusStopName1
                x: 179
                y: 62
                width: 150
                height: 35
                color: "#717171"
                text: qsTr("前のバス停名")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 24
                font.bold: true
                anchors.topMargin: 2
                anchors.top: parent.bottom
            }

            Text {
                id: beforeBusStopNameEn1
                x: 179
                width: 100
                height: 20
                color: "#717171"
                text: qsTr("Last Bus Stop")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 14
                font.bold: true
                anchors.topMargin: 5
                anchors.top: beforeBusStopName1.bottom
            }
        }

        Rectangle {
            id: beforeBusStopMarker2
            x: 1502
            y: -115
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.verticalCenter: parent.verticalCenter
            border.width: 10
            border.color: "#717171"
            anchors.rightMargin: 180
            anchors.right: beforeBusStopMarker1.left
            Text {
                id: beforeBusStopName2
                x: 179
                y: 62
                width: 150
                height: 35
                color: "#717171"
                text: qsTr("前のバス停名")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 24
                anchors.topMargin: 2
                anchors.top: parent.bottom
            }

            Text {
                id: beforeBusStopNameEn2
                x: 179
                width: 100
                height: 20
                color: "#717171"
                text: qsTr("Last Bus Stop")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 14
                anchors.topMargin: 5
                anchors.top: beforeBusStopName2.bottom
            }
        }
    }
}




/*##^## Designer {
    D{i:4;anchors_x:179;anchors_y:82}D{i:5;anchors_x:179;anchors_y:82}D{i:7;anchors_y:89}
D{i:8;anchors_x:179;anchors_y:82}D{i:6;anchors_x:1502}D{i:10;anchors_y:89}D{i:11;anchors_x:179;anchors_y:116}
D{i:13;anchors_y:89}D{i:14;anchors_x:179;anchors_y:116}D{i:12;anchors_x:939}D{i:16;anchors_y:89}
D{i:17;anchors_x:179;anchors_y:116}D{i:15;anchors_x:1426}D{i:19;anchors_x:179;anchors_y:82}
D{i:20;anchors_x:179;anchors_y:116}D{i:22;anchors_x:179;anchors_y:82}D{i:23;anchors_x:179;anchors_y:116}
}
 ##^##*/
