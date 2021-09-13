import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Shapes 1.12

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
            width: 1700
            height: 44
            color: "#69bfd2"
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Rectangle {
            id: beforeBusStopMarker3
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
                id: beforeBusStopName3
                width: 220
                color: "#717171"
                text: viewController.previous_station_list[2][0]
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 20
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
            }
            Text {
                id: beforeBusStopName3En
                width: 100
                color: "#717171"
                text: viewController.previous_station_list[2][1]
                anchors.top: beforeBusStopName3.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: beforeBusStopName3.horizontalCenter
                font.pixelSize: 12
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
            }
            visible : (viewController.previous_station_list[2][0] || viewController.previous_station_list[2][0].length !== 0)
        }

        Rectangle {
            id: nextBusStopMarker3
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.left: nextBusStopMarker2.right
            anchors.leftMargin: 180
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            border.color: "#0068b6"
            border.width: 10

            Text {
                id: nextBusStopName3
                width: 220
                text: viewController.next_station_list[2][0]
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 20
                wrapMode: Text.WordWrap
            }
            Text {
                id: nextBusStopName3En
                width: 100
                color: "#000000"
                text: viewController.next_station_list[2][1]
                anchors.horizontalCenter: nextBusStopName3.horizontalCenter
                anchors.top: nextBusStopName3.bottom
                anchors.topMargin: 5
                font.bold: true
                font.pixelSize: 12
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
            }
            visible : (viewController.next_station_list[2][0] || viewController.next_station_list[2][0].length !== 0)
        }

        Rectangle {
            id: currentBusStopMarker
            width: 60
            height: 60
            color: "#0068b6"
            radius: 30
            border.width: 10
            border.color: "#ffffff"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            Text {
                id: currentBusStopName
                width: 220
                text: viewController.departure_station_name[0]
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 20
                anchors.topMargin: 2
                anchors.top: parent.bottom
                wrapMode: Text.WordWrap
            }
            Text {
                id: currentBusStopNameEn
                width: 213
                color: "#717171"
                text: viewController.departure_station_name[1]
                anchors.top: currentBusStopName.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: currentBusStopName.horizontalCenter
                font.bold: true
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.WordWrap
            }
            visible : (viewController.departure_station_name[0] || viewController.departure_station_name[0].length !== 0)
        }

        Rectangle {
            id: nextBusStopMarker1
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: currentBusStopMarker.right
            anchors.leftMargin: 180
            border.width: 10
            border.color: "#0068b6"
            Text {
                id: nextBusStopName1
                width: 220
                text: viewController.arrival_station_name[0]
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
                font.bold: true
                anchors.topMargin: 2
                anchors.top: parent.bottom
                wrapMode: Text.WordWrap
            }
            Text {
                id: nextBusStopName1En
                width: 213
                text: viewController.arrival_station_name[1]
                anchors.top: nextBusStopName1.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: nextBusStopName1.horizontalCenter
                font.bold: true
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.WordWrap
            }
            visible : (viewController.arrival_station_name[0] || viewController.arrival_station_name[0].length !== 0)
        }

        Rectangle {
            id: beforeBusStopMarker1
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
                width: 220
                color: "#717171"
                text: viewController.previous_station_list[0][0]
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
                font.bold: true
                anchors.topMargin: 2
                anchors.top: parent.bottom
                wrapMode: Text.WordWrap
            }
            Text {
                id: beforeBusStopName1En
                width: 100
                color: "#717171"
                text: viewController.previous_station_list[0][1]
                anchors.top: beforeBusStopName1.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: parent.horizontalCenters
                font.pixelSize: 14
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
            }
            visible : (viewController.previous_station_list[0][0] || viewController.previous_station_list[0][0].length !== 0)
        }

        Rectangle {
            id: nextBusStopMarker2
            width: 60
            height: 60
            color: "#ffffff"
            radius: 30
            anchors.verticalCenter: parent.verticalCenter
            border.width: 10
            anchors.leftMargin: 180
            border.color: "#0068b6"
            anchors.left: nextBusStopMarker1.right
            Text {
                id: nextBusStopName2
                width: 220
                text: viewController.next_station_list[1][0]
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 20
                anchors.topMargin: 2
                anchors.top: parent.bottom
                wrapMode: Text.WordWrap
            }
            Text {
                id: nextBusStopName2En
                width: 100
                text: viewController.next_station_list[1][1]
                anchors.top: nextBusStopName2.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 12
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
            }
            visible : (viewController.next_station_list[1][0] || viewController.next_station_list[1][0].length !== 0)
        }

        Rectangle {
            id: beforeBusStopMarker2
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
                width: 220
                color: "#717171"
                text: viewController.previous_station_list[1][0]
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 20
                anchors.topMargin: 2
                anchors.top: parent.bottom
                wrapMode: Text.WordWrap
            }
            Text {
                id: beforeBusStopName2En
                width: 100
                color: "#717171"
                text: viewController.previous_station_list[1][1]
                anchors.top: beforeBusStopName2.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                font.pixelSize: 12
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
            }
            visible : (viewController.previous_station_list[1][0] || viewController.previous_station_list[1][0].length !== 0)
        }

        Shape {
            width: 30
            height: 44
            anchors.left: currentBusStopMarker.right
            anchors.leftMargin: 75
            anchors.verticalCenter: parent.verticalCenter
            ShapePath {
                fillColor: "white"
                fillRule: ShapePath.WindingFill
                strokeWidth: 0
                strokeColor: "white"
                startX: 0
                startY: 0
                PathLine {
                    x: 30
                    y: 22
                }
                PathLine {
                    x: 0
                    y: 44
                }
                PathLine {
                    x: 0
                    y: 0
                }
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