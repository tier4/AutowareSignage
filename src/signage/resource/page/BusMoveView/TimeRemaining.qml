import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Shapes 1.12

Rectangle {
    id: remainTimeText
    width: 1920
    height: 360
    color: "#ffffff"
    Text {
        id: busRouteText
        x: 288
        y: 207
        color: "#000000"
        text: viewController.route_name
        anchors.verticalCenterOffset: -100
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 68
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
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
                text: viewController.previous_station_list[0]
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 48
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
                width: 294
                height: 49
                text: viewController.departure_station_name
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 48
            }
        }

        Item {
            id: element
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            Shape {
                id: middleTriangle
                width: 30
                height: 44
                anchors.horizontalCenter: parent.horizontalCenter
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

            Shape {
                id: leftTriangle
                width: 30
                height: 44
                anchors.right: middleTriangle.left
                anchors.rightMargin: 0
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

            Shape {
                id: rightTriangle
                x: 15
                y: -68
                width: 30
                height: 44
                anchors.left: middleTriangle.right
                anchors.leftMargin: 0
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

        Text {
            id: remainingTime
            x: 912
            text: viewController.remain_time_text
            anchors.top: beforeAndAfterBar.bottom
            anchors.topMargin: 10
            anchors.horizontalCenter: parent.horizontalCenter
            font.bold: true
            font.pixelSize: 48
        }
    }
}

/*##^## Designer {
    D{i:2;anchors_x:179;anchors_y:"-45"}D{i:6;anchors_x:179;anchors_y:82}D{i:7;anchors_x:179;anchors_y:82}
D{i:5;anchors_x:1507}D{i:9;anchors_y:62}D{i:10;anchors_x:179;anchors_y:82}D{i:27;anchors_y:106}
}
 ##^##*/

