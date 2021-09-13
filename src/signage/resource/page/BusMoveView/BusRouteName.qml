import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Shapes 1.12

Rectangle {
    id: busRouteName
    width: 1920
    height: 360
    color: "#ffffff"
    Text {
        id: busRouteText
        color: "#000000"
        text: getRouteName()
        anchors.verticalCenterOffset: -100
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 50
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        wrapMode: Text.WordWrap
    }
    Text {
        id: busRouteTextEn
        width: 213
        height: 39
        color: "#000000"
        text: getRouteNameEN()
        anchors.top: busRouteText.bottom
        anchors.topMargin: -5
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 36
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        visible: true
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
            width: 1700
            height: 44
            color: "#69bfd2"
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Rectangle {
            id: beforeBusStopMarker
            width: 60
            height: 60
            color: "#0068b6"
            radius: 30
            anchors.left: parent.left
            anchors.leftMargin: 300
            anchors.verticalCenter: beforeAndAfterBar.verticalCenter
            border.color: "#ffffff"
            border.width: 10

            Text {
                id: beforeBusStopName
                width: 350
                color: "#717171"
                text: viewController.departure_station_name[0]
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 30
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
            }
            Text {
                id: beforeBusStopNameEn
                width: 213
                color: "#717171"
                text: viewController.departure_station_name[1]
                anchors.top: beforeBusStopName.bottom
                anchors.topMargin: 5
                anchors.horizontalCenter: beforeBusStopName.horizontalCenter
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 16
                font.bold: true
                wrapMode: Text.WordWrap
            }
            visible : (viewController.departure_station_name[0] || viewController.departure_station_name[0].length !== 0)
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
                width: 350
                text: viewController.arrival_station_name[0]
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                anchors.topMargin: 2
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 30
                wrapMode: Text.WordWrap
            }
            Text {
                id: nextBusStopNameEn
                width: 213
                color: "#000000"
                text: viewController.arrival_station_name[1]
                anchors.horizontalCenter: nextBusStopName.horizontalCenter
                anchors.top: nextBusStopName.bottom
                anchors.topMargin: 5
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.bold: true
                font.pixelSize: 16
                wrapMode: Text.WordWrap
            }
            visible : (viewController.arrival_station_name[0] || viewController.arrival_station_name[0].length !== 0)
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
            text: viewController.remain_arrive_time_text
            anchors.top: beforeAndAfterBar.bottom
            anchors.topMargin: 10
            anchors.horizontalCenter: parent.horizontalCenter
            font.bold: true
            font.pixelSize: 48
            visible: viewController.display_time
            wrapMode: Text.WordWrap
        }
    }

    function getRouteName() {
        var routeName = viewController.route_name[0]
        if(!routeName || routeName.length === 0)
        {
            routeName = qsTr("バスルート名")
        }
        return routeName
    }

    function getRouteNameEN() {
        var routeName = viewController.route_name[1]
        if(!routeName || routeName.length === 0)
        {
            routeName = qsTr("Route Name")
        }
        return routeName
    }
}

/*##^## Designer {
    D{i:2;anchors_x:179;anchors_y:"-45"}D{i:6;anchors_x:179;anchors_y:82}D{i:7;anchors_x:179;anchors_y:82}
D{i:5;anchors_x:1507}D{i:9;anchors_y:89}D{i:10;anchors_x:179;anchors_y:82}
}
 ##^##*/