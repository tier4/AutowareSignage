import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Shapes 1.12

import "../Common"

Rectangle {
    id: busRouteName
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    CenterBar {
        id: centerBar
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
    }

    RouteName {
        id: headerRouteName
        anchors.bottom: centerBar.top
        anchors.horizontalCenter: parent.horizontalCenter
    }

    Rectangle {
        id: currentBusStopMarker
        width: 60
        height: 60
        color: "#0068b6"
        radius: 30
        anchors.left: parent.left
        anchors.leftMargin: 300
        anchors.verticalCenter: centerBar.verticalCenter
        border.color: "#ffffff"
        border.width: 10

        Text {
            id: currentBusStopName
            width: getTextWidth()
            color: "#000000"
            text: viewController.departure_station_name[0]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: currentBusStopMarker.bottom
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 40*viewController.font_ratio
            font.bold: true
            elide: Text.ElideMiddle
        }
        Text {
            id: currentBusStopNameEn
            width: getTextWidth()
            color: "#000000"
            text: viewController.departure_station_name[1]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: currentBusStopName.bottom
            anchors.topMargin: 5
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 30*viewController.font_ratio
            font.bold: true
            elide: Text.ElideMiddle
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
        anchors.verticalCenter: centerBar.verticalCenter
        border.color: "#0068b6"
        border.width: 10

        Text {
            id: nextBusStopName
            width: getTextWidth()
            color: "#000000"
            text: viewController.arrival_station_name[0]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: nextBusStopMarker.bottom
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 40*viewController.font_ratio
            font.bold: true
            elide: Text.ElideMiddle
        }
        Text {
            id: nextBusStopNameEn
            width: getTextWidth()
            color: "#000000"
            text: viewController.arrival_station_name[1]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: nextBusStopName.bottom
            anchors.topMargin: 5
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 30*viewController.font_ratio
            font.bold: true
            elide: Text.ElideMiddle
        }
        visible : (viewController.arrival_station_name || viewController.arrival_station_name.length !== 0)
    }

    Item {
        id: arrow_direction
        anchors.verticalCenter: centerBar.verticalCenter
        anchors.horizontalCenter: centerBar.horizontalCenter

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
        anchors.top: centerBar.bottom
        anchors.topMargin: 10
        anchors.horizontalCenter: parent.horizontalCenter
        font.bold: true
        font.pixelSize: 48*viewController.font_ratio
        visible: viewController.display_time
    }

    function getTextWidth() {
        return viewController.monitor_width / 3 - 100
    }
}
