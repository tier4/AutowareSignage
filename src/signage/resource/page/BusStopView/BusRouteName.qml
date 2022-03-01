import QtQuick 2.9
import QtQuick.Controls 2.2

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

    Item {
        id: currentBusStop
        anchors.verticalCenter: centerBar.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter

        Rectangle {
            id: currentBusStopMarker
            width: 70
            height: 70
            color: "#ffffff"
            radius: 35
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            border.color: "#717171"
            border.width: 5

            Rectangle {
                id: innerLayer
                width: 50
                height: 50
                color: "#0068b6"
                radius: 25
                border.width: 0
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Text {
            id: busStopName
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
            id: busStopNameEn
            width: getTextWidth()
            color: "#000000"
            text: viewController.departure_station_name[1]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: busStopName.bottom
            anchors.topMargin: 5
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 30*viewController.font_ratio
            font.bold: true
            elide: Text.ElideMiddle
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
        anchors.verticalCenter: centerBar.verticalCenter
        border.color: "#717171"
        border.width: 10

        Text {
            id: beforeBusStopName
            width: getTextWidth()
            color: "#717171"
            text: viewController.previous_station_name[0]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: beforeBusStopMarker.bottom
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 40*viewController.font_ratio
            font.bold: true
            elide: Text.ElideMiddle
        }
        Text {
            id: beforeBusStopNameEn
            width: getTextWidth()
            color: "#717171"
            text: viewController.previous_station_name[1]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: beforeBusStopName.bottom
            anchors.topMargin: 5
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 30*viewController.font_ratio
            font.bold: true
            elide: Text.ElideMiddle
        }
        visible : (viewController.previous_station_name[0] || viewController.previous_station_name[0].length !== 0)
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

    function getTextWidth() {
        return viewController.monitor_width / 3 - 100
    }
}
