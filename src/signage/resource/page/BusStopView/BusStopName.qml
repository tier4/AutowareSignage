import QtQuick 2.9
import QtQuick.Controls 2.2

import "../Common"

Rectangle {
    id: busStopName
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    CurrentTime {
        id: displayCurrentTime
    }

    CenterBar {
        id: centerBar
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
    }

    Item {
        id: currentBusStop
        anchors.verticalCenter: centerBar.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter

        Rectangle {
            id: currentBusStopMarker
            width: 90*viewController.size_ratio
            height: 90*viewController.size_ratio
            color: "#ffffff"
            radius: 45*viewController.size_ratio
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            border.color: "#717171"
            border.width: 5*viewController.size_ratio

            Rectangle {
                id: innerLayer
                width: 70*viewController.size_ratio
                height: 70*viewController.size_ratio
                color: "#0068b6"
                radius: 35*viewController.size_ratio
                border.width: 0
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Text {
            id: currentBusStopName
            width: viewController.monitor_width - 200
            color: "#000000"
            text: viewController.departure_station_name[0]
            anchors.bottom: currentBusStopNameEn.top
            anchors.horizontalCenter: parent.horizontalCenter
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 50*viewController.size_ratio
            font.bold: true
            elide: Text.ElideMiddle
            wrapMode: Text.WordWrap
        }

        Text {
            id: currentBusStopNameEn
            width: viewController.monitor_width - 200
            color: "#000000"
            text: viewController.departure_station_name[1]
            anchors.bottom: currentBusStopMarker.top
            anchors.bottomMargin: 10*viewController.size_ratio
            anchors.horizontalCenter: parent.horizontalCenter
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 30*viewController.size_ratio
            font.bold: true
            elide: Text.ElideMiddle
            wrapMode: Text.WordWrap
        }
    }

    Rectangle {
        id: beforeBusStopMarker
        width: 60*viewController.size_ratio
        height: 60*viewController.size_ratio
        color: "#ffffff"
        radius: 30*viewController.size_ratio
        anchors.left: parent.left
        anchors.leftMargin: getMargin()
        anchors.verticalCenter: centerBar.verticalCenter
        border.color: "#717171"
        border.width: 10*viewController.size_ratio

        Text {
            id: beforeBusStopName
            width: getTextWidth()
            color: "#717171"
            text: viewController.previous_station_name[0]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: beforeBusStopMarker.bottom
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 40*viewController.size_ratio
            font.bold: true
            elide: Text.ElideMiddle
            wrapMode: Text.WordWrap
        }
        Text {
            id: beforeBusStopNameEn
            width: getTextWidth()
            color: "#717171"
            text: viewController.previous_station_name[1]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: beforeBusStopName.bottom
            anchors.topMargin: 5*viewController.size_ratio
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 30*viewController.size_ratio
            font.bold: true
            elide: Text.ElideMiddle
            wrapMode: Text.WordWrap
        }
        visible : checkVisible(viewController.previous_station_name[0])
    }

    Rectangle {
        id: nextBusStopMarker
        width: 60*viewController.size_ratio
        height: 60*viewController.size_ratio
        color: "#ffffff"
        radius: 30*viewController.size_ratio
        anchors.right: parent.right
        anchors.rightMargin: getMargin()
        anchors.verticalCenter: centerBar.verticalCenter
        border.color: "#0068b6"
        border.width: 10*viewController.size_ratio

        Text {
            id: nextBusStopName
            width: getTextWidth()
            color: "#000000"
            text: viewController.arrival_station_name[0]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: nextBusStopMarker.bottom
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 40*viewController.size_ratio
            font.bold: true
            elide: Text.ElideMiddle
            wrapMode: Text.WordWrap
        }
        Text {
            id: nextBusStopNameEn
            width: getTextWidth()
            color: "#000000"
            text: viewController.arrival_station_name[1]
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: nextBusStopName.bottom
            anchors.topMargin: 5*viewController.size_ratio
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 30*viewController.size_ratio
            font.bold: true
            elide: Text.ElideMiddle
            wrapMode: Text.WordWrap
        }
        visible : checkVisible(viewController.arrival_station_name[0])
    }

    function getTextWidth() {
        return viewController.monitor_width / 3 - (80*viewController.size_ratio)
    }

    function getMargin() {
        return viewController.monitor_width / 4
    }

    function checkVisible(name) {
        return (name || name.length !== 0)
    }
}
