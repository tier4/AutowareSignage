import QtQuick 2.9
import QtQuick.Controls 2.2

import "Common"

Rectangle {
    id: slowStopView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffff00"

    CurrentTime {
        id: displayCurrentTime
    }

    Text {
        id: slowStopText
        color: "#000000"
        text: qsTr("減速します。ご注意ください")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }

    Text {
        id: autoDrivingEnText
        color: "#000000"
        text: qsTr("Slowing down. Please be careful")
        anchors.top: autoDrivingText.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
