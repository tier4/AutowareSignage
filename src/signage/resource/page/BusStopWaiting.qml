import QtQuick 2.9
import QtQuick.Controls 2.2

import "Common"

Rectangle {
    id: busStopWaitingView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    CurrentTime {
        id: displayCurrentTime
    }

    Text {
        id: busStopWaitingText
        color: "#000000"
        text: qsTr("乗降中")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }

    Text {
        id: busStopWaitingEnText
        color: "#000000"
        text: qsTr("Boarding & Exiting")
        anchors.top: busStopWaitingText.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
