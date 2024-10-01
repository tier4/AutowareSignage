import QtQuick 2.9
import QtQuick.Controls 2.2

import "Common"

Rectangle {
    id: autoDrivingView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#92D050"

    CurrentTime {
        id: displayCurrentTime
        color: "#ffffff"
    }

    Text {
        id: autoDrivingText
        color: "#ffffff"
        text: qsTr("自動運転中")
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
        color: "#ffffff"
        text: qsTr("Auto Driving")
        anchors.top: autoDrivingText.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
