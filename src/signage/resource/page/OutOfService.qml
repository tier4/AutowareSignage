import QtQuick 2.9
import QtQuick.Controls 2.2

import "Common"

Rectangle {
    id: outOfServiceView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    CurrentTime {
        id: displayCurrentTime
    }

    Text {
        id: outOfServiceText
        color: "#000000"
        text: qsTr("回送中")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }

    Text {
        id: outOfServiceEnText
        color: "#000000"
        text: qsTr("Out Of Service")
        anchors.top: outOfServiceText.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 36*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
