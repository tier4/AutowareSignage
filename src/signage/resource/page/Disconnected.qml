import QtQuick 2.9
import QtQuick.Controls 2.2

import "Common"

Rectangle {
    id: disconnectedView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    CurrentTime {
        id: displayCurrentTime
    }

    Text {
        id: disconnectedText1
        color: "#000000"
        text: qsTr("Communication with the autonomous system is delayed.")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        anchors.bottomMargin: 50*viewController.size_ratio
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 30*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
