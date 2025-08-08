import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: middleDoorOpenView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    Text {
        id: displayCurrentTime
        color: "#000000"
        text: viewController.clock_string
        anchors.top: parent.top
        anchors.left: parent.left
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 40*viewController.size_ratio
        font.bold: true
        elide: Text.ElideLeft
    }

    Text {
        id: middleDoorOpenText
        color: "#000000"
        text: qsTr("乗車中")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }

    Text {
        id: middleDoorOpenEnText
        color: "#000000"
        text: qsTr("Boarding")
        anchors.top: middleDoorOpenText.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
} 