import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: cvmEmergencyStop
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffff00"

    Text {
        id: cvmEmergencyStopText
        color: "#000000"
        text: qsTr("緊急停止中")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 100*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }

    Text {
        id: cvmEmergencyStopEnText
        color: "#000000"
        text: qsTr("Emergency Stop")
        anchors.top: cvmEmergencyStopText.bottom
        anchors.topMargin: 20*viewController.size_ratio
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 70*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
