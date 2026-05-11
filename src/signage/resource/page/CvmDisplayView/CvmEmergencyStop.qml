import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: cvmEmergencyStop
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ff8c00"

    Text {
        id: cvmEmergencyStopIcon
        color: "#ffffff"
        text: "⚠"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: cvmEmergencyStopText.top
        anchors.bottomMargin: 20*viewController.size_ratio
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 120*viewController.size_ratio
        font.bold: true
    }

    Text {
        id: cvmEmergencyStopText
        color: "#ffffff"
        text: qsTr("オペレーター停止指示")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }

    Text {
        id: cvmEmergencyStopEnText
        color: "#ffffff"
        text: qsTr("Stopped by Operator")
        anchors.top: cvmEmergencyStopText.bottom
        anchors.topMargin: 20*viewController.size_ratio
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 60*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
