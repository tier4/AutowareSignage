import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: emergencyStoppingView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"
    Text {
        id: emergencyStoppingText
        color: "#000000"
        text: qsTr("Stopping for safety check")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
