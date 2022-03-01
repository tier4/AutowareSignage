import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: emergencyStopView
    width: 1920
    height: viewController.monitor_height
    color: "#ffffff"
    Text {
        id: emergencyStopText
        x: 288
        y: 207
        color: "#000000"
        text: qsTr("回送中")
        anchors.verticalCenterOffset: -40
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 80*viewController.size_ratio
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
    }

    Text {
        id: element
        x: 651
        width: 568
        height: 42
        color: "#000000"
        text: qsTr("Out Of Service")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: emergencyStopText.bottom
        anchors.topMargin: 10
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 36*viewController.size_ratio
    }
}

/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
