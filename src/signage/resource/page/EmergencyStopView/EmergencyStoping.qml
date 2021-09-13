import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: emergencyStopView
    width: 1920
    height: 360
    color: "#f60000"
    Text {
        id: emergencyStopText
        x: 288
        y: 207
        color: "#ffffff"
        text: qsTr("只今オペレータ対応中です")
        anchors.verticalCenterOffset: -40
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 80
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
    }

    Text {
        id: element
        x: 651
        width: 568
        height: 42
        color: "#ffffff"
        text: qsTr("Now the operator is working on it")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: emergencyStopText.bottom
        anchors.topMargin: 10
        font.bold: true
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: 36
    }
}

/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/

