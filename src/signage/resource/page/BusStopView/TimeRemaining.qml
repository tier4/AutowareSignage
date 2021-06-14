import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: emergencyStopView
    width: 1920
    height: 360
    color: "#ffffff"
    Text {
        id: remainingTimeText
        x: 295
        y: 216
        width: 211
        height: 156
        color: "#000000"
        text: viewController.remain_time_text
        anchors.verticalCenterOffset: -40
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        font.bold: true
        font.pointSize: 80
        verticalAlignment: Text.AlignVCenter
    }

    Text {
        id: emergencyStopText2
        x: 645
        color: "#000000"
        text: qsTr("This bus will be leaving in ") + viewController.remain_time + qsTr(" minutes.")
        anchors.top: remainingTimeText.bottom
        anchors.topMargin: 20
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        font.bold: true
        font.pointSize: 32
        verticalAlignment: Text.AlignVCenter
        visible: false
    }
}

/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/

