import QtQuick 2.9
import QtQuick.Controls 2.2

import "Common"

Rectangle {
    id: standingWarningView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    // 警告種別ごとの表示文言
    //  depart           : 発車時 (UC-02)
    //  sudden_stop      : 急減速 (UC-03)
    //  sudden_turn      : 急操舵 (UC-03)
    //  sudden_stop_turn : 急減速かつ急操舵 (UC-03)
    function mainText() {
        switch (viewController.standing_warning_type) {
        case "depart":
            return qsTr("発車します。手すり・つり革にしっかりとおつかまりください")
        case "sudden_stop":
            return qsTr("急停車します！")
        case "sudden_turn":
            return qsTr("急旋回します！")
        case "sudden_stop_turn":
            return qsTr("急停車・急旋回します！")
        default:
            return ""
        }
    }

    function subText() {
        switch (viewController.standing_warning_type) {
        case "depart":
            return qsTr("Bus moving. Please hold on")
        default:
            return qsTr("Sudden stops and turns.")
        }
    }

    // 発進メッセージは長文のため小さめにし、時刻表示との重なりを避ける
    function mainFontSize() {
        return viewController.standing_warning_type === "depart" ? 40 : 60
    }

    function subFontSize() {
        return viewController.standing_warning_type === "depart" ? 30 : 50
    }

    CurrentTime {
        id: displayCurrentTime
    }

    Text {
        id: standingWarningText
        color: "#000000"
        text: standingWarningView.mainText()
        width: parent.width * 0.9
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: standingWarningView.mainFontSize()*viewController.size_ratio
        font.bold: true
        wrapMode: Text.WordWrap
    }

    Text {
        id: standingWarningSubText
        color: "#000000"
        text: standingWarningView.subText()
        anchors.top: standingWarningText.bottom
        anchors.topMargin: 20*viewController.size_ratio
        anchors.horizontalCenter: parent.horizontalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: standingWarningView.subFontSize()*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
