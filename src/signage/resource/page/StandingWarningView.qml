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
    property bool isDepart: viewController.standing_warning_type === "depart"

    // 急制動系(UC-03)の下部に表示する事象文言 (Figma node 207:43 / 207:33)
    //  複合(sudden_stop_turn)はデザイン未定のため両事象を併記
    function eventText() {
        switch (viewController.standing_warning_type) {
        case "sudden_stop":
            return qsTr("急停止します　Sudden Stop")
        case "sudden_turn":
            return qsTr("急カーブします　Sudden Turn")
        case "sudden_stop_turn":
            return qsTr("急停止・急カーブします")
        default:
            return ""
        }
    }

    CurrentTime {
        id: displayCurrentTime
    }

    // 発車時 (UC-02): 従来どおりの2段レイアウト
    //  長文のため小さめにし、時刻表示との重なりを避ける
    Column {
        visible: standingWarningView.isDepart
        anchors.centerIn: parent
        width: parent.width * 0.9
        spacing: 20 * viewController.size_ratio

        Text {
            width: parent.width
            color: "#000000"
            text: qsTr("発車します。\n手すり・つり革にしっかりとおつかまりください")
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 40 * viewController.size_ratio
            font.bold: true
            wrapMode: Text.WordWrap
        }

        Text {
            width: parent.width
            color: "#000000"
            text: qsTr("Bus moving. Please hold on")
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 30 * viewController.size_ratio
            font.bold: true
            elide: Text.ElideMiddle
        }
    }

    // 急制動系 (UC-03): 行動指示を主役にした3段レイアウト (Figma node 207:43 / 207:33)
    //  Figma の 120/88/54px を比率を保って pointSize 60/44/27 へ相対換算
    Column {
        visible: !standingWarningView.isDepart
        anchors.centerIn: parent
        spacing: 48 * viewController.size_ratio

        Column {
            anchors.horizontalCenter: parent.horizontalCenter
            spacing: 16 * viewController.size_ratio

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                color: "#000000"
                text: qsTr("手すりにおつかまりください")
                horizontalAlignment: Text.AlignHCenter
                font.pointSize: 60 * viewController.size_ratio
                font.bold: true
                font.letterSpacing: 6 * viewController.size_ratio
            }

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                color: "#000000"
                text: qsTr("Hold on tight")
                horizontalAlignment: Text.AlignHCenter
                font.pointSize: 44 * viewController.size_ratio
                font.bold: true
                font.letterSpacing: 4.4 * viewController.size_ratio
            }
        }

        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            color: "#000000"
            text: standingWarningView.eventText()
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 27 * viewController.size_ratio
            font.bold: true
            font.letterSpacing: 2.7 * viewController.size_ratio
        }
    }
}
