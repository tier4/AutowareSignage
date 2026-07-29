import QtQuick 2.9
import QtQuick.Controls 2.2

import "Common"

// UC-05 (SYS-HMI-07): バス停到着時に「‹停留所名›に到着しました」を
// announce_interval.arrived 秒間 (既定 5s) 専用画面で表示する。
// BusStopView のローテーション (10s 周期) では到着直後に表示が回ってこないため、
// 到着表示は独立した view_mode として最優先で描画する。
Rectangle {
    id: arrivedView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    CurrentTime {
        id: displayCurrentTime
    }

    Text {
        id: arrivedText
        width: viewController.monitor_width - 200
        height: viewController.monitor_height
        color: "#000000"
        text: viewController.display_phrase
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio*0.7
        font.bold: true
        elide: Text.ElideMiddle
        wrapMode: Text.WordWrap
    }
}
