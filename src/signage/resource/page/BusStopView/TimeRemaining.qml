import QtQuick 2.9
import QtQuick.Controls 2.2

import "../Common"

Rectangle {
    id: remainingTimeView
    width: viewController.monitor_width
    height: viewController.monitor_height
    color: "#ffffff"

    CurrentTime {
        id: displayCurrentTime
    }

    CompanyIcon {
        id: companyIcon
    }

    Text {
        id: remainingTimeText
        width: viewController.monitor_width - 200
        height: viewController.monitor_height
        color: "#000000"
        text: viewController.remain_depart_time_text
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.pointSize: 80*viewController.size_ratio
        font.bold: true
        elide: Text.ElideMiddle
    }
}
