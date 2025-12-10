import QtQuick 2.9
import QtQuick.Controls 2.2

Text {
    id: autonomousDrivingText
    color: "#69bfd2"
    text: "自動運転中"
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.leftMargin: 16
    horizontalAlignment: Text.AlignHCenter
    verticalAlignment: Text.AlignVCenter
    font.pointSize: 40*viewController.size_ratio
    font.bold: true
    elide: Text.ElideLeft
}
