import QtQuick 2.9
import QtQuick.Controls 2.2

Text {
    id: outOfServiceText
    color: "#000000"
    text: viewController.clock_string
    anchors.top: parent.top
    anchors.topMargin: 10*viewController.size_ratio
    anchors.left: parent.left
    anchors.leftMargin: 10*viewController.size_ratio
    horizontalAlignment: Text.AlignHCenter
    verticalAlignment: Text.AlignVCenter
    font.pointSize: 40*viewController.size_ratio
    font.bold: true
    elide: Text.ElideLeft
}
