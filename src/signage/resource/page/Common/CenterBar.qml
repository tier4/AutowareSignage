import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Shapes 1.12

Rectangle {
    id: centerBar
    width: viewController.monitor_width - 200
    height: 44*viewController.size_ratio
    color: "#69bfd2"
}
