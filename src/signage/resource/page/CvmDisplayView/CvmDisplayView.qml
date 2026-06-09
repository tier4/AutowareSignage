import QtQuick 2.9
import QtQuick.Controls 2.2

Rectangle {
    id: cvmDisplayView
    width: viewController.monitor_width
    height: viewController.monitor_height

    CvmEmergencyStop {
        visible: viewController.cvm_display_mode_id === "remote_emergency_display"
    }
}
