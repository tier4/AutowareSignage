
import QtQuick 2.9
import QtMultimedia 5.8

Item {

    Audio {
        id: engageAnnounce
        source: "../sound/engage.mp3"
    }

    Audio {
        id: arriveAnnounce
        source: "../sound/arrived.mp3"
    }

    Audio {
        id: emergencyAnnounce
        source: "../sound/emergency.mp3"
    }

    Audio {
        id: preArriveAnnounce
        source: "../sound/going_to_arrive.mp3"
    }

    Audio {
        id: preDepartAnnounce
        source: "../sound/going_to_depart.mp3"
    }

    Audio {
        id: emergencyRemovedAnnounce
        source: "../sound/emergency_cancel.mp3"
    }

    Audio {
        id: inEmergencyRemovedAnnounce
        source: "../sound/in_emergency.mp3"
    }

    Component.onCompleted: {
        // announce
        announceController._announce_signal.connect(announceExecutor)
    }

    function announceExecutor(type) {
        if (type === "engage") {
            engageAnnounce.play()
        } else if (type === "arrived") {
            arriveAnnounce.play()
        } else if (type === "emergency") {
            emergencyAnnounce.play()
        } else if (type === "going_to_arrive") {
            preArriveAnnounce.play()
        } else if (type === "going_to_depart") {
            preDepartAnnounce.play()
        } else if (type === "emergency_cancel") {
            emergencyRemovedAnnounce.play()
        } else if (type === "in_emergency") {
            inEmergencyRemovedAnnounce.play()
        }
    } 
}

/*##^## Designer {
    D{i:0;height:360;width:1920}
}
 ##^##*/