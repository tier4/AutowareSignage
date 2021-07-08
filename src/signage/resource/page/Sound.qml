
import QtQuick 2.9
import QtMultimedia 5.8

Item {

    Audio {
        id: engageAnnounce
        source: "../sound/engage.wav"
    }

    Audio {
        id: arriveAnnounce
        source: "../sound/arrived.wav"
    }

    Audio {
        id: emergencyAnnounce
        source: "../sound/emergency.wav"
    }

    Audio {
        id: preArriveAnnounce
        source: "../sound/going_to_arrive.wav"
    }

    Audio {
        id: preDepartAnnounce
        source: "../sound/going_to_depart.wav"
    }

    Audio {
        id: emergencyRemovedAnnounce
        source: "../sound/emergency_cancel.wav"
    }

    Audio {
        id: inEmergencyRemovedAnnounce
        source: "../sound/in_emergency.wav"
    }

    Component.onCompleted: {
        // announce
        announceController._announce_signal.connect(announceExecutor)
    }

    function announceExecutor(type) {
        engageAnnounce.stop()
        arriveAnnounce.stop()
        emergencyAnnounce.stop()
        preArriveAnnounce.stop()
        preDepartAnnounce.stop()
        emergencyRemovedAnnounce.stop()
        inEmergencyRemovedAnnounce.stop()
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