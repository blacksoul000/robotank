import QtQuick 2.0
import QtMultimedia 5.0

Rectangle {
    id: page
    width: 640; height: 480
    color: "green"

    property QtObject framePresenter: factory.framePresenter()
    property QtObject trackPresenter: factory.trackPresenter()

    VideoOutput {
        anchors.fill: parent;
        source: framePresenter;
    }

    Target {
        id: target
        Connections {
            target: trackPresenter
            onTargetRectChanged: {
                target.x = rect.x
                target.y = rect.y
                target.width = rect.width
                target.height = rect.height
            }
        }
    }

    SelectArea {
        anchors.fill: parent

        onAccepted: trackPresenter.onTrackRequest(r)
    }
}
