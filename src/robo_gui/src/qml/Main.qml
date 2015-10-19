import QtQuick 2.0
import QtMultimedia 5.0

Rectangle {
    id: page
    width: 640;
    height: 480
    color: "black"

    property QtObject framePresenter: factory.framePresenter()
    property QtObject trackPresenter: factory.trackPresenter()
    property double scaleX: video.sourceRect.width / page.width
    property double scaleY: video.sourceRect.height / page.height

    VideoOutput {
        id: video
        anchors.fill: parent;
        source: framePresenter;
    }

    Target {
        id: target
        Connections {
            target: trackPresenter
            onTargetRectChanged: {
                target.x = rect.x / page.scaleX
                target.y = rect.y / page.scaleY
                target.width = rect.width / page.scaleX
                target.height = rect.height / page.scaleY
            }
        }
    }

    SelectArea {
        anchors.fill: parent
        onAccepted: {
            r.x = r.x * page.scaleX
            r.y = r.y * page.scaleY
            r.width = r.width * page.scaleX
            r.height = r.height * page.scaleY

            trackPresenter.onTrackRequest(r)
        }
    }
}
