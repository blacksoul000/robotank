import QtQuick 2.0
import QtMultimedia 5.0

Rectangle {
    id: page
    width: 640; height: 480
    color: "lightgray"

    property QtObject framePresenter: factory.framePresenter()

    VideoOutput {
        anchors.fill: parent;
        source: framePresenter;
    }
}
