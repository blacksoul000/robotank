import QtQuick 2.0
import QtMultimedia 5.0
import QtQuick.Controls 1.4

Item {
    id: page

    property QtObject framePresenter: factory.framePresenter()
    property QtObject trackPresenter: factory.trackPresenter()
    property double scaleX: video.sourceRect.width / page.width
    property double scaleY: video.sourceRect.height / page.height

    VideoOutput {
        id: video
        anchors.fill: parent;
        source: framePresenter;
    }

    Crosshair {
        anchors.centerIn: parent
        visible: !target.valid
        width: 150
        height: 150
    }

    CaptureArea {
        id: capture
        anchors.centerIn: parent
        visible: !target.valid && framePresenter.hasFrame
        presenter: trackPresenter

        Connections {
            target: trackPresenter
            onCaptureSizeChanged: {
                trackPresenter.setCaptureFrameRect(screenToImage(capture.r))
            }
        }
    }

    Target {
        id: target
        x: trackPresenter.targetRect.x / page.scaleX
        y: trackPresenter.targetRect.y / page.scaleY
        width: trackPresenter.targetRect.width / page.scaleX
        height: trackPresenter.targetRect.height / page.scaleY
    }

    SelectArea {
        anchors.fill: parent
        visible: framePresenter.hasFrame
        onAccepted: {
            trackPresenter.onTrackRequest(screenToImage(r))
        }
    }

    Loader {
        property int hidden: parent.x + parent.width
        property int showed: parent.x + 2 * parent.width / 3
        property bool isLoaded: false
        property bool isHidden: true

        id: settingsLoader
        x: hidden
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: parent.width / 3

        PropertyAnimation {
            id: settingsShow
            target: settingsLoader
            property: "x"
            to: settingsLoader.showed
            duration: 200
        }
        PropertyAnimation {
            id: settingsHide
            target: settingsLoader
            property: "x"
            to: settingsLoader.hidden
            duration: 200
        }
    }

    RButton {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 5

        imageSource: "qrc:/icons/settings.svg"
        onClicked: {
            if (!settingsLoader.isLoaded)
            {
                settingsLoader.isLoaded = true
                settingsLoader.source = "qrc:/qml/Settings.qml"
            }
            if (settingsLoader.isHidden)
            {
                settingsLoader.isHidden = false
                settingsShow.start()
            }
            else
            {
                settingsLoader.isHidden = true
                settingsHide.start()
            }
        }
    }

    function screenToImage(rect) {
        rect.x *= page.scaleX
        rect.y *= page.scaleY
        rect.width *= page.scaleX
        rect.height *= page.scaleY
        return rect
    }
}
