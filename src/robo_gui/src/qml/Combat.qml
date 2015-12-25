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
}
