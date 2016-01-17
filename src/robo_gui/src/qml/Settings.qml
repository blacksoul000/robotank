import QtQuick 2.0
import QtQuick.Controls 1.4

Item {
    id: root

    Text {
        id: label
        anchors.horizontalCenter: parent.horizontalCenter
        text: "Settings"
        color: roboPalette.textColor
        font.pixelSize: roboPalette.captionTextSize
    }

    property QtObject presenter: factory.settingsPresenter()

    ListModel {
        id: trackers
        ListElement {
            text: "Camshift"
            code: 1
        }
        ListElement {
            text: "MedianFlow"
            code: 2
        }
        ListElement {
            text: "Boosting"
            code: 3
        }
        ListElement {
            text: "Mil"
            code: 4
        }
        ListElement {
            text: "Tld"
            code: 5
        }
        ListElement {
            text: "CustomTld"
            code: 6
        }
        ListElement {
            text: "OpenTld"
            code: 7
        }
    }

    ExclusiveGroup { id: selectedTracker }

    Component {
        id: trackerDelegate
        Item {
            property bool even: (index % 2)
            property var item: trackers.get(index / 2)

            width: even ? label.width : button.width
            height: even ? label.height : button.height
            Text {
                id: label
                visible: !even
                color: roboPalette.textColor
                font.pixelSize: roboPalette.textSize
                anchors.verticalCenter: parent.verticalCenter
                text: item.text
            }
            RButton {
                id: button
                visible: even
                imageSource: checked ? "qrc:/icons/ok.svg" : "qrc:/icons/cancel.svg"
                checkable: true
                exclusiveGroup: selectedTracker

                onClicked: {
                    presenter.trackerCode = item.code
                }

                Component.onCompleted: {
                    presenter.trackerCodeChanged.connect(onPresenterTracerCodeChanged)
                    onPresenterTracerCodeChanged(presenter.trackerCode)
                }

                function onPresenterTracerCodeChanged(trackerCode) {
                    if (trackerCode === item.code) selectedTracker.current = button
                }
            }
        }
    }

    Grid {
        id: grid
        anchors.top: label.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 20
        columns: 2
        spacing: 10
        Text {
            id: txt
            color: roboPalette.textColor
            font.pixelSize: roboPalette.textSize
            text: "Video quality"
        }

        Slider {
            maximumValue: 100
            width: 200
            height: txt.height
            stepSize: 1
            value: presenter.quality
            updateValueWhileDragging: false
            onValueChanged: presenter.quality = value

            Component.onCompleted: minimumValue = 1; // set value from presenter first
        }
        Repeater {
            model: trackers.count * 2
            delegate: trackerDelegate
        }
    }
}
