import QtQuick 2.0
import QtQuick.Controls 1.4

Item {
    id: root
    anchors.fill: parent
    anchors.topMargin: 20

    property color textColor: "white"
    property int h1: 32
    property int h2: 16

    Text {
        id: label
        anchors.horizontalCenter: parent.horizontalCenter
        text: "Settings"
        color: root.textColor
        font.pixelSize: root.h1
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
                color: root.textColor
                font.pixelSize: root.h2
                text: item.text
            }
            Button {
                id: button
                visible: even
                text: "Select"
                width: 100
                checkable: true
                exclusiveGroup: selectedTracker
                onClicked: {
                    console.log(item.text, item.code)
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
            color: root.textColor
            font.pixelSize: root.h2
            text: "Video quality"
        }

        Slider {
            minimumValue: 1
            maximumValue: 100
            width: 300
            stepSize: 1
            updateValueWhileDragging: false

            onValueChanged: {
                presenter.setQuality(value)
            }
        }
        Repeater {
            model: trackers.count * 2
            delegate: trackerDelegate
        }
    }
}
