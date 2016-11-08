import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Controls.Styles 1.4

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
        id: trackersModel
        ListElement {
            name: "Camshift"
            code: 1
        }
        ListElement {
            name: "MedianFlow"
            code: 2
        }
        ListElement {
            name: "Boosting"
            code: 3
        }
        ListElement {
            name: "Mil"
            code: 4
        }
        ListElement {
            name: "Tld"
            code: 5
        }
        ListElement {
            name: "CustomTld"
            code: 6
        }
        ListElement {
            name: "OpenTld"
            code: 7
        }
    }

    ExclusiveGroup { id: selectedTracker }

    Component {
        id: trackerDelegate
        Item {
            width: parent.width / 2
            height: 20
            Row {
                anchors.fill: parent
                spacing: 10
                Rectangle {
                    id: box
                    width: 20
                    height: 20

                    anchors.verticalCenter: parent.verticalCenter

                    property bool checked: presenter.trackerCode === code

                    border.color: "white"
                    Image {
                        anchors.fill: parent
                        source: "qrc:/icons/ok.svg"
                        visible: box.checked
                    }

                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            if (box.checked) return;
                            presenter.trackerCode = code
                        }
                    }
                }
                Text {
                    id: label
                    color: roboPalette.textColor
                    font.pixelSize: roboPalette.textSize
                    anchors.verticalCenter: parent.verticalCenter
                    text: name
                }
            }
        }
    }

    GroupBox {
        id: sliders
        title: "Video"
        width: parent.width

        anchors {
            top: label.bottom
            left: parent.left
            right: parent.right
            leftMargin: 5
            rightMargin: 5
            topMargin: 20
        }

        RowLayout {
            anchors.fill: parent

            spacing: 10

            Column {
                spacing: 10
                Text {
                    id: qualityLabel
                    color: roboPalette.textColor
                    font.pixelSize: roboPalette.textSize
                    text: "Quality"
                }
                Text {
                    id: brightnessLabel
                    color: roboPalette.textColor
                    font.pixelSize: roboPalette.textSize
                    text: "Brightness"
                }
                Text {
                    id: contrastLabel
                    color: roboPalette.textColor
                    font.pixelSize: roboPalette.textSize
                    text: "Contrast"
                }
            }
            Column {
                spacing: 10
                Layout.fillWidth: true
                Slider {
                    maximumValue: 100
                    width: parent.width
                    height: qualityLabel.height
                    stepSize: 1
                    value: presenter.quality
                    updateValueWhileDragging: false
                    onValueChanged: presenter.quality = value

                    Component.onCompleted: minimumValue = 1; // set value from presenter first
                }
                Slider {
                    maximumValue: 100
                    width: parent.width
                    height: brightnessLabel.height
                    stepSize: 1
                    value: presenter.brightness
                    updateValueWhileDragging: false
                    onValueChanged: presenter.brightness = value

                    Component.onCompleted: minimumValue = 1; // set value from presenter first
                }
                Slider {
                    maximumValue: 100
                    width: parent.width
                    height: contrastLabel.height
                    stepSize: 1
                    value: presenter.contrast
                    updateValueWhileDragging: false
                    onValueChanged: presenter.contrast = value

                    Component.onCompleted: minimumValue = 1; // set value from presenter first
                }
            }
            Column {
                spacing: 10
                Text {
                    color: roboPalette.textColor
                    font.pixelSize: roboPalette.textSize
                    text: "" + presenter.quality
                }
                Text {
                    color: roboPalette.textColor
                    font.pixelSize: roboPalette.textSize
                    text: "" + presenter.brightness
                }
                Text {
                    color: roboPalette.textColor
                    font.pixelSize: roboPalette.textSize
                    text: "" + presenter.contrast
                }
            }
        }
    }

    GroupBox {
        title: "Trackers"
        width: parent.width
        id: trackers

        anchors {
            top: sliders.bottom
            left: parent.left
            right: parent.right
            leftMargin: 5
            rightMargin: 5
            topMargin: 10
        }

        Grid {
            anchors.fill: parent

            columns: 2
            spacing: 10

            Repeater {
                model: trackersModel
                delegate: trackerDelegate
            }
        }
    }

    GroupBox {
        title: "Sensors calibrtion"
        width: parent.width

        anchors {
            top: trackers.bottom
            left: parent.left
            right: parent.right
            leftMargin: 5
            rightMargin: 5
            topMargin: 10
        }

        Grid {
            anchors.fill: parent

            spacing: 10

            Button {
                text: "Gun"
                onClicked: {
                    presenter.calibrateGun()
                }
            }
            Button {
                text: "Camera"
                onClicked: {
                    presenter.calibrateCamera()
                }
            }
            Button {
                text: "Gyro"
                onClicked: {
                    presenter.calibrateGyro()
                }
            }
        }
    }
}
