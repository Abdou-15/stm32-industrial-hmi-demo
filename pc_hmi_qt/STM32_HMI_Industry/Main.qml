import QtQuick
import QtQuick.Controls
import STM32Backend 1.0    // SerialManger

ApplicationWindow {
    id: root
    visible: true
    width: 800
    height: 480
    title: qsTr("STM32 Industrial HMI")

    SerialManger {
        id: serial

        onErrorOccurred: (message) => {
            console.log("Serial error:", message)
        }

        onMachineStateChanged: {
            console.log("New frame:",
                        "Tin", tempInlet,
                        "Tout", tempOutlet,
                        "P", pressureBar,
                        "RPM", motorRpm)
        }
    }

    Column {
        anchors.centerIn: parent
        spacing: 16

        Text {
            text: qsTr("STM32 Process Dashboard")
            font.pixelSize: 28
            font.bold: true
        }

        // Connection status
        Text {
            text: serial.connected ? "Status: Connected" : "Status: Disconnected"
            color: serial.connected ? "green" : "red"
            font.pixelSize: 16
        }

        Row {
            spacing: 12
            anchors.horizontalCenter: parent.horizontalCenter

            Button {
                text: qsTr("Connect")
                enabled: !serial.connected
                onClicked: {
                    // TODO: put your actual macOS device path here:
                    // e.g. check in /dev: ls /dev/cu.usbmodem*
                    serial.openPort("/dev/cu.usbmodem3154325033311")
                }
            }

            Button {
                text: qsTr("Disconnect")
                enabled: serial.connected
                onClicked: serial.closePort()
            }
        }

        Rectangle {
            width: 450
            height: 220
            radius: 12
            border.width: 1
            border.color: "#888888"

            Column {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 8

                Text {
                    text: qsTr("Process Values")
                    font.pixelSize: 20
                    font.bold: true
                }

                Row {
                    spacing: 40

                    Column {
                        spacing: 4
                        Text { text: qsTr("Temp Inlet [°C]"); font.pixelSize: 14 }
                        Text {
                            text: serial.connected
                                  ? serial.tempInlet.toFixed(1)
                                  : "--"
                            font.pixelSize: 24
                            font.bold: true
                        }

                        Text { text: qsTr("Temp Outlet [°C]"); font.pixelSize: 14 }
                        Text {
                            text: serial.connected
                                  ? serial.tempOutlet.toFixed(1)
                                  : "--"
                            font.pixelSize: 24
                            font.bold: true
                        }
                    }

                    Column {
                        spacing: 4
                        Text { text: qsTr("Pressure [bar]"); font.pixelSize: 14 }
                        Text {
                            text: serial.connected
                                  ? serial.pressureBar.toFixed(2)
                                  : "--"
                            font.pixelSize: 24
                            font.bold: true
                        }

                        Text { text: qsTr("Motor Speed [rpm]"); font.pixelSize: 14 }
                        Text {
                            text: serial.connected
                                  ? serial.motorRpm.toFixed(0)
                                  : "--"
                            font.pixelSize: 24
                            font.bold: true
                        }
                    }
                }
            }
        }

        Rectangle {
            width: 450
            height: 150
            radius: 12
            border.width: 1
            border.color: "#888888"

            Column {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 8

                Text {
                    text: qsTr("Status Flags")
                    font.pixelSize: 20
                    font.bold: true
                }

                Row {
                    spacing: 24

                    Rectangle {
                        width: 80
                        height: 40
                        radius: 8
                        color: serial.doorClosed ? "green" : "red"

                        Text {
                            anchors.centerIn: parent
                            text: qsTr("Door")
                            color: "white"
                            font.pixelSize: 14
                        }
                    }

                    Rectangle {
                        width: 80
                        height: 40
                        radius: 8
                        color: serial.estopActive ? "red" : "#444444"

                        Text {
                            anchors.centerIn: parent
                            text: qsTr("E-Stop")
                            color: "white"
                            font.pixelSize: 14
                        }
                    }

                    Rectangle {
                        width: 80
                        height: 40
                        radius: 8
                        color: serial.alarmActive ? "red" : "#444444"

                        Text {
                            anchors.centerIn: parent
                            text: qsTr("Alarm")
                            color: "white"
                            font.pixelSize: 14
                        }
                    }
                }

                Row {
                    spacing: 16
                    Text {
                        text: qsTr("Alarm Code: ") +
                              (serial.connected ? serial.alarmCode : 0)
                        font.pixelSize: 14
                    }
                    Text {
                        text: qsTr("Uptime [ms]: ") +
                              (serial.connected ? serial.uptimeMs : 0)
                        font.pixelSize: 14
                    }
                }
            }
        }

        Button {
            text: qsTr("Quit")
            onClicked: Qt.quit()
            anchors.horizontalCenter: parent.horizontalCenter
        }
    }
}
