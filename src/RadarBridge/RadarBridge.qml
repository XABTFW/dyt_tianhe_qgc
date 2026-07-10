/****************************************************************************
 *
 * Radar Bridge control / monitor window.
 *
 * A small pop-up Window that drives the RadarBridgeManager C++ singleton.
 * It is loaded lazily from MainRootWindow and shown from a tool-strip button
 * placed underneath the swarm button.
 *
 * All controls read/write RadarBridgeManager's Q_PROPERTYs and call its
 * Q_INVOKABLE methods, so the whole feature is driven from here.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

import QGroundControl
import QGroundControl.Controls
import QGroundControl.ScreenTools
import QGroundControl.Palette
import QGroundControl.RadarBridge   // RadarBridgeManager singleton

Window {
    id:             root
    title:          qsTr("Radar Bridge")
    width:          ScreenTools.defaultFontPixelWidth * 90
    height:         ScreenTools.defaultFontPixelHeight * 46
    minimumWidth:   ScreenTools.defaultFontPixelWidth * 70
    minimumHeight:  ScreenTools.defaultFontPixelHeight * 36
    visible:        false
    color:          qgcPal.window

    property var qgcPal: QGCPalette { colorGroupEnabled: true }

    // Called by the tool-strip button to reveal the window.
    function openWindow() {
        RadarBridgeManager.refreshPorts()
        root.show()
        root.raise()
        root.requestActivate()
    }

    readonly property var baudRates: [115200, 460800, 921600]
    readonly property var rateOptions: [5, 10, 20]

    ScrollView {
        anchors.fill:    parent
        anchors.margins: ScreenTools.defaultFontPixelWidth
        clip:            true

        ColumnLayout {
            width:   root.width - ScreenTools.defaultFontPixelWidth * 3
            spacing: ScreenTools.defaultFontPixelHeight * 0.6

            // ---- Title ----
            QGCLabel {
                text:            qsTr("Radar Bridge")
                font.pointSize:  ScreenTools.largeFontPointSize
                font.bold:       true
            }

            // ---- 1. Enable switch ----
            RowLayout {
                Layout.fillWidth: true
                QGCLabel { text: qsTr("Enable Radar Bridge"); Layout.fillWidth: true }
                QGCSwitch {
                    checked: RadarBridgeManager.enabled
                    onClicked: RadarBridgeManager.enabled = checked
                }
            }

            // ---- 2. Serial port selection ----
            // Two ways to set the port:
            //   - Pick a detected port from the dropdown (real serial ports), OR
            //   - Type any port name manually in the text field. This is needed
            //     for virtual ports such as socat's /dev/pts/N, which are NOT
            //     enumerated by QSerialPortInfo and therefore never show up in
            //     the dropdown.
            RowLayout {
                Layout.fillWidth: true
                QGCLabel { text: qsTr("Serial Port"); Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 22 }
                QGCComboBox {
                    id:               portCombo
                    Layout.fillWidth: true
                    model:            RadarBridgeManager.availablePorts
                    // Picking a detected port fills it into the manual field.
                    onActivated: (index) => {
                        var name = textAt(index)
                        RadarBridgeManager.serialPortName = name
                        portField.text = name
                    }
                }
                QGCButton {
                    text: qsTr("Refresh")
                    onClicked: RadarBridgeManager.refreshPorts()
                }
            }
            RowLayout {
                Layout.fillWidth: true
                QGCLabel {
                    text: qsTr("Port (manual)")
                    Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 22
                }
                QGCTextField {
                    id:               portField
                    Layout.fillWidth: true
                    text:             RadarBridgeManager.serialPortName
                    placeholderText:  qsTr("e.g. /dev/ttyUSB0, /dev/pts/3, COM3")
                    // Commit the typed name to the backend.
                    onEditingFinished: RadarBridgeManager.serialPortName = text
                }
            }

            // ---- 3. Baud rate selection ----
            RowLayout {
                Layout.fillWidth: true
                QGCLabel { text: qsTr("Baud Rate"); Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 22 }
                QGCComboBox {
                    id:               baudCombo
                    Layout.fillWidth: true
                    model:            root.baudRates
                    Component.onCompleted: {
                        var idx = root.baudRates.indexOf(RadarBridgeManager.baudRate)
                        currentIndex = idx >= 0 ? idx : 0
                    }
                    onActivated: (index) => { RadarBridgeManager.baudRate = root.baudRates[index] }
                }
            }

            // ---- 7. Send rate limit ----
            RowLayout {
                Layout.fillWidth: true
                QGCLabel { text: qsTr("Send Rate Limit"); Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 22 }
                QGCComboBox {
                    id:               rateCombo
                    Layout.fillWidth: true
                    model:            root.rateOptions.map(function(v){ return v + " Hz" })
                    Component.onCompleted: {
                        var idx = root.rateOptions.indexOf(RadarBridgeManager.sendRateHz)
                        currentIndex = idx >= 0 ? idx : 1
                    }
                    onActivated: (index) => { RadarBridgeManager.sendRateHz = root.rateOptions[index] }
                }
            }

            // ---- 5 & 6. Message type toggles ----
            RowLayout {
                Layout.fillWidth: true
                QGCLabel { text: qsTr("Send POSITION to PX4 (GPS_INPUT)"); Layout.fillWidth: true }
                QGCSwitch {
                    checked: RadarBridgeManager.sendPosition
                    onClicked: RadarBridgeManager.sendPosition = checked
                }
            }
            RowLayout {
                Layout.fillWidth: true
                QGCLabel { text: qsTr("Send RADAR_SCAN_2D to PX4 (OBSTACLE_DISTANCE)"); Layout.fillWidth: true }
                QGCSwitch {
                    checked: RadarBridgeManager.sendRadarScan
                    onClicked: RadarBridgeManager.sendRadarScan = checked
                }
            }

            // ---- 4. Connect / Disconnect ----
            RowLayout {
                Layout.fillWidth: true
                spacing:          ScreenTools.defaultFontPixelWidth
                QGCButton {
                    text:      qsTr("Connect")
                    enabled:   !RadarBridgeManager.connected
                    onClicked: RadarBridgeManager.connectRadar()
                }
                QGCButton {
                    text:      qsTr("Disconnect")
                    enabled:   RadarBridgeManager.connected
                    onClicked: RadarBridgeManager.disconnectRadar()
                }
                QGCButton {
                    text:      qsTr("Reset Stats")
                    onClicked: RadarBridgeManager.resetStatistics()
                }
            }

            // ---- Master send on/off button ----
            // One button to start/stop forwarding decoded data to the flight
            // controller. While OFF, frames are still parsed (stats update) but
            // nothing is sent to PX4.
            RowLayout {
                Layout.fillWidth: true
                QGCLabel { text: qsTr("Forward to PX4"); Layout.fillWidth: true }
                QGCButton {
                    text:            RadarBridgeManager.sendingEnabled ? qsTr("Stop Sending to PX4")
                                                                       : qsTr("Start Sending to PX4")
                    // Only meaningful once the radar link is up.
                    enabled:         RadarBridgeManager.connected
                    onClicked: {
                        if (RadarBridgeManager.sendingEnabled)
                            RadarBridgeManager.stopSending()
                        else
                            RadarBridgeManager.startSending()
                    }
                }
                Rectangle {
                    width:  ScreenTools.defaultFontPixelHeight * 0.9
                    height: width
                    radius: width / 2
                    color:  RadarBridgeManager.sendingEnabled ? "green" : "gray"
                }
            }

            // ---- 8. Connection status ----
            Rectangle {
                Layout.fillWidth: true
                height:           ScreenTools.defaultFontPixelHeight * 2
                radius:           4
                color:            qgcPal.windowShade
                RowLayout {
                    anchors.fill:        parent
                    anchors.leftMargin:  ScreenTools.defaultFontPixelWidth
                    anchors.rightMargin: ScreenTools.defaultFontPixelWidth
                    Rectangle {
                        width: ScreenTools.defaultFontPixelHeight * 0.8
                        height: width
                        radius: width / 2
                        color: RadarBridgeManager.connected ? "green" : "red"
                    }
                    QGCLabel {
                        Layout.fillWidth: true
                        text:  RadarBridgeManager.statusText
                    }
                }
            }

            // ---- 9. Statistics ----
            QGCLabel { text: qsTr("Statistics"); font.bold: true }
            GridLayout {
                Layout.fillWidth: true
                columns:          2
                columnSpacing:    ScreenTools.defaultFontPixelWidth * 2
                rowSpacing:       ScreenTools.defaultFontPixelHeight * 0.3

                QGCLabel { text: qsTr("Received Bytes") }
                QGCLabel { text: RadarBridgeManager.receivedBytes }
                QGCLabel { text: qsTr("Parsed Frames") }
                QGCLabel { text: RadarBridgeManager.parsedFrames }
                QGCLabel { text: qsTr("CRC Errors") }
                QGCLabel { text: RadarBridgeManager.crcErrors }
                QGCLabel { text: qsTr("Bad Frames") }
                QGCLabel { text: RadarBridgeManager.badFrames }
                QGCLabel { text: qsTr("SEQ Drops") }
                QGCLabel { text: RadarBridgeManager.seqDrops }
                QGCLabel { text: qsTr("MAVLink Sent") }
                QGCLabel { text: RadarBridgeManager.mavlinkSentCount }
                QGCLabel { text: qsTr("Input Frame Rate") }
                QGCLabel { text: RadarBridgeManager.inputFrameRate.toFixed(1) + " Hz" }
                QGCLabel { text: qsTr("Output MAVLink Rate") }
                QGCLabel { text: RadarBridgeManager.outputMavlinkRate.toFixed(1) + " Hz" }
            }

            // ---- 10. Last POSITION ----
            QGCLabel { text: qsTr("Last POSITION"); font.bold: true }
            GridLayout {
                Layout.fillWidth: true
                columns:          2
                columnSpacing:    ScreenTools.defaultFontPixelWidth * 2
                rowSpacing:       ScreenTools.defaultFontPixelHeight * 0.3

                QGCLabel { text: qsTr("Latitude") }
                QGCLabel { text: RadarBridgeManager.lastPosLat.toFixed(7) }
                QGCLabel { text: qsTr("Longitude") }
                QGCLabel { text: RadarBridgeManager.lastPosLon.toFixed(7) }
                QGCLabel { text: qsTr("Altitude (m)") }
                QGCLabel { text: RadarBridgeManager.lastPosAlt.toFixed(2) }
                QGCLabel { text: qsTr("time_ms") }
                QGCLabel { text: RadarBridgeManager.lastPosTimeMs }
            }
            // Raw hex of the last parsed POSITION frame (SOF..CRC).
            QGCLabel { text: qsTr("Parsed frame (hex)") }
            QGCLabel {
                Layout.fillWidth:   true
                wrapMode:           Text.WrapAnywhere
                font.family:        ScreenTools.fixedFontFamily
                text:               RadarBridgeManager.lastPositionHex
            }

            // ---- 11. Last RADAR_SCAN_2D ----
            QGCLabel { text: qsTr("Last RADAR_SCAN_2D"); font.bold: true }
            GridLayout {
                Layout.fillWidth: true
                columns:          2
                columnSpacing:    ScreenTools.defaultFontPixelWidth * 2
                rowSpacing:       ScreenTools.defaultFontPixelHeight * 0.3

                QGCLabel { text: qsTr("point_count") }
                QGCLabel { text: RadarBridgeManager.lastScanPointCount }
                QGCLabel { text: qsTr("min_distance_cm") }
                QGCLabel { text: RadarBridgeManager.lastScanMinCm }
                QGCLabel { text: qsTr("max_distance_cm") }
                QGCLabel { text: RadarBridgeManager.lastScanMaxCm }
            }
        }
    }
}
