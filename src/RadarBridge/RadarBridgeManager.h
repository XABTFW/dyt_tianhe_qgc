/****************************************************************************
 *
 * RadarBridgeManager
 *
 * Top-level orchestrator for the radar bridge feature and the object exposed
 * to QML. Responsibilities:
 *   - Open / close / auto-reconnect the radar serial port (QSerialPort).
 *   - Feed received bytes into the RadarProtocolParser.
 *   - Decode POSITION / RADAR_SCAN_2D payloads.
 *   - Apply an outgoing send-rate limit.
 *   - Forward converted MAVLink messages to PX4 via RadarMavlinkBridge.
 *   - Expose live status + statistics to the QML settings/monitor page.
 *
 * Registered as a QML singleton (see registerQmlTypes) so the page can access
 * it as `RadarBridgeManager` after `import QGroundControl.RadarBridge 1.0`.
 *
 ****************************************************************************/

#pragma once

#include "RadarTypes.h"

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QStringList>

class QSerialPort;
class QTimer;
class RadarProtocolParser;
class RadarMavlinkBridge;
class RadarSettings;

class RadarBridgeManager : public QObject
{
    Q_OBJECT

    // --- Configuration (proxied to RadarSettings) -----------------------
    Q_PROPERTY(bool        enabled        READ enabled        WRITE setEnabled        NOTIFY enabledChanged)
    Q_PROPERTY(QString     serialPortName READ serialPortName WRITE setSerialPortName NOTIFY serialPortNameChanged)
    Q_PROPERTY(int         baudRate       READ baudRate       WRITE setBaudRate       NOTIFY baudRateChanged)
    Q_PROPERTY(bool        sendPosition   READ sendPosition   WRITE setSendPosition   NOTIFY sendPositionChanged)
    Q_PROPERTY(bool        sendRadarScan  READ sendRadarScan  WRITE setSendRadarScan  NOTIFY sendRadarScanChanged)
    Q_PROPERTY(int         sendRateHz     READ sendRateHz     WRITE setSendRateHz     NOTIFY sendRateHzChanged)
    Q_PROPERTY(int         targetId       READ targetId       WRITE setTargetId       NOTIFY targetIdChanged)

    // Master gate for forwarding to the flight controller. When false, frames
    // are still parsed/decoded (stats + last-sample update) but NOTHING is sent
    // to PX4. This is the on/off button in the UI.
    Q_PROPERTY(bool        sendingEnabled READ sendingEnabled WRITE setSendingEnabled NOTIFY sendingEnabledChanged)

    // --- Connection state ------------------------------------------------
    Q_PROPERTY(bool        connected      READ connected                              NOTIFY connectedChanged)
    Q_PROPERTY(QString     statusText     READ statusText                             NOTIFY statusTextChanged)
    Q_PROPERTY(QStringList availablePorts READ availablePorts                         NOTIFY availablePortsChanged)

    // --- Statistics ------------------------------------------------------
    Q_PROPERTY(quint64     receivedBytes    READ receivedBytes    NOTIFY statisticsChanged)
    Q_PROPERTY(quint64     parsedFrames     READ parsedFrames     NOTIFY statisticsChanged)
    Q_PROPERTY(quint64     crcErrors        READ crcErrors        NOTIFY statisticsChanged)
    Q_PROPERTY(quint64     badFrames        READ badFrames        NOTIFY statisticsChanged)
    Q_PROPERTY(quint64     seqDrops         READ seqDrops         NOTIFY statisticsChanged)
    Q_PROPERTY(quint64     mavlinkSentCount READ mavlinkSentCount NOTIFY statisticsChanged)
    Q_PROPERTY(double      inputFrameRate   READ inputFrameRate   NOTIFY ratesChanged)
    Q_PROPERTY(double      outputMavlinkRate READ outputMavlinkRate NOTIFY ratesChanged)

    // --- Latest decoded data (for the monitor UI) ------------------------
    Q_PROPERTY(double  lastPosLat        READ lastPosLat        NOTIFY lastPositionChanged)
    Q_PROPERTY(double  lastPosLon        READ lastPosLon        NOTIFY lastPositionChanged)
    Q_PROPERTY(double  lastPosAlt        READ lastPosAlt        NOTIFY lastPositionChanged)
    Q_PROPERTY(quint32 lastPosTimeMs     READ lastPosTimeMs     NOTIFY lastPositionChanged)
    Q_PROPERTY(QString lastPositionHex   READ lastPositionHex   NOTIFY lastPositionChanged)
    Q_PROPERTY(int     lastScanPointCount READ lastScanPointCount NOTIFY lastScanChanged)
    Q_PROPERTY(int     lastScanMinCm     READ lastScanMinCm     NOTIFY lastScanChanged)
    Q_PROPERTY(int     lastScanMaxCm     READ lastScanMaxCm     NOTIFY lastScanChanged)

public:
    explicit RadarBridgeManager(QObject *parent = nullptr);
    ~RadarBridgeManager() override;

    static RadarBridgeManager *instance();
    static void registerQmlTypes();

    /// Called once at app startup. Wires everything up and auto-connects if
    /// the persisted settings have the bridge enabled.
    void init();

    // --- QML-invokable control ------------------------------------------
    Q_INVOKABLE void connectRadar();
    Q_INVOKABLE void disconnectRadar();
    Q_INVOKABLE void refreshPorts();
    Q_INVOKABLE void resetStatistics();

    // --- Property getters -----------------------------------------------
    bool        enabled()        const;
    QString     serialPortName() const;
    int         baudRate()       const;
    bool        sendPosition()   const;
    bool        sendRadarScan()  const;
    int         sendRateHz()     const;
    int         targetId()       const;
    bool        sendingEnabled() const { return _sendingEnabled; }

    bool        connected()      const { return _connected; }
    QString     statusText()     const { return _statusText; }
    QStringList availablePorts() const { return _availablePorts; }

    quint64 receivedBytes()    const;
    quint64 parsedFrames()     const;
    quint64 crcErrors()        const;
    quint64 badFrames()        const;
    quint64 seqDrops()         const;
    quint64 mavlinkSentCount() const { return _mavlinkSentCount; }
    double  inputFrameRate()   const { return _inputFrameRate; }
    double  outputMavlinkRate() const { return _outputMavlinkRate; }

    double  lastPosLat()        const { return _lastPosition.latDeg(); }
    double  lastPosLon()        const { return _lastPosition.lonDeg(); }
    double  lastPosAlt()        const { return _lastPosition.altM(); }
    quint32 lastPosTimeMs()     const { return _lastPosition.timeMs; }
    QString lastPositionHex()   const { return _lastPositionHex; }
    int     lastScanPointCount() const { return _lastScan.pointCount; }
    int     lastScanMinCm()     const { return _lastScan.minDistanceCm; }
    int     lastScanMaxCm()     const { return _lastScan.maxDistanceCm; }

    // --- Property setters (proxy to RadarSettings) ----------------------
    void setEnabled(bool v);
    void setSerialPortName(const QString &v);
    void setBaudRate(int v);
    void setSendPosition(bool v);
    void setSendRadarScan(bool v);
    void setSendRateHz(int v);
    void setTargetId(int v);
    void setSendingEnabled(bool v);

    // QML convenience toggles for the on/off button.
    Q_INVOKABLE void startSending() { setSendingEnabled(true); }
    Q_INVOKABLE void stopSending()  { setSendingEnabled(false); }

signals:
    void enabledChanged();
    void serialPortNameChanged();
    void baudRateChanged();
    void sendPositionChanged();
    void sendRadarScanChanged();
    void sendRateHzChanged();
    void targetIdChanged();
    void sendingEnabledChanged();
    void connectedChanged();
    void statusTextChanged();
    void availablePortsChanged();
    void statisticsChanged();
    void ratesChanged();
    void lastPositionChanged();
    void lastScanChanged();

private slots:
    void _onSerialReadyRead();
    void _onSerialErrorOccurred(int error);
    void _onFrameParsed(quint8 seq, const QByteArray &payload, const QByteArray &rawFrame);
    void _onReconnectTimer();
    void _onRateTimer();

private:
    void _setConnected(bool c);
    void _setStatus(const QString &s);
    bool _openPort();
    void _closePort();
    void _handlePosition(const QByteArray &payload, const QByteArray &rawFrame);
    // Rate limiter: returns true if a message of this type may be sent now.
    bool _rateAllows(qint64 &lastSentMs) const;

    QSerialPort         *_serialPort   = nullptr;
    RadarProtocolParser *_parser       = nullptr;
    RadarMavlinkBridge  *_bridge       = nullptr;
    RadarSettings       *_settings     = nullptr;
    QTimer              *_reconnectTimer = nullptr;
    QTimer              *_rateTimer    = nullptr;

    bool        _connected = false;
    bool        _sendingEnabled = false;   ///< master gate for forwarding to PX4
    QString     _statusText = QStringLiteral("Disconnected");
    QStringList _availablePorts;

    // Rate-limit bookkeeping (wall-clock ms of last send per stream).
    qint64 _lastPositionSentMs = 0;
    qint64 _lastScanSentMs     = 0;

    // Output MAVLink counter (bridge increments via signal).
    quint64 _mavlinkSentCount = 0;

    // Latest decoded samples.
    RadarProtocol::PositionData    _lastPosition;
    RadarProtocol::RadarScan2DData _lastScan;
    QString                        _lastPositionHex;   ///< raw hex of last POSITION frame

    // Rate computation (1 s sliding window driven by _rateTimer).
    quint64 _framesAtLastTick  = 0;
    quint64 _mavlinkAtLastTick = 0;
    double  _inputFrameRate    = 0.0;
    double  _outputMavlinkRate = 0.0;
    QElapsedTimer _rateClock;
};
