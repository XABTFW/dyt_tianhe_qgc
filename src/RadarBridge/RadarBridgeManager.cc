/****************************************************************************
 *
 * RadarBridgeManager implementation.
 *
 ****************************************************************************/

#include "RadarBridgeManager.h"
#include "RadarProtocolParser.h"
#include "RadarMavlinkBridge.h"
#include "RadarSettings.h"

#include <QtCore/QDateTime>
#include <QtCore/QTimer>
#include <QtCore/qapplicationstatic.h>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QtQml/QQmlEngine>

using namespace RadarProtocol;

// Application-lifetime singleton, matching the pattern used by QGC managers.
Q_APPLICATION_STATIC(RadarBridgeManager, _radarBridgeManagerInstance);

namespace {
constexpr int kReconnectIntervalMs = 2000; ///< auto-reconnect retry period
constexpr int kRateTimerMs         = 1000; ///< rate recomputation period
}

RadarBridgeManager::RadarBridgeManager(QObject *parent)
    : QObject(parent)
{
}

RadarBridgeManager::~RadarBridgeManager()
{
    _closePort();
}

RadarBridgeManager *RadarBridgeManager::instance()
{
    return _radarBridgeManagerInstance();
}

void RadarBridgeManager::registerQmlTypes()
{
    // Exposed as a QML singleton. In QML:
    //   import QGroundControl.RadarBridge 1.0
    //   ... RadarBridgeManager.enabled, RadarBridgeManager.connectRadar() ...
    qmlRegisterSingletonType<RadarBridgeManager>(
        "QGroundControl.RadarBridge", 1, 0, "RadarBridgeManager",
        [](QQmlEngine *, QJSEngine *) -> QObject * {
            QObject *inst = RadarBridgeManager::instance();
            // The C++ singleton owns its lifetime; QML must not delete it.
            QQmlEngine::setObjectOwnership(inst, QQmlEngine::CppOwnership);
            return inst;
        });
}

void RadarBridgeManager::init()
{
    // Build the collaborators (parented to this so they live as long as we do).
    _settings       = new RadarSettings(this);
    _parser         = new RadarProtocolParser(this);
    _bridge         = new RadarMavlinkBridge(this);
    _serialPort     = new QSerialPort(this);
    _reconnectTimer = new QTimer(this);
    _rateTimer      = new QTimer(this);

    _reconnectTimer->setInterval(kReconnectIntervalMs);
    _rateTimer->setInterval(kRateTimerMs);

    // --- Serial port wiring ---
    connect(_serialPort, &QSerialPort::readyRead, this, &RadarBridgeManager::_onSerialReadyRead);
    connect(_serialPort, &QSerialPort::errorOccurred, this,
            [this](QSerialPort::SerialPortError e) { _onSerialErrorOccurred(static_cast<int>(e)); });

    // --- Parser wiring ---
    connect(_parser, &RadarProtocolParser::frameParsed, this, &RadarBridgeManager::_onFrameParsed);
    connect(_parser, &RadarProtocolParser::statisticsChanged, this, &RadarBridgeManager::statisticsChanged);

    // --- Bridge wiring: count every sent MAVLink message ---
    connect(_bridge, &RadarMavlinkBridge::mavlinkMessageSent, this, [this]() {
        ++_mavlinkSentCount;
        emit statisticsChanged();
    });

    // --- Settings change propagation to our own signals ---
    connect(_settings, &RadarSettings::enabledChanged,        this, &RadarBridgeManager::enabledChanged);
    connect(_settings, &RadarSettings::serialPortNameChanged, this, &RadarBridgeManager::serialPortNameChanged);
    connect(_settings, &RadarSettings::baudRateChanged,       this, &RadarBridgeManager::baudRateChanged);
    connect(_settings, &RadarSettings::sendPositionChanged,   this, &RadarBridgeManager::sendPositionChanged);
    connect(_settings, &RadarSettings::sendRadarScanChanged,  this, &RadarBridgeManager::sendRadarScanChanged);
    connect(_settings, &RadarSettings::sendRateHzChanged,     this, &RadarBridgeManager::sendRateHzChanged);

    // --- Timers ---
    connect(_reconnectTimer, &QTimer::timeout, this, &RadarBridgeManager::_onReconnectTimer);
    connect(_rateTimer, &QTimer::timeout, this, &RadarBridgeManager::_onRateTimer);

    _rateClock.start();
    _rateTimer->start();

    refreshPorts();

    // Auto-start if the user had it enabled last session.
    if (_settings->enabled()) {
        connectRadar();
    }
}

// ---------------------------------------------------------------------------
// Configuration proxies
// ---------------------------------------------------------------------------
bool    RadarBridgeManager::enabled()        const { return _settings && _settings->enabled(); }
QString RadarBridgeManager::serialPortName() const { return _settings ? _settings->serialPortName() : QString(); }
int     RadarBridgeManager::baudRate()       const { return _settings ? _settings->baudRate() : 115200; }
bool    RadarBridgeManager::sendPosition()   const { return _settings && _settings->sendPosition(); }
bool    RadarBridgeManager::sendRadarScan()  const { return _settings && _settings->sendRadarScan(); }
int     RadarBridgeManager::sendRateHz()     const { return _settings ? _settings->sendRateHz() : 10; }

void RadarBridgeManager::setEnabled(bool v)
{
    if (!_settings) return;
    _settings->setEnabled(v);
    // Enabling/disabling the bridge starts or stops the link immediately.
    if (v) {
        connectRadar();
    } else {
        disconnectRadar();
    }
}

void RadarBridgeManager::setSendingEnabled(bool v)
{
    if (_sendingEnabled == v) return;
    _sendingEnabled = v;
    _setStatus(v ? QStringLiteral("Sending to PX4: ON")
                 : QStringLiteral("Sending to PX4: OFF"));
    emit sendingEnabledChanged();
}

void RadarBridgeManager::setSerialPortName(const QString &v) { if (_settings) _settings->setSerialPortName(v); }
void RadarBridgeManager::setBaudRate(int v)                  { if (_settings) _settings->setBaudRate(v); }
void RadarBridgeManager::setSendPosition(bool v)            { if (_settings) _settings->setSendPosition(v); }
void RadarBridgeManager::setSendRadarScan(bool v)          { if (_settings) _settings->setSendRadarScan(v); }
void RadarBridgeManager::setSendRateHz(int v)              { if (_settings) _settings->setSendRateHz(v); }

// ---------------------------------------------------------------------------
// Statistics proxies
// ---------------------------------------------------------------------------
quint64 RadarBridgeManager::receivedBytes() const { return _parser ? _parser->statistics().receivedBytes : 0; }
quint64 RadarBridgeManager::parsedFrames()  const { return _parser ? _parser->statistics().parsedFrames : 0; }
quint64 RadarBridgeManager::crcErrors()     const { return _parser ? _parser->statistics().crcErrors : 0; }
quint64 RadarBridgeManager::badFrames()     const { return _parser ? _parser->statistics().badFrames : 0; }
quint64 RadarBridgeManager::seqDrops()      const { return _parser ? _parser->statistics().seqDrops : 0; }

void RadarBridgeManager::resetStatistics()
{
    if (_parser) _parser->resetStatistics();
    _mavlinkSentCount = 0;
    _framesAtLastTick = 0;
    _mavlinkAtLastTick = 0;
    emit statisticsChanged();
}

// ---------------------------------------------------------------------------
// Port enumeration & control
// ---------------------------------------------------------------------------
void RadarBridgeManager::refreshPorts()
{
    QStringList ports;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ports << info.portName();
    }
    if (ports != _availablePorts) {
        _availablePorts = ports;
        emit availablePortsChanged();
    }
}

void RadarBridgeManager::connectRadar()
{
    if (!_serialPort) return;

    if (_connected) {
        return; // already open
    }
    if (_openPort()) {
        _reconnectTimer->stop();
    } else {
        // Keep retrying in the background while the feature is enabled.
        if (enabled()) {
            _reconnectTimer->start();
        }
    }
}

void RadarBridgeManager::disconnectRadar()
{
    _reconnectTimer->stop();
    _closePort();
}

bool RadarBridgeManager::_openPort()
{
    const QString name = serialPortName();
    if (name.isEmpty()) {
        _setStatus(QStringLiteral("No serial port selected"));
        return false;
    }

    _serialPort->setPortName(name);
    _serialPort->setBaudRate(baudRate());
    _serialPort->setDataBits(QSerialPort::Data8);
    _serialPort->setParity(QSerialPort::NoParity);
    _serialPort->setStopBits(QSerialPort::OneStop);
    _serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (!_serialPort->open(QIODevice::ReadOnly)) {
        _setStatus(QStringLiteral("Open failed: %1").arg(_serialPort->errorString()));
        _setConnected(false);
        return false;
    }

    if (_parser) _parser->reset();
    _setStatus(QStringLiteral("Connected to %1 @ %2").arg(name).arg(baudRate()));
    _setConnected(true);
    return true;
}

void RadarBridgeManager::_closePort()
{
    if (_serialPort && _serialPort->isOpen()) {
        _serialPort->close();
    }
    if (_connected) {
        _setStatus(QStringLiteral("Disconnected"));
        _setConnected(false);
    }
}

// ---------------------------------------------------------------------------
// Serial events
// ---------------------------------------------------------------------------
void RadarBridgeManager::_onSerialReadyRead()
{
    if (!_serialPort || !_parser) return;
    const QByteArray chunk = _serialPort->readAll();
    _parser->addData(chunk);
}

void RadarBridgeManager::_onSerialErrorOccurred(int error)
{
    // QSerialPort::NoError == 0
    if (error == QSerialPort::NoError) {
        return;
    }
    // On a resource/device error, close and let the reconnect timer retry.
    if (error == QSerialPort::ResourceError ||
        error == QSerialPort::PermissionError ||
        error == QSerialPort::DeviceNotFoundError) {
        _setStatus(QStringLiteral("Serial error (%1), will retry").arg(error));
        _closePort();
        if (enabled()) {
            _reconnectTimer->start();
        }
    }
}

void RadarBridgeManager::_onReconnectTimer()
{
    // Periodic attempt to (re)open the port while enabled.
    if (!enabled()) {
        _reconnectTimer->stop();
        return;
    }
    refreshPorts();
    if (!_connected) {
        connectRadar();
    }
}

// ---------------------------------------------------------------------------
// Frame handling
// ---------------------------------------------------------------------------
void RadarBridgeManager::_onFrameParsed(quint8 seq, const QByteArray &payload, const QByteArray &rawFrame)
{
    Q_UNUSED(seq)
    // The parser only emits POSITION frames.
    _handlePosition(payload, rawFrame);
}

void RadarBridgeManager::_handlePosition(const QByteArray &payload, const QByteArray &rawFrame)
{
    // Payload layout (16 bytes): time_ms(4) lat_e7(4) lon_e7(4) alt_mm(4)
    if (payload.size() < kPayloadLen) {
        return; // malformed; parser already validated CRC/framing
    }

    // Keep the raw frame as an upper-case, space-separated hex string for the UI.
    _lastPositionHex = QString::fromLatin1(rawFrame.toHex(' ')).toUpper();

    LeReader r(reinterpret_cast<const uint8_t *>(payload.constData()), payload.size());
    PositionData pos;
    r.seek(kPayOffTimeMs); pos.timeMs = r.u32();  // @0
    r.seek(kPayOffLat);    pos.latE7  = r.i32();  // @4
    r.seek(kPayOffLon);    pos.lonE7  = r.i32();  // @8
    r.seek(kPayOffAlt);    pos.altMm  = r.i32();  // @12
    pos.valid  = r.ok();
    if (!pos.valid) {
        return;
    }

    _lastPosition = pos;
    emit lastPositionChanged();

    // Master gate + per-type toggle + rate limit before forwarding to PX4.
    if (_sendingEnabled && sendPosition() && _rateAllows(_lastPositionSentMs)) {
        _bridge->sendPositionAsGpsInput(pos);
        // Optional map display copy (see header note about GLOBAL_POSITION_INT):
        // _bridge->sendPositionAsGlobalPositionInt(pos);
    }
}

// ---------------------------------------------------------------------------
// Rate limiting
// ---------------------------------------------------------------------------
bool RadarBridgeManager::_rateAllows(qint64 &lastSentMs) const
{
    const int hz = qMax(1, sendRateHz());
    const qint64 minIntervalMs = 1000 / hz;
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (now - lastSentMs < minIntervalMs) {
        return false;
    }
    lastSentMs = now;
    return true;
}

// ---------------------------------------------------------------------------
// Rate computation (1 Hz)
// ---------------------------------------------------------------------------
void RadarBridgeManager::_onRateTimer()
{
    const double elapsedS = _rateClock.restart() / 1000.0;
    if (elapsedS <= 0.0) {
        return;
    }

    const quint64 frames = parsedFrames();
    const quint64 sent   = _mavlinkSentCount;

    _inputFrameRate    = static_cast<double>(frames - _framesAtLastTick) / elapsedS;
    _outputMavlinkRate = static_cast<double>(sent - _mavlinkAtLastTick) / elapsedS;

    _framesAtLastTick  = frames;
    _mavlinkAtLastTick = sent;

    emit ratesChanged();
}

// ---------------------------------------------------------------------------
// State helpers
// ---------------------------------------------------------------------------
void RadarBridgeManager::_setConnected(bool c)
{
    if (_connected == c) return;
    _connected = c;
    emit connectedChanged();
}

void RadarBridgeManager::_setStatus(const QString &s)
{
    if (_statusText == s) return;
    _statusText = s;
    emit statusTextChanged();
}
