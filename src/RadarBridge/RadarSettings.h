/****************************************************************************
 *
 * RadarSettings
 *
 * Persistent configuration for the radar bridge. Backed by QSettings so it
 * survives restarts. Kept intentionally self-contained (a plain QObject)
 * rather than wired into QGC's DECLARE_SETTINGGROUP macro system, so the
 * module can be dropped in without touching SettingsManager. See the notes
 * at the bottom of RadarSettings.cc for how to migrate it into the QGC
 * SettingsManager group system if desired.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>

class RadarSettings : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool    enabled        READ enabled        WRITE setEnabled        NOTIFY enabledChanged)
    Q_PROPERTY(QString serialPortName READ serialPortName WRITE setSerialPortName NOTIFY serialPortNameChanged)
    Q_PROPERTY(int     baudRate       READ baudRate       WRITE setBaudRate       NOTIFY baudRateChanged)
    Q_PROPERTY(bool    sendPosition   READ sendPosition   WRITE setSendPosition   NOTIFY sendPositionChanged)
    Q_PROPERTY(bool    sendRadarScan  READ sendRadarScan  WRITE setSendRadarScan  NOTIFY sendRadarScanChanged)
    Q_PROPERTY(int     sendRateHz     READ sendRateHz     WRITE setSendRateHz     NOTIFY sendRateHzChanged)
    Q_PROPERTY(int     targetId       READ targetId       WRITE setTargetId       NOTIFY targetIdChanged)

public:
    explicit RadarSettings(QObject *parent = nullptr);

    bool    enabled()        const { return _enabled; }
    QString serialPortName() const { return _serialPortName; }
    int     baudRate()       const { return _baudRate; }
    bool    sendPosition()   const { return _sendPosition; }
    bool    sendRadarScan()  const { return _sendRadarScan; }
    int     sendRateHz()     const { return _sendRateHz; }
    int     targetId()       const { return _targetId; }

    void setEnabled(bool v);
    void setSerialPortName(const QString &v);
    void setBaudRate(int v);
    void setSendPosition(bool v);
    void setSendRadarScan(bool v);
    void setSendRateHz(int v);
    void setTargetId(int v);

signals:
    void enabledChanged();
    void serialPortNameChanged();
    void baudRateChanged();
    void sendPositionChanged();
    void sendRadarScanChanged();
    void sendRateHzChanged();
    void targetIdChanged();

private:
    void _load();
    void _save();

    bool    _enabled        = false;
    QString _serialPortName;
    int     _baudRate       = 115200;
    bool    _sendPosition   = true;
    bool    _sendRadarScan  = true;
    int     _sendRateHz     = 10;
    int     _targetId       = 1;
};
