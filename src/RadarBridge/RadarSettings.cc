/****************************************************************************
 *
 * RadarSettings implementation.
 *
 ****************************************************************************/

#include "RadarSettings.h"

#include <QtCore/QSettings>

namespace {
constexpr const char *kGroup          = "RadarBridge";
constexpr const char *kEnabled        = "enabled";
constexpr const char *kPortName       = "serialPortName";
constexpr const char *kBaudRate       = "baudRate";
constexpr const char *kSendPosition   = "sendPosition";
constexpr const char *kSendRadarScan  = "sendRadarScan";
constexpr const char *kSendRateHz     = "sendRateHz";
constexpr const char *kTargetId       = "targetId";
}

RadarSettings::RadarSettings(QObject *parent)
    : QObject(parent)
{
    _load();
}

void RadarSettings::_load()
{
    QSettings s;
    s.beginGroup(kGroup);
    _enabled        = s.value(kEnabled,        false).toBool();
    _serialPortName = s.value(kPortName,       QString()).toString();
    _baudRate       = s.value(kBaudRate,       115200).toInt();
    _sendPosition   = s.value(kSendPosition,   true).toBool();
    _sendRadarScan  = s.value(kSendRadarScan,  true).toBool();
    _sendRateHz     = s.value(kSendRateHz,     10).toInt();
    _targetId       = qMax(1, s.value(kTargetId, 1).toInt());
    s.endGroup();
}

void RadarSettings::_save()
{
    QSettings s;
    s.beginGroup(kGroup);
    s.setValue(kEnabled,        _enabled);
    s.setValue(kPortName,       _serialPortName);
    s.setValue(kBaudRate,       _baudRate);
    s.setValue(kSendPosition,   _sendPosition);
    s.setValue(kSendRadarScan,  _sendRadarScan);
    s.setValue(kSendRateHz,     _sendRateHz);
    s.setValue(kTargetId,       _targetId);
    s.endGroup();
}

void RadarSettings::setEnabled(bool v)
{
    if (_enabled == v) return;
    _enabled = v;
    _save();
    emit enabledChanged();
}

void RadarSettings::setSerialPortName(const QString &v)
{
    if (_serialPortName == v) return;
    _serialPortName = v;
    _save();
    emit serialPortNameChanged();
}

void RadarSettings::setBaudRate(int v)
{
    if (_baudRate == v) return;
    _baudRate = v;
    _save();
    emit baudRateChanged();
}

void RadarSettings::setSendPosition(bool v)
{
    if (_sendPosition == v) return;
    _sendPosition = v;
    _save();
    emit sendPositionChanged();
}

void RadarSettings::setSendRadarScan(bool v)
{
    if (_sendRadarScan == v) return;
    _sendRadarScan = v;
    _save();
    emit sendRadarScanChanged();
}

void RadarSettings::setSendRateHz(int v)
{
    if (v <= 0) v = 1;
    if (_sendRateHz == v) return;
    _sendRateHz = v;
    _save();
    emit sendRateHzChanged();
}

void RadarSettings::setTargetId(int v)
{
    v = qMax(1, v);
    if (_targetId == v) return;
    _targetId = v;
    _save();
    emit targetIdChanged();
}

// ---------------------------------------------------------------------------
// NOTE: Migrating to QGC's SettingsManager group system
// ---------------------------------------------------------------------------
// If you prefer these settings to appear alongside the other QGC settings
// groups (e.g. so they can be edited via Fact-based UI), create a subclass of
// SettingsGroup using the DECLARE_SETTINGGROUP / DECLARE_SETTINGSFACT macros,
// as done in src/Settings/RTKSettings.{h,cc}, and register it in
// SettingsManager. That approach gives you Facts (with metadata json) instead
// of the plain Q_PROPERTYs used here. For a bolt-on module the plain QSettings
// approach above keeps the footprint minimal.
// ---------------------------------------------------------------------------
