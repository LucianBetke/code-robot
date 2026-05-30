// ============================================================
// File: WheelMeasurement.cpp
// Zweck:
//  - Geschwindigkeit in cm/s aus Encoder-Ticks berechnen
//  - Geschwindigkeit ueber Tickfenster bestimmen
//  - Stillstand ueber Timeout erkennen
// ============================================================

#include "WheelMeasurement.h"

#include "src/Encoder.h"
#include "src/RobotConfig.h"
#include "src/ScaleUtils.h"

namespace
{
    const float SPEED_ALPHA = 0.2f;
}

WheelMeasurement::WheelMeasurement(Enc& enc)
    : _enc(enc)
{
    _last_counts = _enc.getCounts();
    _counts_total = 0;

    _acc_counts = 0;
    _rps_filt = 0.0f;

    _last_time_ms = 0;
    _last_tick_ms = 0;
}

void WheelMeasurement::reset()
{
    _last_counts = _enc.getCounts();

    _counts_total = 0;

    _acc_counts = 0;
    _rps_filt = 0.0f;

    _last_time_ms = 0;
    _last_tick_ms = 0;
}

void WheelMeasurement::update(uint32_t now_ms)
{
    long cur = _enc.getCounts();
    long d = cur - _last_counts;

    if (d != 0)
    {
        if (d > 32767)
        {
            d = 32767;
        }

        if (d < -32768)
        {
            d = -32768;
        }

        updateFromTicks((int16_t)d, now_ms);

        _last_counts = cur;
        _last_tick_ms = now_ms;
    }

    timeoutCheck(now_ms);
}

void WheelMeasurement::updateFromTicks(int16_t dcounts, uint32_t now_ms)
{
    // Gesamtcounts muessen den ersten Tick nach reset() mitzaehlen.
    // Diese Werte werden fuer Odometrie / Plot gebraucht.
    _counts_total += dcounts;

    // Beim ersten Tick nach reset() oder Timeout existiert noch kein
    // gueltiges Zeitfenster fuer eine Geschwindigkeitsberechnung.
    //
    // Wichtig:
    //  - Der erste Tick wird fuer counts_total behalten.
    //  - Der erste Tick wird NICHT in _acc_counts uebernommen.
    //  - Dadurch entsteht kein kuenstlicher Startsprung in v_ist.
    if (_last_time_ms == 0)
    {
        _acc_counts = 0;
        _last_time_ms = now_ms;
        return;
    }

    _acc_counts += dcounts;

    const uint32_t dt_ms = now_ms - _last_time_ms;

    if (dt_ms < SPEED_MIN_DT_MS)
    {
        return;
    }

    _last_time_ms = now_ms;

    const float dt_s = (float)dt_ms * 0.001f;
    const float revs = (float)_acc_counts / (float)COUNTS_PER_REV;
    const float rps_i = revs / dt_s;

    _rps_filt += SPEED_ALPHA * (rps_i - _rps_filt);

    _acc_counts = 0;
}

void WheelMeasurement::timeoutCheck(uint32_t now_ms)
{
    if (_last_tick_ms == 0)
    {
        return;
    }

    if ((uint32_t)(now_ms - _last_tick_ms) > SPEED_TIMEOUT_MS)
    {
        _rps_filt = 0.0f;

        // Das aktuelle Geschwindigkeitsfenster ist nach Timeout ungueltig.
        // Beim naechsten Tick wird wieder nur eine neue Zeitreferenz gesetzt.
        _acc_counts = 0;
        _last_time_ms = 0;
    }
}

float WheelMeasurement::cms() const
{
    return _rps_filt * RAD_UMFANG_CM;
}

int16_t WheelMeasurement::cmsInt() const
{
    return scaleRoundToInt16(cms());
}