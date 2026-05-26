// ============================================================
// File: SpeedWeg.cpp
// ============================================================

#include "SpeedWeg.h"
#include "src/globals.h"
#include "src/Encoder.h"

// ------------------------------------------------------------
// Lokale Konstanten
// ------------------------------------------------------------
namespace
{
    const float SPEED_ALPHA = 0.2f;
}

// ------------------------------------------------------------
// Konstruktor
// ------------------------------------------------------------
SpeedWeg::SpeedWeg(Enc& enc)
    : _enc(enc)
{
    _last_counts = _enc.getCounts();
    _counts_total = 0;

    _acc_counts = 0;
    _rps_filt = 0.0f;
    _last_time_ms = 0;
    _last_tick_ms = 0;
}

// ------------------------------------------------------------
void SpeedWeg::reset()
{
    _counts_total = 0;

    _acc_counts = 0;
    _rps_filt = 0.0f;
    _last_time_ms = 0;
    _last_tick_ms = 0;

    _last_counts = _enc.getCounts();
}

// ------------------------------------------------------------
// Encoder poll + Berechnung
// Zeitbasis: ms
// ------------------------------------------------------------
void SpeedWeg::update(uint32_t now_ms)
{
    long cur = _enc.getCounts();
    long d = cur - _last_counts;

    if (d != 0)
    {
        if (d > 32767) d = 32767;
        if (d < -32768) d = -32768;

        updateFromTicks((int16_t)d, now_ms);

        _last_counts = cur;
        _last_tick_ms = now_ms;
    }

    timeoutCheck(now_ms);
}

// ------------------------------------------------------------
void SpeedWeg::updateFromTicks(int16_t dcounts, uint32_t now_ms)
{
    // 1) Counts immer summieren.
    // Diese Gesamtcounts sind die Grundlage fuer Odometrie/Plot.
    _counts_total += dcounts;

    // 2) Tick-Sammelpuffer fuer Geschwindigkeitsfenster
    _acc_counts += dcounts;

    // 3) Zeit seit letzter Geschwindigkeitsmessung
    if (_last_time_ms == 0)
    {
        _last_time_ms = now_ms;
        return;
    }

    uint32_t dt_ms = now_ms - _last_time_ms;

    // Noch nicht genug Zeit? Nichts tun.
    if (dt_ms < SPEED_MIN_DT_MS)
    {
        return;
    }

    // Zeit aktualisieren: Fensterende
    _last_time_ms = now_ms;

    // 4) Geschwindigkeit berechnen
    const float dt_s = (float)dt_ms * 0.001f;
    const float revs = (float)_acc_counts / (float)COUNTS_PER_REV;
    const float rps_i = revs / dt_s;

    // 5) Tiefpass
    _rps_filt += SPEED_ALPHA * (rps_i - _rps_filt);

    // 6) Fenster zuruecksetzen
    _acc_counts = 0;
}

// ------------------------------------------------------------
void SpeedWeg::timeoutCheck(uint32_t now_ms)
{
    if (_last_tick_ms == 0)
    {
        return;
    }

    if ((uint32_t)(now_ms - _last_tick_ms) > SPEED_TIMEOUT_MS)
    {
        _rps_filt = 0.0f;
    }
}

// ------------------------------------------------------------
float SpeedWeg::mps() const
{
    return (_rps_filt * RAD_UMFANG_MM) / 1000.0f;
}