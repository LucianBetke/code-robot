// ============================================================
// File: SpeedWeg.cpp
// ============================================================

#include "SpeedWeg.h"
#include "Encoder.h"
#include "globals.h"
#include <math.h>

// ------------------------------------------------------------
// Konstruktor
// ------------------------------------------------------------
SpeedWeg::SpeedWeg(Enc& enc)
    : _enc(enc)
{
    _last_counts = _enc.getCounts();
    _counts_total = 0;
    _weg_mm_total = 0.0f;
    _weg_abs_mm_total = 0.0f;

    _acc_counts = 0;
    _alpha = 0.2f;
    _rps_filt = 0.0f;
    _last_time_ms = 0;
    _timeout_ms = 60;
    _last_tick_ms = 0;
}

// ------------------------------------------------------------
void SpeedWeg::reset() {
    _counts_total = 0;
    _weg_mm_total = 0.0f;
    _weg_abs_mm_total = 0.0f;

    _acc_counts = 0;
    _rps_filt = 0.0f;
    _last_time_ms = 0;
    _last_tick_ms = 0;

    _last_counts = _enc.getCounts();
}

// ------------------------------------------------------------
void SpeedWeg::setAlpha(float a) {
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;
    _alpha = a;
}

// ------------------------------------------------------------
void SpeedWeg::setTimeoutMs(uint16_t ms) {
    _timeout_ms = (uint32_t)ms;
}

// ------------------------------------------------------------
// Encoder poll + Berechnung  (Zeitbasis: ms)
// ------------------------------------------------------------
void SpeedWeg::pollEncoder(uint32_t now_ms) {
    long cur = _enc.getCounts();
    long d = cur - _last_counts;

    if (d != 0) {
        if (d > 32767) d = 32767;
        if (d < -32768) d = -32768;

        updateFromTicks((int16_t)d, now_ms);
        _last_counts = cur;
        _last_tick_ms = now_ms;   // Zeitpunkt dieses Ticks merken
    }

    timeoutCheck(now_ms);
}

// ------------------------------------------------------------
void SpeedWeg::updateFromTicks(int16_t dcounts, uint32_t now_ms) {

    // 1) Weg immer summieren
    const float d_mm =
        ((float)dcounts / (float)COUNTS_PER_REV) * RAD_UMFANG_MM;

    _counts_total += dcounts;
    _weg_mm_total += d_mm;
    _weg_abs_mm_total += fabsf(d_mm);

    // 2) Tick-Sammelpuffer
    _acc_counts += dcounts;

    // 3) Zeit seit letzter Geschwindigkeitsmessung
    if (_last_time_ms == 0) {
        _last_time_ms = now_ms;
        return;
    }

    uint32_t dt_ms = now_ms - _last_time_ms;

    // Noch nicht genug Zeit? Nichts tun.
    if (dt_ms < SPEED_MIN_DT_MS) {
        return;
    }

    // Zeit aktualisieren (Fensterende)
    _last_time_ms = now_ms;

    // 4) Geschwindigkeit berechnen
    const float dt_s = (float)dt_ms * 0.001f;
    const float revs = (float)_acc_counts / (float)COUNTS_PER_REV;
    const float rps_i = revs / dt_s;

    // 5) Tiefpass
    _rps_filt += _alpha * (rps_i - _rps_filt);

    // 6) Fenster zurücksetzen
    _acc_counts = 0;
}

// ------------------------------------------------------------
void SpeedWeg::timeoutCheck(uint32_t now_ms) {
    if (_last_tick_ms == 0) return;  // noch kein Tick

    if ((now_ms - _last_tick_ms) > _timeout_ms) {
        _rps_filt = 0.0f;
    }
}

// ------------------------------------------------------------
float SpeedWeg::mps() const {
    return (_rps_filt * RAD_UMFANG_MM) / 1000.0f;
}

// ------------------------------------------------------------
float SpeedWeg::rpm() const {
    return _rps_filt * 60.0f;
}

float SpeedWeg::weg_mm() const {
    return _weg_mm_total;
}

float SpeedWeg::weg_abs_mm() const {
    return _weg_abs_mm_total;
}
