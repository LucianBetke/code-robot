// ============================================================
// File: SpeedWeg.h
// Zweck: Geschwindigkeit (m/s) & Weg aus Encoder-Ticks
//        inkl. Tiefpass-Filter für stabile v_Ist-Werte
//        -> Zeitbasis: Millisekunden (ms)
// ============================================================

#pragma once

#include <Arduino.h>
#include "src/globals.h"

class Enc;

class SpeedWeg {
public:
    explicit SpeedWeg(Enc& enc);

    void reset();
    void setAlpha(float a);
    void setTimeoutMs(uint16_t ms);

    // now_ms = millis()
    void update(uint32_t now_ms);

    float mps() const;       // m/s
    float rpm() const;       // U/min
    float weg_mm() const;    // mm (signed)
    float weg_abs_mm() const;

    float weg_mm_total() const { return _weg_mm_total; }
    float weg_abs_mm_total() const { return _weg_abs_mm_total; }
    long  counts_total() const { return _counts_total; }

private:
    void updateFromTicks(int16_t dcounts, uint32_t now_ms);
    void timeoutCheck(uint32_t now_ms);

private:
    Enc& _enc;

    // Weg & Zähler
    long  _last_counts = 0;
    long  _counts_total = 0;
    float _weg_mm_total = 0.0f;
    float _weg_abs_mm_total = 0.0f;

    // Tick-Sammelpuffer für das Geschwindigkeitsfenster
    int32_t _acc_counts = 0;

    // Filter & Zeit (in ms)
    float    _rps_filt = 0.0f;
    float    _alpha = 0.2f;
    uint32_t _last_time_ms = 0;    // Zeitpunkt der letzten Geschwindigkeitsmessung
    uint32_t _timeout_ms = 60;   // Stillstand nach 60 ms
    uint32_t _last_tick_ms = 0;    // Zeitpunkt des letzten Ticks (für Timeout)
};
