// ============================================================
// File: SpeedWeg.h
// Zweck:
//  - Geschwindigkeit (m/s) aus Encoder-Ticks
//  - Encoder-Gesamtcounts fuer Odometrie/Plot
//  - Tiefpass-Filter fuer stabile v_Ist-Werte
//  - Zeitbasis: Millisekunden (ms)
// ============================================================

#pragma once

#include <Arduino.h>
#include "src/globals.h"

class Enc;

class SpeedWeg {
public:
    explicit SpeedWeg(Enc& enc);

    void reset();

    // now_ms = millis()
    void update(uint32_t now_ms);

    float mps() const;       // m/s

    long counts_total() const { return _counts_total; }

private:
    void updateFromTicks(int16_t dcounts, uint32_t now_ms);
    void timeoutCheck(uint32_t now_ms);

private:
    Enc& _enc;

    // Encoder-Zaehler
    long _last_counts = 0;
    long _counts_total = 0;

    // Tick-Sammelpuffer fuer das Geschwindigkeitsfenster
    int32_t _acc_counts = 0;

    // Filter & Zeit
    float    _rps_filt = 0.0f;
    uint32_t _last_time_ms = 0;    // Zeitpunkt der letzten Geschwindigkeitsmessung
    uint32_t _last_tick_ms = 0;    // Zeitpunkt des letzten Ticks fuer Timeout
};