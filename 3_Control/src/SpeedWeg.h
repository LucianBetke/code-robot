// ============================================================
// File: SpeedWeg.h
// Zweck:
//  - Geschwindigkeit in cm/s aus Encoder-Ticks
//  - Encoder-Gesamtcounts fuer Odometrie/Plot
//  - Tiefpass-Filter fuer stabile v_Ist-Werte
//  - Zeitbasis: Millisekunden
// ============================================================

#pragma once

#include <Arduino.h>
#include "src/globals.h"

class Enc;

class SpeedWeg
{
public:
    explicit SpeedWeg(Enc& enc);

    void reset();

    void update(uint32_t now_ms);

    float cms() const;
    int16_t cmsInt() const;

    long counts_total() const { return _counts_total; }

private:
    void updateFromTicks(int16_t dcounts, uint32_t now_ms);
    void timeoutCheck(uint32_t now_ms);

private:
    Enc& _enc;

    long _last_counts = 0;
    long _counts_total = 0;

    int32_t _acc_counts = 0;

    float    _rps_filt = 0.0f;
    uint32_t _last_time_ms = 0;
    uint32_t _last_tick_ms = 0;
};