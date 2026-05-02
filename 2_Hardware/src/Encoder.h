// ============================================================
/* File: Encoder.h
 * Zweck:
 *  - Quadratur-Encoder (Port C / PCINT1: A0..A5) zählen
 *  - Atomarer Zugriff auf Zählerstand
 *  - Dispatcher ruft handleIsr() mit PINC-Snapshot
 * Abhängigkeiten: Arduino.h, <avr/io.h>, <avr/interrupt.h>
 */
 // ============================================================
#pragma once
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

class Enc {
public:
    Enc();                                              // leerer Konstruktor
    Enc(uint8_t pinA, uint8_t pinB);                    // bleibt bis _alt-Migration

    void begin(uint8_t pinA, uint8_t pinB,              // NEU: Pins als Parameter
        bool resetCounts = true);
    void begin(bool resetCounts = true);                // bleibt (alter Aufruf)

    void    resetCounts();
    int32_t getCounts() const;
    void    setCounts(int32_t v);
    void    handleIsr(uint8_t pinc_snapshot);

private:
    static const int8_t s_qlut[4][4];

    uint8_t  _pinA;
    uint8_t  _pinB;
    uint8_t  _bitA;
    uint8_t  _bitB;
    uint8_t  _maskA;
    uint8_t  _maskB;
    volatile int32_t _counts;
    volatile uint8_t _prevState;

    void enablePcintGroup1_SelectedPins();
};

namespace PCINT1_Dispatcher {
    void add(Enc* enc);
    void isr(uint8_t pinc_snapshot);
}