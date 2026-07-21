// ============================================================
// File: Encoder.cpp
// Zweck:
//  - Quadratur-Encoder auf Port C / PCINT1 auswerten
//  - Dispatcher verteilt den PINC-Snapshot an alle Encoder
// ============================================================

#include "Encoder.h"

// Quadratur-LUT:
//
// prev \ curr: 00  01  10  11
//
// Zustand:
//   Bit 0 = A
//   Bit 1 = B
//
// Ergebnis:
//   +1 = Schritt vorwärts
//   -1 = Schritt rückwärts
//    0 = kein gültiger Schritt / kein Wechsel / ungültiger Sprung
const int8_t Enc::s_qlut[4][4] =
{
    {  0, +1, -1,  0 },
    { -1,  0,  0, +1 },
    { +1,  0,  0, -1 },
    {  0, -1, +1,  0 }
};

Enc::Enc()
    : _pinA(0),
    _pinB(0),
    _bitA(0),
    _bitB(0),
    _maskA(0),
    _maskB(0),
    _counts(0),
    _prevState(0)
{}

void Enc::begin(uint8_t pinA, uint8_t pinB, bool doResetCounts)
{
    _pinA = pinA;
    _pinB = pinB;

    _bitA = uint8_t(_pinA - A0);
    _bitB = uint8_t(_pinB - A0);

    _maskA = uint8_t(1u << _bitA);
    _maskB = uint8_t(1u << _bitB);

    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    const uint8_t pinc = PINC;

    const uint8_t a = (pinc & _maskA) ? 1 : 0;
    const uint8_t b = (pinc & _maskB) ? 1 : 0;

    _prevState = uint8_t((b << 1) | a);

    if (doResetCounts)
    {
        this->resetCounts();
    }

    PCIFR |= _BV(PCIF1);

    PCINT1_Dispatcher::add(this);

    enablePcintGroup1_SelectedPins();
}

void Enc::resetCounts()
{
    setCounts(0);
}

int32_t Enc::getCounts() const
{
    int32_t v;

    const uint8_t s = SREG;
    noInterrupts();

    v = _counts;

    SREG = s;

    return v;
}

void Enc::setCounts(int32_t v)
{
    const uint8_t s = SREG;
    noInterrupts();

    _counts = v;

    SREG = s;
}

void Enc::handleIsr(uint8_t pinc_snapshot)
{
    const uint8_t a = (pinc_snapshot & _maskA) ? 1 : 0;
    const uint8_t b = (pinc_snapshot & _maskB) ? 1 : 0;

    const uint8_t curr = uint8_t((b << 1) | a);

    const int8_t d = s_qlut[_prevState][curr];

    _prevState = curr;

    if (d != 0)
    {
        _counts += d;
    }
}

void Enc::enablePcintGroup1_SelectedPins()
{
    PCICR |= _BV(PCIE1);
    PCMSK1 |= (_maskA | _maskB);
}

// ============================================================
// PCINT1_Dispatcher
// ============================================================

namespace
{
    Enc* s_list[2] = { nullptr, nullptr };
    uint8_t s_count = 0;
}

namespace PCINT1_Dispatcher
{
    void add(Enc* enc)
    {
        if (s_count < 2)
        {
            s_list[s_count] = enc;
            s_count++;
        }
    }

    void isr(uint8_t pinc_snapshot)
    {
        for (uint8_t i = 0; i < s_count; i++)
        {
            if (s_list[i] != nullptr)
            {
                s_list[i]->handleIsr(pinc_snapshot);
            }
        }
    }
}