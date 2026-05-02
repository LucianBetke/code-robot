// Encoder.cpp
#include "Encoder.h"

// Quadratur-LUT: prev \ curr: 00 01 10 11  (A=LSB, B=MSB)
const int8_t Enc::s_qlut[4][4] = {
    {  0, +1, -1,  0 },
    { -1,  0,  0, +1 },
    { +1,  0,  0, -1 },
    {  0, -1, +1,  0 }
};

Enc::Enc()
    : _pinA(0), _pinB(0), _counts(0), _prevState(0)
    , _bitA(0), _bitB(0), _maskA(0), _maskB(0)
{
}

void Enc::begin(uint8_t pinA, uint8_t pinB, bool resetCounts)
{
    _pinA = pinA;
    _pinB = pinB;
    _bitA = uint8_t(_pinA - A0);
    _bitB = uint8_t(_pinB - A0);
    _maskA = uint8_t(1u << _bitA);
    _maskB = uint8_t(1u << _bitB);

    // ab hier identisch mit dem bisherigen begin():
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    uint8_t pinc = PINC;
    uint8_t a = (pinc & _maskA) ? 1 : 0;
    uint8_t b = (pinc & _maskB) ? 1 : 0;
    _prevState = (b << 1) | a;
    if (resetCounts) this->resetCounts();
    PCIFR |= _BV(PCIF1);
    PCINT1_Dispatcher::add(this);
    enablePcintGroup1_SelectedPins();
}

void Enc::resetCounts() { setCounts(0); }


int32_t Enc::getCounts() const
{
    int32_t v;
    uint8_t s = SREG; noInterrupts();
    v = _counts;
    SREG = s;
    return v;
}

void Enc::setCounts(int32_t v)
{
    uint8_t s = SREG; noInterrupts();
    _counts = v;
    SREG = s;
}

void Enc::handleIsr(uint8_t pinc_snapshot)
{
    // A/B-Bits aus dem EINMAL gelesenen Snapshot ziehen
    uint8_t a = (pinc_snapshot & _maskA) ? 1 : 0;
    uint8_t b = (pinc_snapshot & _maskB) ? 1 : 0;
    uint8_t curr = (b << 1) | a;   // 0..3

    int8_t d = s_qlut[_prevState][curr];
    _prevState = curr;
    if (d) _counts += d;
}

void Enc::enablePcintGroup1_SelectedPins()
{
    // PCINT-Gruppe 1 (Port C) aktivieren und genau diese Pins maskieren
    PCICR |= _BV(PCIE1);
    PCMSK1 |= (_maskA | _maskB);
}

// ------------ Dispatcher-Implementierung ------------
namespace {
    Enc* s_list[2] = { nullptr, nullptr };
    uint8_t s_count = 0;
}

namespace PCINT1_Dispatcher {
    void add(Enc* enc)
    {
        if (s_count < 2) s_list[s_count++] = enc;
    }

    void isr(uint8_t pinc_snapshot)
    {
        for (uint8_t i = 0; i < s_count; ++i)
            if (s_list[i]) s_list[i]->handleIsr(pinc_snapshot);
    }
}

// ------------ ISR selbst (hier mit drin) ------------
ISR(PCINT1_vect)
{
    uint8_t pinc = PINC;                  // EIN Port-Read
    PCINT1_Dispatcher::isr(pinc);         // an alle Encoder verteilen
}
