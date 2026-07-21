// ============================================================
// File: UltrasonicEchoCapture.cpp
// ============================================================

#include "UltrasonicEchoCapture.h"

#include <avr/interrupt.h>
#include <avr/io.h>

UltrasonicEchoCapture* UltrasonicEchoCapture::s_instance = nullptr;

UltrasonicEchoCapture::UltrasonicEchoCapture()
    : _frontInputRegister(nullptr),
    _frontEchoMask(0),
    _leftEchoMask(0),
    _rightEchoMask(0),
    _sideEchoMask(0),
    _previousSideEchoBits(0),
    _activeSideEchoMask(0),
    _activeChannel(UltrasonicEchoChannel::None),
    _riseUs(0),
    _fallUs(0),
    _riseSeen(false),
    _complete(false)
{}

void UltrasonicEchoCapture::begin(
    uint8_t frontEchoPin,
    uint8_t leftEchoPin,
    uint8_t rightEchoPin)
{
    s_instance = this;

    pinMode(frontEchoPin, INPUT);
    pinMode(leftEchoPin, INPUT);
    pinMode(rightEchoPin, INPUT);

    const uint8_t frontPort = digitalPinToPort(frontEchoPin);
    _frontInputRegister = portInputRegister(frontPort);
    _frontEchoMask = digitalPinToBitMask(frontEchoPin);

    _leftEchoMask = digitalPinToBitMask(leftEchoPin);
    _rightEchoMask = digitalPinToBitMask(rightEchoPin);
    _sideEchoMask = uint8_t(_leftEchoMask | _rightEchoMask);

    const uint8_t savedSreg = SREG;
    cli();

    _previousSideEchoBits = uint8_t(PINC & _sideEchoMask);
    _activeSideEchoMask = 0;
    _activeChannel = UltrasonicEchoChannel::None;
    _riseUs = 0;
    _fallUs = 0;
    _riseSeen = false;
    _complete = false;

    SREG = savedSreg;

    // Ein eventuell bereits gesetztes Pending-Flag loeschen,
    // bevor A4/A5 zur bestehenden PCINT1-Maske hinzukommen.
    // PCIFR-Flags werden durch Schreiben einer Eins geloescht. Direkte
    // Zuweisung statt |=, damit nicht versehentlich andere gerade
    // gesetzte PCIF-Flags (PCIF0/PCIF2) mit zurueckgeschrieben und
    // dadurch geloescht werden.
    PCIFR = _BV(PCIF1);
    PCMSK1 |= _sideEchoMask;

    // D3 = INT1. D2/INT0 bleibt unveraendert der Sync-Eingang.
    attachInterrupt(
        digitalPinToInterrupt(frontEchoPin),
        UltrasonicEchoCapture::isrFrontEcho,
        CHANGE
    );
}

void UltrasonicEchoCapture::arm(UltrasonicEchoChannel channel)
{
    uint8_t sideMask = 0;

    if (channel == UltrasonicEchoChannel::Left)
    {
        sideMask = _leftEchoMask;
    }
    else if (channel == UltrasonicEchoChannel::Right)
    {
        sideMask = _rightEchoMask;
    }

    const uint8_t savedSreg = SREG;
    cli();

    // Definierte Vergleichsbasis fuer die naechste A4/A5-Flanke.
    _previousSideEchoBits = uint8_t(PINC & _sideEchoMask);

    _activeChannel = channel;
    _activeSideEchoMask = sideMask;
    _riseUs = 0;
    _fallUs = 0;
    _riseSeen = false;
    _complete = false;

    SREG = savedSreg;
}

void UltrasonicEchoCapture::cancel()
{
    const uint8_t savedSreg = SREG;
    cli();

    _activeChannel = UltrasonicEchoChannel::None;
    _activeSideEchoMask = 0;
    _riseSeen = false;
    _complete = false;

    SREG = savedSreg;
}

bool UltrasonicEchoCapture::takeCompleted(EchoCaptureResult& result)
{
    const uint8_t savedSreg = SREG;
    cli();

    if (!_complete)
    {
        SREG = savedSreg;
        return false;
    }

    result.riseUs = _riseUs;
    result.fallUs = _fallUs;

    _activeChannel = UltrasonicEchoChannel::None;
    _activeSideEchoMask = 0;
    _riseSeen = false;
    _complete = false;

    SREG = savedSreg;
    return true;
}

bool UltrasonicEchoCapture::frontEchoLevel() const
{
    if (_frontInputRegister == nullptr)
    {
        return false;
    }

    return (*_frontInputRegister & _frontEchoMask) != 0;
}

void UltrasonicEchoCapture::isrFrontEcho()
{
    if (s_instance != nullptr)
    {
        s_instance->handleFrontEchoFromIsr();
    }
}

void UltrasonicEchoCapture::isrPcint1(uint8_t pincSnapshot)
{
    if (s_instance != nullptr)
    {
        s_instance->handlePcint1FromIsr(pincSnapshot);
    }
}

void UltrasonicEchoCapture::handleFrontEchoFromIsr()
{
    if (_activeChannel != UltrasonicEchoChannel::Front || _complete)
    {
        return;
    }

    const bool highLevel =
        (_frontInputRegister != nullptr) &&
        ((*_frontInputRegister & _frontEchoMask) != 0);

    captureEdgeFromIsr(highLevel);
}

void UltrasonicEchoCapture::handlePcint1FromIsr(uint8_t pincSnapshot)
{
    const uint8_t currentEchoBits =
        uint8_t(pincSnapshot & _sideEchoMask);

    const uint8_t changed =
        uint8_t(currentEchoBits ^ _previousSideEchoBits);

    _previousSideEchoBits = currentEchoBits;

    // Dieser Pfad wird auch bei jeder Encoderflanke aufgerufen.
    // Deshalb vor micros() so frueh wie moeglich abbrechen.
    const uint8_t relevant =
        uint8_t(changed & _activeSideEchoMask);

    if (relevant == 0 || _complete)
    {
        return;
    }

    const bool highLevel =
        (currentEchoBits & _activeSideEchoMask) != 0;

    captureEdgeFromIsr(highLevel);
}

void UltrasonicEchoCapture::captureEdgeFromIsr(bool highLevel)
{
    const uint32_t nowUs = micros();

    if (highLevel)
    {
        _riseUs = nowUs;
        _riseSeen = true;
        return;
    }

    if (_riseSeen)
    {
        _fallUs = nowUs;
        _complete = true;
    }
}