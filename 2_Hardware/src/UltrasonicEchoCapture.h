// ============================================================
// File: UltrasonicEchoCapture.h
// Zweck:
//  - Echo-Flanken der HC-SR04 erfassen
//  - Front-Echo ueber einen Hardware-Interrupt (INT1 / D3)
//  - Links/Rechts ueber den gemeinsamen PCINT1-Vektor (PORTC)
//  - Zeitstempel nur in der ISR erfassen
//  - Fertige Rise/Fall-Paare atomar an das Hauptprogramm uebergeben
//
// Die Sensoren sind auf beide Nanos verteilt: vorne die beiden
// seitlichen, hinten der vordere. Nicht vorhandene Kanaele
// werden mit ULTRASONIC_NO_PIN uebergeben und bleiben inaktiv.
// ============================================================

#ifndef ULTRASONIC_ECHO_CAPTURE_H
#define ULTRASONIC_ECHO_CAPTURE_H

#include <Arduino.h>

// Kennzeichnet einen auf diesem Nano nicht vorhandenen Sensor.
static const uint8_t ULTRASONIC_NO_PIN = 255;

// Der Messmanager aktiviert immer genau einen Kanal.
enum class UltrasonicEchoChannel : uint8_t
{
    None = 0,
    Front = 1,
    Left = 2,
    Right = 3
};

struct EchoCaptureResult
{
    uint32_t riseUs;
    uint32_t fallUs;
};

class UltrasonicEchoCapture
{
public:
    UltrasonicEchoCapture();

    void begin(
        uint8_t frontEchoPin,
        uint8_t leftEchoPin,
        uint8_t rightEchoPin);

    void arm(UltrasonicEchoChannel channel);
    void cancel();

    bool takeCompleted(EchoCaptureResult& result);

    // Diagnose: aktueller Pegel des Front-Echopins (D3).
    // true  = Pin steht HIGH
    // false = Pin steht LOW
    bool frontEchoLevel() const;

    // Aufruf aus dem zentralen PCINT1-Router.
    static void isrPcint1(uint8_t pincSnapshot);

private:
    static UltrasonicEchoCapture* s_instance;

    static void isrFrontEcho();

    void handleFrontEchoFromIsr();
    void handlePcint1FromIsr(uint8_t pincSnapshot);
    void captureEdgeFromIsr(bool highLevel);

    volatile uint8_t* _frontInputRegister;
    uint8_t _frontEchoMask;

    uint8_t _leftEchoMask;
    uint8_t _rightEchoMask;
    uint8_t _sideEchoMask;

    volatile uint8_t _previousSideEchoBits;
    volatile uint8_t _activeSideEchoMask;
    volatile UltrasonicEchoChannel _activeChannel;

    volatile uint32_t _riseUs;
    volatile uint32_t _fallUs;
    volatile bool _riseSeen;
    volatile bool _complete;
};

#endif // ULTRASONIC_ECHO_CAPTURE_H