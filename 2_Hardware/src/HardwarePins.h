// ============================================================
// File: HardwarePins.h
// Zweck:
//  - Hardware-Pins fuer vorne und hinten zentral definieren
//  - Pin-Konfiguration als HardwarePinSet fuer hardware_begin()
// ============================================================

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

#include <Arduino.h>

// ============================================================
// HardwarePinSet
// ============================================================

struct HardwarePinSet
{
    uint8_t encLiA;
    uint8_t encLiB;

    uint8_t encReA;
    uint8_t encReB;

    uint8_t motorLi1;
    uint8_t motorLi2;

    uint8_t motorRe1;
    uint8_t motorRe2;

    uint8_t boardControlPin;
};

// ============================================================
// PinsFront
// ============================================================

namespace PinsFront
{
    constexpr uint8_t M_Li_BIN1 = 9;
    constexpr uint8_t M_Li_BIN2 = 10;

    constexpr uint8_t M_Re_AIN1 = 6;
    constexpr uint8_t M_Re_AIN2 = 5;

    // Sync-Ausgang zum hinteren Nano.
    // Verbindung: vorne D4 -> hinten D2.
    constexpr uint8_t SYNC_OUTPUT_PIN = 4;

    constexpr uint8_t ENC_Li_PIN_A = A2;
    constexpr uint8_t ENC_Li_PIN_B = A3;

    constexpr uint8_t ENC_Re_PIN_A = A0;
    constexpr uint8_t ENC_Re_PIN_B = A1;

    // Die beiden seitlichen HC-SR04 haengen am vorderen Nano.
    // Die Echos muessen auf PORTC liegen, weil sie ueber den
    // gemeinsamen PCINT1-Vektor erfasst werden.
    constexpr uint8_t US_LEFT_TRIGGER_PIN = 2;
    constexpr uint8_t US_LEFT_ECHO_PIN = A5;

    constexpr uint8_t US_RIGHT_TRIGGER_PIN = 3;
    constexpr uint8_t US_RIGHT_ECHO_PIN = A4;

    constexpr HardwarePinSet PINS =
    {
        ENC_Li_PIN_A,
        ENC_Li_PIN_B,

        ENC_Re_PIN_A,
        ENC_Re_PIN_B,

        M_Li_BIN1,
        M_Li_BIN2,

        M_Re_AIN1,
        M_Re_AIN2,

        SYNC_OUTPUT_PIN
    };
}

// ============================================================
// PinsRear
// ============================================================

namespace PinsRear
{
    constexpr uint8_t M_Re_AIN1 = 9;
    constexpr uint8_t M_Re_AIN2 = 10;

    constexpr uint8_t M_Li_BIN1 = 6;
    constexpr uint8_t M_Li_BIN2 = 5;

    // STBY-Ausgang fuer die beiden hinteren Motortreiber.
    constexpr uint8_t MOTOR_STBY_PIN = 8;

    constexpr uint8_t ENC_Re_PIN_A = A2;
    constexpr uint8_t ENC_Re_PIN_B = A3;

    constexpr uint8_t ENC_Li_PIN_A = A0;
    constexpr uint8_t ENC_Li_PIN_B = A1;

    // Nur noch der vordere HC-SR04 haengt am hinteren Nano.
    // Trigger: D4
    // Echo:    D3
    //
    // D11 ist dadurch frei.
    // D13 traegt die Verbindungs-LED.
    // A4 und A5 sind fuer die spaetere IMU vorgesehen.
    constexpr uint8_t US_FRONT_TRIGGER_PIN = 4;
    constexpr uint8_t US_FRONT_ECHO_PIN = 3;

    constexpr HardwarePinSet PINS =
    {
        ENC_Li_PIN_A,
        ENC_Li_PIN_B,

        ENC_Re_PIN_A,
        ENC_Re_PIN_B,

        M_Li_BIN1,
        M_Li_BIN2,

        M_Re_AIN1,
        M_Re_AIN2,

        MOTOR_STBY_PIN
    };
}

#endif // HARDWARE_PINS_H