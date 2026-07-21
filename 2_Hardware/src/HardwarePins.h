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

    uint8_t stby_sync;
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

    constexpr uint8_t STBY_SYNC_PIN = 8;

    constexpr uint8_t ENC_Li_PIN_A = A2;
    constexpr uint8_t ENC_Li_PIN_B = A3;

    constexpr uint8_t ENC_Re_PIN_A = A0;
    constexpr uint8_t ENC_Re_PIN_B = A1;

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

        STBY_SYNC_PIN
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

    constexpr uint8_t STBY_SYNC_PIN = 8;

    constexpr uint8_t ENC_Re_PIN_A = A2;
    constexpr uint8_t ENC_Re_PIN_B = A3;

    constexpr uint8_t ENC_Li_PIN_A = A0;
    constexpr uint8_t ENC_Li_PIN_B = A1;

    // Drei HC-SR04 am hinteren Nano.
    constexpr uint8_t US_FRONT_TRIGGER_PIN = 11;
    constexpr uint8_t US_FRONT_ECHO_PIN = 3;

    constexpr uint8_t US_LEFT_TRIGGER_PIN = 12;
    constexpr uint8_t US_LEFT_ECHO_PIN = A5;

    constexpr uint8_t US_RIGHT_TRIGGER_PIN = 13;
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

        STBY_SYNC_PIN
    };
}

#endif // HARDWARE_PINS_H