// hardware_pins.h
#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H
#include <Arduino.h>
// ============================================================
// HardwarePinSet — fasst alle Pins für hardware_begin() zusammen
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
    uint8_t stby;
};
// ============================================================
// namespace PinsFront
// ============================================================
namespace PinsFront
{
    constexpr uint8_t M_Li_BIN1 = 9;   // D9  BIN1 → physisch LINKS
    constexpr uint8_t M_Li_BIN2 = 10;  // D10 BIN2 → physisch LINKS
    constexpr uint8_t M_Re_AIN1 = 6;   // D6  AIN1 → physisch RECHTS
    constexpr uint8_t M_Re_AIN2 = 5;   // D5  AIN2 → physisch RECHTS
    constexpr uint8_t STBY_PIN = 8;

    constexpr uint8_t ENC_Li_PIN_A = A2;  // A2 → Encoder LINKS,  Kanal A
    constexpr uint8_t ENC_Li_PIN_B = A3;  // A3 → Encoder LINKS,  Kanal B
    constexpr uint8_t ENC_Re_PIN_A = A0;  // A0 → Encoder RECHTS, Kanal A
    constexpr uint8_t ENC_Re_PIN_B = A1;  // A1 → Encoder RECHTS, Kanal B

    constexpr HardwarePinSet PINS = {
        ENC_Li_PIN_A, ENC_Li_PIN_B,
        ENC_Re_PIN_A, ENC_Re_PIN_B,
        M_Li_BIN1,    M_Li_BIN2,
        M_Re_AIN1,    M_Re_AIN2,
        STBY_PIN
    };
}
// ============================================================
// namespace PinsRear
// ============================================================
namespace PinsRear
{
    constexpr uint8_t M_Re_AIN1 = 9;
    constexpr uint8_t M_Re_AIN2 = 10;
    constexpr uint8_t M_Li_BIN1 = 6;
    constexpr uint8_t M_Li_BIN2 = 5;
    constexpr uint8_t STBY_PIN = 8;

    constexpr uint8_t ENC_Re_PIN_A = A2;
    constexpr uint8_t ENC_Re_PIN_B = A3;
    constexpr uint8_t ENC_Li_PIN_A = A0;
    constexpr uint8_t ENC_Li_PIN_B = A1;

    constexpr HardwarePinSet PINS = {
        ENC_Li_PIN_A, ENC_Li_PIN_B,
        ENC_Re_PIN_A, ENC_Re_PIN_B,
        M_Li_BIN1,    M_Li_BIN2,
        M_Re_AIN1,    M_Re_AIN2,
        STBY_PIN
    };
}
#endif // HARDWARE_PINS_H