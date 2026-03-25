// hardware_rear_pins.h
#pragma once
#include <Arduino.h>

// ==========================
// Motor hinten
// ==========================
constexpr uint8_t M_Re_AIN1 = 9;
constexpr uint8_t M_Re_AIN2 = 10;

constexpr uint8_t M_Li_BIN1 = 6;
constexpr uint8_t M_Li_BIN2 = 5;

constexpr uint8_t STBY_PIN = 8;

// ==========================
// Encoder hinten
// ==========================
constexpr uint8_t ENC_Re_PIN_A = A2;
constexpr uint8_t ENC_Re_PIN_B = A3;

constexpr uint8_t ENC_Li_PIN_A = A0;
constexpr uint8_t ENC_Li_PIN_B = A1;
