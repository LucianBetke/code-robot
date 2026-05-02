// Hardware.cpp
#include <Arduino.h>

#include "src/globals.h"
#include "src/Encoder.h"
#include "src/Motor.h"
#include "src/Hardware.h"

#include "board_config.h"

Enc enc[WHEEL_COUNT] =
{
    Enc(BoardPins::ENC_Li_PIN_A, BoardPins::ENC_Li_PIN_B),
    Enc(BoardPins::ENC_Re_PIN_A, BoardPins::ENC_Re_PIN_B)
};

Motor motor[WHEEL_COUNT] =
{
    Motor(BoardPins::M_Li_BIN1, BoardPins::M_Li_BIN2, enc[Li]),
    Motor(BoardPins::M_Re_AIN1, BoardPins::M_Re_AIN2, enc[Re])
};

void hardware_begin(bool resetEnc)
{
    pinMode(BoardPins::STBY_PIN, OUTPUT);
    digitalWrite(BoardPins::STBY_PIN, LOW);

    motor[Li].init();
    motor[Re].init();

    enc[Li].begin(resetEnc);
    enc[Re].begin(resetEnc);
}

void hardware_enableMotors()
{
    digitalWrite(BoardPins::STBY_PIN, HIGH);
}

void hardware_disableMotors()
{
    digitalWrite(BoardPins::STBY_PIN, LOW);
}