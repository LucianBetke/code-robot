// ============================================================
// File: RadControl.cpp
// Zweck:
//  - Gemeinsame Radregelungs-Implementierung fuer vorne und hinten
//  - Konfiguration kommt von der App via radControl_begin(cfg)
// ============================================================

#include "RadControl.h"

#include "src/Encoder.h"
#include "src/Motor.h"

// ============================================================
// enc[] und motor[] werden im Hardware-Layer definiert.
// Hier werden sie nur bekannt gemacht.
// ============================================================

extern Enc enc[WHEEL_COUNT];
extern Motor motor[WHEEL_COUNT];

// ============================================================
// Globale Objekte
//
// Die Regler werden hier mit Default-Parametern angelegt.
// Die echten Parameter setzt radControl_begin() spaeter
// per setParams() und setDeadPwm().
// ============================================================

WheelMeasurement wheelMeasurements[WHEEL_COUNT] =
{
    WheelMeasurement(enc[Li]),
    WheelMeasurement(enc[Re])
};

PIRegler regler[WHEEL_COUNT] =
{
    PIRegler(0.0f, 0.0f, -MAX_PWM, MAX_PWM, SLEW_LIMIT_PWM),
    PIRegler(0.0f, 0.0f, -MAX_PWM, MAX_PWM, SLEW_LIMIT_PWM)
};

Rad rad[WHEEL_COUNT] =
{
    Rad(motor[Li], wheelMeasurements[Li], regler[Li], RAD_REGEL_DT_MS, 0),
    Rad(motor[Re], wheelMeasurements[Re], regler[Re], RAD_REGEL_DT_MS, 0)
};

// ============================================================
// radControl_begin
//
// Setzt PI-Parameter und DeadPWM fuer beide lokalen Raeder.
// ============================================================

void radControl_begin(const RadControlConfig& cfg)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        regler[i].setParams(cfg.pi[i].Kp, cfg.pi[i].Ki);
        rad[i].setDeadPwm(cfg.deadPwm[i]);
    }
}

// ============================================================
// wheelMeasurement_reset_all
//
// Setzt nur die Radmessung zurueck.
// Encoder-Rohzaehler werden dabei nicht zwingend geloescht;
// WheelMeasurement setzt seinen internen Bezug neu.
// ============================================================

void wheelMeasurement_reset_all()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        wheelMeasurements[i].reset();
    }
}

// ============================================================
// radControl_resetPiStates
//
// Setzt nur die inneren PI-Zustaende zurueck:
//  - Integralanteil
//  - vorherige PWM
//  - Debugwerte
//
// Wichtig:
//  - Sollwerte bleiben erhalten.
//  - Motoren werden nicht gestoppt.
//  - WheelMeasurement wird nicht zurueckgesetzt.
//  - Kein kuenstlicher Null-Sollwert zwischen zwei Fahrbefehlen.
// ============================================================

void radControl_resetPiStates()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        regler[i].reset();
    }
}

// ============================================================
// radControl_update
// ============================================================

void radControl_update(uint32_t nowMs)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        rad[i].update(nowMs);
    }
}

// ============================================================
// radControl_setSoll
// ============================================================

void radControl_setSoll(uint8_t wheel, float v_cms)
{
    if (wheel < WHEEL_COUNT)
    {
        rad[wheel].setSoll(v_cms);
    }
}

// ============================================================
// radControl_stopAll
// ============================================================

void radControl_stopAll()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        rad[i].stop();
    }
}