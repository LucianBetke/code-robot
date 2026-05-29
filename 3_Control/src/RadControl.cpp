// ============================================================
// File: RadControl.cpp (3_Control)
// Zweck:
//  - Gemeinsame Radregelungs-Implementierung fuer vorne und hinten
//  - Konfiguration kommt von der App via radControl_begin(cfg)
// ============================================================

#include "RadControl.h"
#include "src/Encoder.h"   // Klasse Enc (aus 2_Hardware)
#include "src/Motor.h"     // Klasse Motor (aus 2_Hardware)

// ============================================================
// enc[] und motor[] werden in der App definiert
// (vorne/src/Hardware.cpp bzw. hinten/src/Hardware.cpp).
// Hier nur als extern bekannt machen.
// ============================================================

extern Enc enc[WHEEL_COUNT];
extern Motor motor[WHEEL_COUNT];

// ============================================================
// Globale Objekte
//
// Die Regler werden hier mit Default-Parametern (0.0f, 0.0f)
// angelegt. Die echten Parameter setzt radControl_begin() spaeter
// per setParams() / setDeadPwm().
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
// Wird von der App im setup() aufgerufen mit der gewuenschten
// Konfiguration. Setzt PI-Parameter und DeadPWM in Reglern und
// Raedern.
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
// ============================================================

void wheelMeasurement_reset_all()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        wheelMeasurements[i].reset();
    }
}

// ============================================================
// control_resetPiStates
//
// Setzt nur die inneren PI-Zustaende zurueck:
//  - Integralanteil
//  - vorherige PWM
//
// Wichtig:
//  - Sollwerte bleiben erhalten.
//  - Motoren werden nicht gestoppt.
//  - WheelMeasurement wird nicht zurueckgesetzt.
//  - Kein kuenstlicher Null-Sollwert zwischen zwei Fahrabschnitten.
// ============================================================

void radControl_resetPiStates()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        regler[i].reset();
    }
}

void radControl_update(uint32_t now)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        rad[i].update(now);
    }
}

void radControl_setSoll(uint8_t w, float v)
{
    if (w < WHEEL_COUNT)
    {
        rad[w].setSoll(v);
    }
}

void radControl_stopAll()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        rad[i].stop();
    }
}