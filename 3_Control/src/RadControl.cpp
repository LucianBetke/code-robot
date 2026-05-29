// ============================================================
// File: Control.cpp (3_Control)
// Zweck:
//  - Gemeinsame Steuerungs-Implementierung fuer vorne und hinten
//  - Konfiguration kommt von der App via control_begin(cfg)
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
// angelegt. Die echten Parameter setzt control_begin() spaeter
// per setParams() / setDeadPwm().
// ============================================================

SpeedWeg speed[WHEEL_COUNT] =
{
    SpeedWeg(enc[Li]),
    SpeedWeg(enc[Re])
};

PIRegler regler[WHEEL_COUNT] =
{
    PIRegler(0.0f, 0.0f, -MAX_PWM, MAX_PWM, SLEW_LIMIT_PWM),
    PIRegler(0.0f, 0.0f, -MAX_PWM, MAX_PWM, SLEW_LIMIT_PWM)
};

Rad rad[WHEEL_COUNT] =
{
    Rad(motor[Li], speed[Li], regler[Li], RAD_REGEL_DT_MS, 0),
    Rad(motor[Re], speed[Re], regler[Re], RAD_REGEL_DT_MS, 0)
};

// ============================================================
// control_begin
//
// Wird von der App im setup() aufgerufen mit der gewuenschten
// Konfiguration. Setzt PI-Parameter und DeadPWM in Reglern und
// Raedern.
// ============================================================

void control_begin(const RadControlConfig& cfg)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        regler[i].setParams(cfg.pi[i].Kp, cfg.pi[i].Ki);
        rad[i].setDeadPwm(cfg.deadPwm[i]);
    }
}

// ============================================================
// Bestehende Funktionen
// ============================================================

void speed_reset_all()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        speed[i].reset();
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
//  - SpeedWeg wird nicht zurueckgesetzt.
//  - Kein kuenstlicher Null-Sollwert zwischen zwei Fahrabschnitten.
// ============================================================

void control_resetPiStates()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        regler[i].reset();
    }
}

void control_update(uint32_t now)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        rad[i].update(now);
    }
}

void control_setSoll(uint8_t w, float v)
{
    if (w < WHEEL_COUNT)
    {
        rad[w].setSoll(v);
    }
}

void control_stopAll()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        rad[i].stop();
    }
}