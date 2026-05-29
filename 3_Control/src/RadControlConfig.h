// ============================================================
// File: RadControlConfig.h
// Zweck:
//  - Datentyp RadControlConfig fuer PI-Parameter und DeadPWM
//  - Vordefinierte Konfigurationen fuer vorne und hinten
//
// Wichtig:
//  - Radgeschwindigkeiten werden jetzt in cm/s geregelt.
//  - Alte PI-Werte aus m/s wurden durch 100 geteilt.
// ============================================================

#ifndef RAD_CONTROL_CONFIG_H
#define RAD_CONTROL_CONFIG_H

#include <Arduino.h>
#include "src/RobotConfig.h"
#include "src/ControlTypes.h"

struct ControlConfig
{
    int16_t deadPwm[WHEEL_COUNT];
    PIParam pi[WHEEL_COUNT];
};

namespace ConfigFront
{
    constexpr ControlConfig CONFIG =
    {
        // deadPwm: Li, Re
        { 70, 70 },

        // PI-Parameter: Li, Re
        // Einheit Fehler: cm/s
        {
            { 0.020f, 0.080f },   // Li: vorher 2.0 / 8.0 bei m/s
            { 0.020f, 0.100f }    // Re: vorher 2.0 / 10.0 bei m/s
        }
    };
}

namespace ConfigRear
{
    constexpr ControlConfig CONFIG =
    {
        // deadPwm: Li, Re
        { 80, 60 },

        // PI-Parameter: Li, Re
        // Einheit Fehler: cm/s
        {
            { 0.020f, 0.060f },   // Li: vorher 2.0 / 6.0 bei m/s
            { 0.020f, 0.055f }    // Re: vorher 2.0 / 5.5 bei m/s
        }
    };
}

#endif // RAD_CONTROL_CONFIG_H