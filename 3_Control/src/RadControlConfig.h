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

struct RadControlConfig
{
    int16_t deadPwm[WHEEL_COUNT];
    PIParam pi[WHEEL_COUNT];
};

namespace ConfigFront
{
    constexpr RadControlConfig CONFIG =
    {
        // deadPwm: Li, Re
        { 55, 70 },

        // PI-Parameter: Li, Re
        // Einheit Fehler: cm/s
        {
            { 0.015f, 0.060f },   // Li = VoLi
            { 0.020f, 0.100f }    // Re: vorher 2.0 / 10.0 bei m/s
        }
    };
}

namespace ConfigRear
{
    constexpr RadControlConfig CONFIG =
    {
        // deadPwm: Li, Re
        { 80, 50 },

        // PI-Parameter: Li, Re
        // Einheit Fehler: cm/s
        {
            { 0.020f, 0.060f },   // Li: vorher 2.0 / 6.0 bei m/s
            { 0.015f, 0.055f }   // Re = HiRe etwas sanfter
        }
    };
}

#endif // RAD_CONTROL_CONFIG_H