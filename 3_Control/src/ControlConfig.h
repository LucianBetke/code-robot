// ============================================================
// File: ControlConfig.h
// Zweck:
//  - Datentyp ControlConfig fuer PI-Parameter und DeadPWM
//  - Vordefinierte Konfigurationen fuer vorne und hinten
//
// Wichtig:
//  - Radgeschwindigkeiten werden jetzt in cm/s geregelt.
//  - Alte PI-Werte aus m/s wurden durch 100 geteilt.
// ============================================================

#ifndef CONTROLCONFIG_H
#define CONTROLCONFIG_H

#include <Arduino.h>
#include "src/globals.h"
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

#endif // CONTROLCONFIG_H