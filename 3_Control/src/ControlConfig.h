// ============================================================
// File: ControlConfig.h
// Zweck:
//  - Datentyp ControlConfig fuer PI-Parameter und DeadPWM
//  - Vordefinierte Konfigurationen fuer vorne und hinten
//
//  Apps (vorne, hinten) waehlen ihre Konfiguration aus und
//  uebergeben sie an control_begin().
//  3_Control selbst kennt vorne/hinten nicht.
// ============================================================

#ifndef CONTROLCONFIG_H
#define CONTROLCONFIG_H


#include <Arduino.h>
#include "src/globals.h"
#include "src/ControlTypes.h"   // PIParam

// ============================================================
// Datentyp: enthaelt alles, was Control zur Initialisierung
// braucht.
// ============================================================
struct ControlConfig
{
    int16_t deadPwm[WHEEL_COUNT];   // Mindest-PWM zum Anfahren
    PIParam pi[WHEEL_COUNT];        // Kp und Ki je Rad
};

// ============================================================
// namespace ConfigFront
//
// Der namespace ist nur eine "Schublade" mit einem Namen.
// Variablen darin spricht man mit "ConfigFront::CONFIG" an.
// Damit koennen ConfigFront und ConfigRear je ein eigenes
// CONFIG haben, ohne sich gegenseitig zu stoeren.
// ============================================================
namespace ConfigFront
{
    constexpr ControlConfig CONFIG =
    {
        // deadPwm: Li, Re
        { 80, 60 },

        // PI-Parameter: Li, Re
        {
            { 2.0f, 6.0f },   // Li: Kp, Ki
            { 2.0f, 5.5f }    // Re: Kp, Ki
        }
    };
}

// ============================================================
// namespace ConfigRear
//
// Aktuell identisch mit ConfigFront, aber als eigene
// Schublade -> spaeter unabhaengig anpassbar.
// ============================================================
namespace ConfigRear
{
    constexpr ControlConfig CONFIG =
    {
        // deadPwm: Li, Re
        { 80, 60 },

        // PI-Parameter: Li, Re
        {
            { 2.0f, 6.0f },   // Li: Kp, Ki
            { 2.0f, 5.5f }    // Re: Kp, Ki
        }
    };
}

#endif // CONTROLCONFIG_H
