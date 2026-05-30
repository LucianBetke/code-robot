// ============================================================
// File: WheelValues.h
// Zweck:
//  - Gemeinsame Fahrzeug-Radwert-Pakete
//  - Feste Radreihenfolge kommt aus RobotConfig.h:
//      VoRe = 0
//      VoLi = 1
//      HiLi = 2
//      HiRe = 3
//  - KEINE eigene Radnummerierung in dieser Datei!
// ============================================================

#ifndef WHEEL_VALUES_H
#define WHEEL_VALUES_H

#include <Arduino.h>
#include "src/RobotConfig.h"

// ============================================================
// Radgeschwindigkeiten des Gesamtfahrzeugs
//
// Einheit:
//   cm/s
//
// Index:
//   v[VoRe] = vorne rechts
//   v[VoLi] = vorne links
//   v[HiLi] = hinten links
//   v[HiRe] = hinten rechts
// ============================================================

struct WheelSpeedCms
{
    float v[WHEEL_VEHICLE_COUNT];
};

// ============================================================
// Encoder-Gesamtcounts des Gesamtfahrzeugs
//
// Einheit:
//   Encoder-Counts
//
// Index:
//   v[VoRe] = vorne rechts
//   v[VoLi] = vorne links
//   v[HiLi] = hinten links
//   v[HiRe] = hinten rechts
// ============================================================

struct WheelCounts
{
    long v[WHEEL_VEHICLE_COUNT];
};

#endif // WHEEL_VALUES_H