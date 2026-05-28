// ============================================================
// File: ScaleUtils.h
// Zweck:
//  - Gemeinsame Rundungs- und Skalierfunktionen
//  - Vermeidet doppelte lokale Hilfsfunktionen in mehreren .cpp-Dateien
// ============================================================

#ifndef SCALE_UTILS_H
#define SCALE_UTILS_H

#include <stdint.h>

int16_t scaleRoundToInt16(float value);
long scaleFloatToInt100(float value);
long scaleFloatToInt1000(float value);

#endif // SCALE_UTILS_H