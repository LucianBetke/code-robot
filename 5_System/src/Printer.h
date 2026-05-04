// ============================================================
// Printer.h
// ============================================================
#ifndef PRINTER_H
#define PRINTER_H

#include <Arduino.h>
#include "src/globals.h"
#include "src/VehicleController.h"
#include "src/Control.h"

class Printer
{
public:
    void printHeader();
    void printWheels(VehicleController& vehicle,
        float v2_ist, float v3_ist,
        int16_t pwm2, int16_t pwm3,
        uint32_t t_ms);
};

#endif