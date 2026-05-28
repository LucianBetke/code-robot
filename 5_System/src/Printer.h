#ifndef PRINTER_H
#define PRINTER_H

#include <Arduino.h>
#include "src/globals.h"
#include "src/VehicleController.h"
#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/RearFrameClient.h"
#include "PrinterConfig.h"

class MecanumOdometer;

class Printer
{
public:
    void printInfo(VehicleController& vehicle, const ControlConfig& cfg);

    void printCompletedFrame(
        VehicleController& vehicle,
        const RearPendingFrame& frame,
        int16_t hiLi_i_cms,
        int16_t hiRe_i_cms,
        int16_t hiLi_pwm,
        int16_t hiRe_pwm);

    void printOdom2(
        uint16_t cmdpId,
        uint32_t t_ms,
        const MecanumOdometer& odom);

#ifdef PRINTER_MODE_CHASSIS
    void printWheels(VehicleController& vehicle,
        float v2_ist_cms, float v3_ist_cms,
        uint32_t t_ms);

    void printFrame(VehicleController& vehicle,
        uint32_t t_ms,
        int16_t voLi_i_cms,
        int16_t voRe_i_cms,
        int16_t hiLi_i_cms,
        int16_t hiRe_i_cms);
#endif

#ifdef PRINTER_MODE_RAEDER
    void printWheels(VehicleController& vehicle,
        float v2_ist_cms, float v3_ist_cms,
        int16_t pwm2, int16_t pwm3,
        uint32_t t_ms);

    void printFrame(
        uint32_t t_ms,
        int16_t voLi_s_cms,
        int16_t voLi_i_cms,
        int16_t voLi_pwm,
        int16_t voRe_s_cms,
        int16_t voRe_i_cms,
        int16_t voRe_pwm,
        int16_t hiLi_s_cms,
        int16_t hiLi_i_cms,
        int16_t hiLi_pwm,
        int16_t hiRe_s_cms,
        int16_t hiRe_i_cms,
        int16_t hiRe_pwm);
#endif
};

#endif