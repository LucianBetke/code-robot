#ifndef TELEMETRY_PRINTER_H
#define TELEMETRY_PRINTER_H

#include <Arduino.h>
#include "src/RobotConfig.h"
#include "src/VehicleController.h"
#include "src/RadControl.h"
#include "src/RadControlConfig.h"
#include "src/RearFrameClient.h"
#include "TelemetryPrinterConfig.h"

class MecanumOdometer;

class TelemetryPrinter
{
public:
    void printInfo(VehicleController& vehicle, const RadControlConfig& cfg);

    void printCompletedFrame(
        VehicleController& vehicle,
        const RearPendingFrame& frame,
        float hiLi_i_cms,
        float hiRe_i_cms,
        int16_t hiLi_pwm,
        int16_t hiRe_pwm);

    void printOdom2(
        uint16_t cmdpId,
        uint32_t t_ms,
        const MecanumOdometer& odom);

#ifdef PRINTER_MODE_CHASSIS
    void printWheels(
        VehicleController& vehicle,
        float v2_ist_cms,
        float v3_ist_cms,
        uint32_t t_ms);

    void printFrame(
        VehicleController& vehicle,
        uint32_t t_ms,
        float voLi_i_cms,
        float voRe_i_cms,
        float hiLi_i_cms,
        float hiRe_i_cms);
#endif

#ifdef PRINTER_MODE_RAEDER
    void printWheels(
        VehicleController& vehicle,
        float v2_ist_cms,
        float v3_ist_cms,
        int16_t pwm2,
        int16_t pwm3,
        uint32_t t_ms);

    void printFrame(
        uint32_t t_ms,
        float voLi_s_cms,
        float voLi_i_cms,
        int16_t voLi_pwm,
        float voRe_s_cms,
        float voRe_i_cms,
        int16_t voRe_pwm,
        float hiLi_s_cms,
        float hiLi_i_cms,
        int16_t hiLi_pwm,
        float hiRe_s_cms,
        float hiRe_i_cms,
        int16_t hiRe_pwm);
#endif
};

#endif