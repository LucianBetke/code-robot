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
        int16_t hiLi_i_cms,
        int16_t hiRe_i_cms,
        int16_t hiLi_pwm,
        int16_t hiRe_pwm);

    void printOdom(
        uint16_t cmdpId,
        uint32_t t_ms,
        const MecanumOdometer& odom);

#ifdef PRINTER_MODE_CHASSIS
    void printChassisDebug(
        VehicleController& vehicle,
        uint32_t t_ms,
        int16_t hiLi_send_cms,
        int16_t hiRe_send_cms);

    void printWheels(
        VehicleController& vehicle,
        int16_t v2_ist_cms,
        int16_t v3_ist_cms,
        uint32_t t_ms);

    void printFrame(
        VehicleController& vehicle,
        uint32_t t_ms,
        int16_t voLi_i_cms,
        int16_t voRe_i_cms,
        int16_t hiLi_i_cms,
        int16_t hiRe_i_cms);
#endif

#ifdef PRINTER_MODE_RAEDER
    void printWheels(
        VehicleController& vehicle,
        int16_t v2_ist_cms,
        int16_t v3_ist_cms,
        int16_t pwm2,
        int16_t pwm3,
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