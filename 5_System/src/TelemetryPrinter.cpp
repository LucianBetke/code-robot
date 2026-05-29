// ============================================================
// File: TelemetryPrinter.cpp
// ============================================================

#include "TelemetryPrinter.h"

#include "src/MecanumOdometer.h"
#include "src/ScaleUtils.h"

namespace
{
    void printSpeedCms(float valueCms)
    {
        Serial.print((int)scaleRoundToInt16(valueCms));
    }

    void printValue100(float value)
    {
        Serial.print(scaleFloatToInt100(value));
    }
}

void TelemetryPrinter::printInfo(VehicleController& vehicle, const RadControlConfig& cfg)
{
#if PRINTER_ENABLE_INFO
    Serial.print(F("#INFO,Raeder,Li,Kp100="));  Serial.print(scaleFloatToInt100(cfg.pi[Li].Kp));
    Serial.print(F(",Ki100="));                 Serial.print(scaleFloatToInt100(cfg.pi[Li].Ki));
    Serial.print(F(",dead="));                  Serial.print(cfg.deadPwm[Li]);
    Serial.print(F(",Re,Kp100="));              Serial.print(scaleFloatToInt100(cfg.pi[Re].Kp));
    Serial.print(F(",Ki100="));                 Serial.print(scaleFloatToInt100(cfg.pi[Re].Ki));
    Serial.print(F(",dead="));                  Serial.println(cfg.deadPwm[Re]);

    Serial.print(F("#INFO,Chassis,vx,Kp100=")); Serial.print(scaleFloatToInt100(vehicle.KpVx()));
    Serial.print(F(",Ki100="));                 Serial.print(scaleFloatToInt100(vehicle.KiVx()));
    Serial.print(F(",vy,Kp100="));              Serial.print(scaleFloatToInt100(vehicle.KpVy()));
    Serial.print(F(",Ki100="));                 Serial.print(scaleFloatToInt100(vehicle.KiVy()));
    Serial.print(F(",wz,Kp100="));              Serial.print(scaleFloatToInt100(vehicle.KpWz()));
    Serial.print(F(",Ki100="));                 Serial.println(scaleFloatToInt100(vehicle.KiWz()));
#else
    (void)vehicle;
    (void)cfg;
#endif
}

void TelemetryPrinter::printCompletedFrame(
    VehicleController& vehicle,
    const RearPendingFrame& frame,
    float hiLi_i_cms,
    float hiRe_i_cms,
    int16_t hiLi_pwm,
    int16_t hiRe_pwm)
{
#ifdef PRINTER_MODE_CHASSIS
    printFrame(
        vehicle,
        frame.t,
        speed[Li].cms(),
        speed[Re].cms(),
        hiLi_i_cms,
        hiRe_i_cms
    );
#endif

#ifdef PRINTER_MODE_RAEDER
    printFrame(
        frame.t,

        vehicle.getWheelSoll(VoLi),
        speed[Li].cms(),
        frame.voLi_pwm,

        vehicle.getWheelSoll(VoRe),
        speed[Re].cms(),
        frame.voRe_pwm,

        vehicle.getWheelSoll(HiLi),
        hiLi_i_cms,
        hiLi_pwm,

        vehicle.getWheelSoll(HiRe),
        hiRe_i_cms,
        hiRe_pwm
    );
#endif
}

void TelemetryPrinter::printOdom2(
    uint16_t cmdpId,
    uint32_t t_ms,
    const MecanumOdometer& odom)
{
#if PRINTER_ENABLE_ODOM
    if (cmdpId == 0)
    {
        return;
    }

    Serial.print(F("#ODOM2,"));
    Serial.print((unsigned int)cmdpId);              Serial.print(',');
    Serial.print((unsigned long)t_ms);               Serial.print(',');
    Serial.print(scaleFloatToInt100(odom.absCm()));  Serial.print(',');
    Serial.print(scaleFloatToInt100(odom.xCm()));    Serial.print(',');
    Serial.print(scaleFloatToInt100(odom.yCm()));    Serial.print(',');
    Serial.println(scaleFloatToInt100(odom.phiDeg()));
#else
    (void)cmdpId;
    (void)t_ms;
    (void)odom;
#endif
}

#ifdef PRINTER_MODE_CHASSIS

void TelemetryPrinter::printWheels(
    VehicleController& vehicle,
    float v2_ist_cms,
    float v3_ist_cms,
    uint32_t t_ms)
{
#if PRINTER_ENABLE_CHASSIS
    Serial.print(F("#CHASSIS,"));
    Serial.print(t_ms);                  Serial.print(',');

    printSpeedCms(speed[Li].cms());      Serial.print(',');
    printSpeedCms(speed[Re].cms());      Serial.print(',');
    printSpeedCms(v2_ist_cms);           Serial.print(',');
    printSpeedCms(v3_ist_cms);           Serial.print(',');

    printSpeedCms(vehicle.vxIst());      Serial.print(',');
    printSpeedCms(vehicle.vyIst());      Serial.print(',');
    printValue100(vehicle.wzIst());
    Serial.println();
#else
    (void)vehicle;
    (void)v2_ist_cms;
    (void)v3_ist_cms;
    (void)t_ms;
#endif
}

void Printer::printFrame(
    VehicleController& vehicle,
    uint32_t t_ms,
    float voLi_i_cms,
    float voRe_i_cms,
    float hiLi_i_cms,
    float hiRe_i_cms)
{
#if PRINTER_ENABLE_CHASSIS
    Serial.print(F("#CHASSIS,"));
    Serial.print(t_ms);                  Serial.print(',');

    printSpeedCms(voLi_i_cms);           Serial.print(',');
    printSpeedCms(voRe_i_cms);           Serial.print(',');
    printSpeedCms(hiLi_i_cms);           Serial.print(',');
    printSpeedCms(hiRe_i_cms);           Serial.print(',');

    printSpeedCms(vehicle.vxIst());      Serial.print(',');
    printSpeedCms(vehicle.vyIst());      Serial.print(',');
    printValue100(vehicle.wzIst());
    Serial.println();
#else
    (void)vehicle;
    (void)t_ms;
    (void)voLi_i_cms;
    (void)voRe_i_cms;
    (void)hiLi_i_cms;
    (void)hiRe_i_cms;
#endif
}

#endif

#ifdef PRINTER_MODE_RAEDER

void TelemetryPrinter::printWheels(
    VehicleController& vehicle,
    float v2_ist_cms,
    float v3_ist_cms,
    int16_t pwm2,
    int16_t pwm3,
    uint32_t t_ms)
{
#if PRINTER_ENABLE_WHEELS
    Serial.print(F("#WHEELS,"));
    Serial.print(t_ms);                             Serial.print(',');

    printSpeedCms(vehicle.getWheelSoll(VoLi));      Serial.print(',');
    printSpeedCms(speed[Li].cms());                 Serial.print(',');
    Serial.print(rad[Li].lastPwm());                Serial.print(',');

    printSpeedCms(vehicle.getWheelSoll(VoRe));      Serial.print(',');
    printSpeedCms(speed[Re].cms());                 Serial.print(',');
    Serial.print(rad[Re].lastPwm());                Serial.print(',');

    printSpeedCms(vehicle.getWheelSoll(HiLi));      Serial.print(',');
    printSpeedCms(v2_ist_cms);                      Serial.print(',');
    Serial.print(pwm2);                             Serial.print(',');

    printSpeedCms(vehicle.getWheelSoll(HiRe));      Serial.print(',');
    printSpeedCms(v3_ist_cms);                      Serial.print(',');
    Serial.println(pwm3);
#else
    (void)vehicle;
    (void)v2_ist_cms;
    (void)v3_ist_cms;
    (void)pwm2;
    (void)pwm3;
    (void)t_ms;
#endif

#if PRINTER_ENABLE_COUNTS
    Serial.print(F("#CNTF,"));
    Serial.print(t_ms);                     Serial.print(',');
    Serial.print(speed[Li].counts_total()); Serial.print(',');
    Serial.println(speed[Re].counts_total());
#endif
}

void TelemetryPrinter::printFrame(
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
    int16_t hiRe_pwm)
{
#if PRINTER_ENABLE_WHEELS
    Serial.print(F("#WHEELS,"));
    Serial.print(t_ms);             Serial.print(',');

    printSpeedCms(voLi_s_cms);      Serial.print(',');
    printSpeedCms(voLi_i_cms);      Serial.print(',');
    Serial.print(voLi_pwm);         Serial.print(',');

    printSpeedCms(voRe_s_cms);      Serial.print(',');
    printSpeedCms(voRe_i_cms);      Serial.print(',');
    Serial.print(voRe_pwm);         Serial.print(',');

    printSpeedCms(hiLi_s_cms);      Serial.print(',');
    printSpeedCms(hiLi_i_cms);      Serial.print(',');
    Serial.print(hiLi_pwm);         Serial.print(',');

    printSpeedCms(hiRe_s_cms);      Serial.print(',');
    printSpeedCms(hiRe_i_cms);      Serial.print(',');
    Serial.println(hiRe_pwm);
#else
    (void)t_ms;
    (void)voLi_s_cms;
    (void)voLi_i_cms;
    (void)voLi_pwm;
    (void)voRe_s_cms;
    (void)voRe_i_cms;
    (void)voRe_pwm;
    (void)hiLi_s_cms;
    (void)hiLi_i_cms;
    (void)hiLi_pwm;
    (void)hiRe_s_cms;
    (void)hiRe_i_cms;
    (void)hiRe_pwm;
#endif

#if PRINTER_ENABLE_COUNTS
    Serial.print(F("#CNTF,"));
    Serial.print(t_ms);                     Serial.print(',');
    Serial.print(speed[Li].counts_total()); Serial.print(',');
    Serial.println(speed[Re].counts_total());
#endif
}

#endif