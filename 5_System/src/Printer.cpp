// Printer.cpp
#include "Printer.h"

#include "src/MecanumOdometer.h"

namespace
{
    long valueToInt100(float value)
    {
        if (value >= 0.0f)
        {
            return (long)(value * 100.0f + 0.5f);
        }

        return (long)(value * 100.0f - 0.5f);
    }

    int16_t mpsToCmsRounded(float value_mps)
    {
        const float value_cms = value_mps * 100.0f;

        if (value_cms >= 0.0f)
        {
            return (int16_t)(value_cms + 0.5f);
        }

        return (int16_t)(value_cms - 0.5f);
    }

    void printSpeedCmsFromMps(float value_mps)
    {
        Serial.print((int)mpsToCmsRounded(value_mps));
    }
}

void Printer::printInfo(VehicleController& vehicle, const ControlConfig& cfg)
{
#if PRINTER_ENABLE_INFO
    Serial.print(F("#INFO,Raeder,Li,Kp100=")); Serial.print(valueToInt100(cfg.pi[Li].Kp));
    Serial.print(F(",Ki100="));                Serial.print(valueToInt100(cfg.pi[Li].Ki));
    Serial.print(F(",dead="));                 Serial.print(cfg.deadPwm[Li]);
    Serial.print(F(",Re,Kp100="));             Serial.print(valueToInt100(cfg.pi[Re].Kp));
    Serial.print(F(",Ki100="));                Serial.print(valueToInt100(cfg.pi[Re].Ki));
    Serial.print(F(",dead="));                 Serial.println(cfg.deadPwm[Re]);

    Serial.print(F("#INFO,Chassis,vx,Kp100=")); Serial.print(valueToInt100(vehicle.KpVx()));
    Serial.print(F(",Ki100="));                  Serial.print(valueToInt100(vehicle.KiVx()));
    Serial.print(F(",vy,Kp100="));               Serial.print(valueToInt100(vehicle.KpVy()));
    Serial.print(F(",Ki100="));                  Serial.print(valueToInt100(vehicle.KiVy()));
    Serial.print(F(",wz,Kp100="));               Serial.print(valueToInt100(vehicle.KpWz()));
    Serial.print(F(",Ki100="));                  Serial.println(valueToInt100(vehicle.KiWz()));
#else
    (void)vehicle;
    (void)cfg;
#endif
}

void Printer::printCompletedFrame(
    VehicleController& vehicle,
    const RearPendingFrame& frame,
    float hiLi_i,
    float hiRe_i,
    int16_t hiLi_pwm,
    int16_t hiRe_pwm)
{
#ifdef PRINTER_MODE_CHASSIS
    printFrame(
        vehicle,
        frame.t,
        frame.voLi_i,
        frame.voRe_i,
        hiLi_i,
        hiRe_i
    );
#endif

#ifdef PRINTER_MODE_RAEDER
    printFrame(
        frame.t,

        frame.voLi_s,
        frame.voLi_i,
        frame.voLi_pwm,

        frame.voRe_s,
        frame.voRe_i,
        frame.voRe_pwm,

        frame.hiLi_s,
        hiLi_i,
        hiLi_pwm,

        frame.hiRe_s,
        hiRe_i,
        hiRe_pwm
    );
#endif
}

void Printer::printOdom2(
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
    Serial.print(valueToInt100(odom.absCm()));       Serial.print(',');
    Serial.print(valueToInt100(odom.xCm()));         Serial.print(',');
    Serial.print(valueToInt100(odom.yCm()));         Serial.print(',');
    Serial.println(valueToInt100(odom.phiDeg()));
#else
    (void)cmdpId;
    (void)t_ms;
    (void)odom;
#endif
}

#ifdef PRINTER_MODE_CHASSIS

void Printer::printWheels(VehicleController& vehicle,
    float v2_ist, float v3_ist,
    uint32_t t_ms)
{
#if PRINTER_ENABLE_CHASSIS
    Serial.print(F("#CHASSIS,"));
    Serial.print((unsigned long)t_ms); Serial.print(',');

    printSpeedCmsFromMps(speed[Li].mps()); Serial.print(',');
    printSpeedCmsFromMps(speed[Re].mps()); Serial.print(',');
    printSpeedCmsFromMps(v2_ist);          Serial.print(',');
    printSpeedCmsFromMps(v3_ist);          Serial.print(',');

    printSpeedCmsFromMps(vehicle.vxIst()); Serial.print(',');
    printSpeedCmsFromMps(vehicle.vyIst()); Serial.print(',');

    Serial.println(valueToInt100(vehicle.wzIst()));
#else
    (void)vehicle;
    (void)v2_ist;
    (void)v3_ist;
    (void)t_ms;
#endif
}

void Printer::printFrame(VehicleController& vehicle,
    uint32_t t_ms,
    float voLi_i,
    float voRe_i,
    float hiLi_i,
    float hiRe_i)
{
#if PRINTER_ENABLE_CHASSIS
    Serial.print(F("#CHASSIS,"));
    Serial.print((unsigned long)t_ms); Serial.print(',');

    printSpeedCmsFromMps(voLi_i);          Serial.print(',');
    printSpeedCmsFromMps(voRe_i);          Serial.print(',');
    printSpeedCmsFromMps(hiLi_i);          Serial.print(',');
    printSpeedCmsFromMps(hiRe_i);          Serial.print(',');

    printSpeedCmsFromMps(vehicle.vxIst()); Serial.print(',');
    printSpeedCmsFromMps(vehicle.vyIst()); Serial.print(',');

    Serial.println(valueToInt100(vehicle.wzIst()));
#else
    (void)vehicle;
    (void)t_ms;
    (void)voLi_i;
    (void)voRe_i;
    (void)hiLi_i;
    (void)hiRe_i;
#endif
}

#endif

#ifdef PRINTER_MODE_RAEDER

void Printer::printWheels(VehicleController& vehicle,
    float v2_ist, float v3_ist,
    int16_t pwm2, int16_t pwm3,
    uint32_t t_ms)
{
#if PRINTER_ENABLE_WHEELS
    Serial.print(F("#WHEELS,"));
    Serial.print((unsigned long)t_ms);             Serial.print(',');

    printSpeedCmsFromMps(vehicle.getWheelSoll(VoLi)); Serial.print(',');
    printSpeedCmsFromMps(speed[Li].mps());            Serial.print(',');
    Serial.print(rad[Li].lastPwm());                  Serial.print(',');

    printSpeedCmsFromMps(vehicle.getWheelSoll(VoRe)); Serial.print(',');
    printSpeedCmsFromMps(speed[Re].mps());            Serial.print(',');
    Serial.print(rad[Re].lastPwm());                  Serial.print(',');

    printSpeedCmsFromMps(vehicle.getWheelSoll(HiLi)); Serial.print(',');
    printSpeedCmsFromMps(v2_ist);                     Serial.print(',');
    Serial.print(pwm2);                               Serial.print(',');

    printSpeedCmsFromMps(vehicle.getWheelSoll(HiRe)); Serial.print(',');
    printSpeedCmsFromMps(v3_ist);                     Serial.print(',');
    Serial.println(pwm3);
#else
    (void)vehicle;
    (void)v2_ist;
    (void)v3_ist;
    (void)pwm2;
    (void)pwm3;
    (void)t_ms;
#endif

#if PRINTER_ENABLE_COUNTS
    Serial.print(F("#CNTF,"));
    Serial.print((unsigned long)t_ms);           Serial.print(',');
    Serial.print(speed[Li].counts_total());      Serial.print(',');
    Serial.println(speed[Re].counts_total());
#endif
}

void Printer::printFrame(
    uint32_t t_ms,
    float voLi_s,
    float voLi_i,
    int16_t voLi_pwm,
    float voRe_s,
    float voRe_i,
    int16_t voRe_pwm,
    float hiLi_s,
    float hiLi_i,
    int16_t hiLi_pwm,
    float hiRe_s,
    float hiRe_i,
    int16_t hiRe_pwm)
{
#if PRINTER_ENABLE_WHEELS
    Serial.print(F("#WHEELS,"));
    Serial.print((unsigned long)t_ms); Serial.print(',');

    printSpeedCmsFromMps(voLi_s);      Serial.print(',');
    printSpeedCmsFromMps(voLi_i);      Serial.print(',');
    Serial.print(voLi_pwm);            Serial.print(',');

    printSpeedCmsFromMps(voRe_s);      Serial.print(',');
    printSpeedCmsFromMps(voRe_i);      Serial.print(',');
    Serial.print(voRe_pwm);            Serial.print(',');

    printSpeedCmsFromMps(hiLi_s);      Serial.print(',');
    printSpeedCmsFromMps(hiLi_i);      Serial.print(',');
    Serial.print(hiLi_pwm);            Serial.print(',');

    printSpeedCmsFromMps(hiRe_s);      Serial.print(',');
    printSpeedCmsFromMps(hiRe_i);      Serial.print(',');
    Serial.println(hiRe_pwm);
#else
    (void)t_ms;
    (void)voLi_s;
    (void)voLi_i;
    (void)voLi_pwm;
    (void)voRe_s;
    (void)voRe_i;
    (void)voRe_pwm;
    (void)hiLi_s;
    (void)hiLi_i;
    (void)hiLi_pwm;
    (void)hiRe_s;
    (void)hiRe_i;
    (void)hiRe_pwm;
#endif

#if PRINTER_ENABLE_COUNTS
    Serial.print(F("#CNTF,"));
    Serial.print((unsigned long)t_ms);      Serial.print(',');
    Serial.print(speed[Li].counts_total()); Serial.print(',');
    Serial.println(speed[Re].counts_total());
#endif
}

#endif