// Printer.cpp
#include "Printer.h"

void Printer::printHeader(VehicleController& vehicle, const ControlConfig& cfg)
{
    Serial.print(F("#INFO,Raeder,Li,Kp=")); Serial.print(cfg.pi[Li].Kp, 2);
    Serial.print(F(",Ki="));                Serial.print(cfg.pi[Li].Ki, 2);
    Serial.print(F(",dead="));              Serial.print(cfg.deadPwm[Li]);
    Serial.print(F(",Re,Kp="));             Serial.print(cfg.pi[Re].Kp, 2);
    Serial.print(F(",Ki="));                Serial.print(cfg.pi[Re].Ki, 2);
    Serial.print(F(",dead="));              Serial.println(cfg.deadPwm[Re]);

    Serial.print(F("#INFO,Chassis,vx,Kp=")); Serial.print(vehicle.KpVx(), 2);
    Serial.print(F(",Ki="));                  Serial.print(vehicle.KiVx(), 2);
    Serial.print(F(",vy,Kp="));               Serial.print(vehicle.KpVy(), 2);
    Serial.print(F(",Ki="));                  Serial.print(vehicle.KiVy(), 2);
    Serial.print(F(",wz,Kp="));               Serial.print(vehicle.KpWz(), 2);
    Serial.print(F(",Ki="));                  Serial.println(vehicle.KiWz(), 2);

#ifdef PRINTER_MODE_CHASSIS
    Serial.println(F("#HDR,CHASSIS,ms,VoLi_i,VoRe_i,HiLi_i,HiRe_i,vx_i,vy_i,wz_i"));
#endif

#ifdef PRINTER_MODE_RAEDER
    Serial.println(F("#HDR,WHEELS,ms,VoLi_s,VoLi_i,VoLi_pwm,VoRe_s,VoRe_i,VoRe_pwm,HiLi_s,HiLi_i,HiLi_pwm,HiRe_s,HiRe_i,HiRe_pwm"));
#endif
}

#ifdef PRINTER_MODE_CHASSIS

void Printer::printWheels(VehicleController& vehicle,
    float v2_ist, float v3_ist,
    uint32_t t_ms)
{
    Serial.print(F("#CHASSIS,"));
    Serial.print(t_ms);               Serial.print(',');
    Serial.print(speed[Li].mps(), 2); Serial.print(',');
    Serial.print(speed[Re].mps(), 2); Serial.print(',');
    Serial.print(v2_ist, 2);          Serial.print(',');
    Serial.print(v3_ist, 2);          Serial.print(',');
    Serial.print(vehicle.vxIst(), 2); Serial.print(',');
    Serial.print(vehicle.vyIst(), 2); Serial.print(',');
    Serial.println(vehicle.wzIst(), 2);
}

void Printer::printFrame(VehicleController& vehicle,
    uint32_t t_ms,
    float voLi_i,
    float voRe_i,
    float hiLi_i,
    float hiRe_i)
{
    Serial.print(F("#CHASSIS,"));
    Serial.print(t_ms);               Serial.print(',');
    Serial.print(voLi_i, 2);          Serial.print(',');
    Serial.print(voRe_i, 2);          Serial.print(',');
    Serial.print(hiLi_i, 2);          Serial.print(',');
    Serial.print(hiRe_i, 2);          Serial.print(',');
    Serial.print(vehicle.vxIst(), 2); Serial.print(',');
    Serial.print(vehicle.vyIst(), 2); Serial.print(',');
    Serial.println(vehicle.wzIst(), 2);
}

#endif

#ifdef PRINTER_MODE_RAEDER

void Printer::printWheels(VehicleController& vehicle,
    float v2_ist, float v3_ist,
    int16_t pwm2, int16_t pwm3,
    uint32_t t_ms)
{
    Serial.print(F("#WHEELS,"));
    Serial.print(t_ms);                           Serial.print(',');

    Serial.print(vehicle.getWheelSoll(VoLi), 2); Serial.print(',');
    Serial.print(speed[Li].mps(), 2);            Serial.print(',');
    Serial.print(rad[Li].lastPwm());             Serial.print(',');

    Serial.print(vehicle.getWheelSoll(VoRe), 2); Serial.print(',');
    Serial.print(speed[Re].mps(), 2);            Serial.print(',');
    Serial.print(rad[Re].lastPwm());             Serial.print(',');

    Serial.print(vehicle.getWheelSoll(HiLi), 2); Serial.print(',');
    Serial.print(v2_ist, 2);                     Serial.print(',');
    Serial.print(pwm2);                          Serial.print(',');

    Serial.print(vehicle.getWheelSoll(HiRe), 2); Serial.print(',');
    Serial.print(v3_ist, 2);                     Serial.print(',');
    Serial.println(pwm3);
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
    Serial.print(F("#WHEELS,"));
    Serial.print(t_ms);      Serial.print(',');

    Serial.print(voLi_s, 2); Serial.print(',');
    Serial.print(voLi_i, 2); Serial.print(',');
    Serial.print(voLi_pwm);  Serial.print(',');

    Serial.print(voRe_s, 2); Serial.print(',');
    Serial.print(voRe_i, 2); Serial.print(',');
    Serial.print(voRe_pwm);  Serial.print(',');

    Serial.print(hiLi_s, 2); Serial.print(',');
    Serial.print(hiLi_i, 2); Serial.print(',');
    Serial.print(hiLi_pwm);  Serial.print(',');

    Serial.print(hiRe_s, 2); Serial.print(',');
    Serial.print(hiRe_i, 2); Serial.print(',');
    Serial.println(hiRe_pwm);
}

#endif