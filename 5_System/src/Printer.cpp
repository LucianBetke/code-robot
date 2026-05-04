// ============================================================
// Printer.cpp
// ============================================================
#include "Printer.h"

void Printer::printHeader()
{
    Serial.print('#');
    Serial.println(F("ms,VoLi_s,VoLi_i,VoLi_pwm,VoRe_s,VoRe_i,VoRe_pwm,HiLi_s,HiLi_i,HiLi_pwm,HiRe_s,HiRe_i,HiRe_pwm"));
}

void Printer::printWheels(VehicleController& vehicle,
    float v2_ist, float v3_ist,
    int16_t pwm2, int16_t pwm3,
    uint32_t t_ms)
{
    Serial.print('#');
    Serial.print(t_ms);                              Serial.print(',');
    Serial.print(vehicle.getWheelSoll(VoLi), 2);    Serial.print(',');
    Serial.print(speed[Li].mps(), 2);               Serial.print(',');
    Serial.print(rad[Li].lastPwm());                Serial.print(',');
    Serial.print(vehicle.getWheelSoll(VoRe), 2);    Serial.print(',');
    Serial.print(speed[Re].mps(), 2);               Serial.print(',');
    Serial.print(rad[Re].lastPwm());                Serial.print(',');
    Serial.print(vehicle.getWheelSoll(HiLi), 2);   Serial.print(',');
    Serial.print(v2_ist, 2);                        Serial.print(',');
    Serial.print(pwm2);                             Serial.print(',');
    Serial.print(vehicle.getWheelSoll(HiRe), 2);   Serial.print(',');
    Serial.print(v3_ist, 2);                        Serial.print(',');
    Serial.println(pwm3);
}