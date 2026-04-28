// Print.cpp
#include "Print.h"
#include "Control.h"

void print_begin()
{
    Serial.println("REGEL Hinten");

    PIParam piRe = rad[Re].getPI();
    PIParam piLi = rad[Li].getPI();

    Serial.print("KpRe=");
    Serial.print(piRe.Kp, 2);
    Serial.print(" KiRe=");
    Serial.print(piRe.Ki, 2);
    Serial.print(" KpLi=");
    Serial.print(piLi.Kp, 2);
    Serial.print(" KiLi=");
    Serial.println(piLi.Ki, 2);

    Serial.print("DeadRe=");
    Serial.print(rad[Re].deadPwm());
    Serial.print(" DeadLi=");
    Serial.println(rad[Li].deadPwm());

    Serial.println("t_s,vSollLi,vIstLi,pwmLi,vSollRe,vIstRe,pwmRe");
}

void print_update(uint32_t now)
{
    static uint32_t lastPrint = 0;

    if (now - lastPrint < DBG_INTERVAL_MS)
        return;

    lastPrint = now;

    float t_s = now / 1000.0f;

    Serial.print(t_s, 3);
    Serial.print(',');

    Serial.print(rad[Li].soll(), 3);
    Serial.print(',');
    Serial.print(rad[Li].vIst(), 3);
    Serial.print(',');
    Serial.print(rad[Li].lastPwm());
    Serial.print(',');

    Serial.print(rad[Re].soll(), 3);
    Serial.print(',');
    Serial.print(rad[Re].vIst(), 3);
    Serial.print(',');
    Serial.println(rad[Re].lastPwm());
}