// Printer.cpp
#include "Printer.h"

void Printer::regel_line_both(float t_s,
    float vSollRe, float vIstRe, int16_t pwmRe,
    float vSollLi, float vIstLi, int16_t pwmLi)
{
    Serial.print(t_s, 3);
    Serial.print(',');

    Serial.print(vSollRe, 3);
    Serial.print(',');
    Serial.print(vIstRe, 3);
    Serial.print(',');
    Serial.print(pwmRe);
    Serial.print(',');

    Serial.print(vSollLi, 3);
    Serial.print(',');
    Serial.print(vIstLi, 3);
    Serial.print(',');
    Serial.println(pwmLi);
}