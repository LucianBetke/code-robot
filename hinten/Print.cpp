// Print.cpp
#include "Print.h"
#include "Printer.h"
#include "Control.h"

Printer printer;

void print_update(uint32_t now)
{
    static uint32_t last = 0;
    if (now - last < 50) return;
    last = now;

    float t_s = now * 0.001f;

    printer.regel_line_both(
        t_s,
        rad[Re].soll(),
        rad[Re].vIst(),
        rad[Re].lastPwm(),
        rad[Li].soll(),
        rad[Li].vIst(),
        rad[Li].lastPwm()
    );
}