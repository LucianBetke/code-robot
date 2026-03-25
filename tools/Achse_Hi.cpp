// Achse_Hi.cpp
#include "Achse_Hi.h"

Achse_Hi::Achse_Hi(Motor& li, Motor& re)
    : _li(li), _re(re)
{
}

void Achse_Hi::bremse(bool art)
{
    _li.bremse(art);
    _re.bremse(art);
}