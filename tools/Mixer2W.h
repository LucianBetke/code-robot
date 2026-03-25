// Mixer2W.h
#pragma once
#include "VehicleTypes.h"

class Mixer2W {
public:
    void setKW(float kW);
    AxleRef mix(const VehicleCmd& cmd) const;

private:
    float _kW = 1.0f;
};