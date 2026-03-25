#include "Mixer2W.h"

void Mixer2W::setKW(float kW) { _kW = kW; }

AxleRef Mixer2W::mix(const VehicleCmd& cmd) const {
    AxleRef r;
    r.vL = cmd.vx - _kW * cmd.wz;
    r.vR = cmd.vx + _kW * cmd.wz;
    return r;
}
