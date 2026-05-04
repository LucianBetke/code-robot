// ============================================================
// VehicleController.h
// ============================================================
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "src/globals.h"
#include "VehicleRegler.h"

class VehicleController
{
public:
    void begin(float Kp_vx, float Ki_vx,
        float Kp_vy, float Ki_vy,
        float Kp_wz, float Ki_wz);
    void cmd(float vx, float vy, float wz);
    void updateIst(float v0, float v1, float v2, float v3);
    void update(uint32_t now);
    void stop();

    float getWheelSoll(WheelVehicle w) const;
    float vxIst() const { return _vx_ist; }
    float vyIst() const { return _vy_ist; }
    float wzIst() const { return _wz_ist; }

private:
    void applyMixer(float vx, float vy, float wz);

    float _vx = 0.0f;
    float _vy = 0.0f;
    float _wz = 0.0f;

    float _vx_ist = 0.0f;
    float _vy_ist = 0.0f;
    float _wz_ist = 0.0f;

    float _wheelSoll[WHEEL_VEHICLE_COUNT] = { 0 };

    VehicleRegler _regler;
    uint32_t _lastUpdateMs = 0;
};

#endif