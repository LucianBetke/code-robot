// ============================================================
// VehiclePIController.h
// ============================================================
#ifndef VEHICLE_PI_CONTROLLER_H
#define VEHICLE_PI_CONTROLLER_H

#include <Arduino.h>

class VehicleRegler
{
public:
    VehicleRegler();
    void setParams(float Kp_vx, float Ki_vx,
        float Kp_vy, float Ki_vy,
        float Kp_wz, float Ki_wz);
    void reset();

    float updateVx(float soll, float ist, uint16_t dt_ms);
    float updateVy(float soll, float ist, uint16_t dt_ms);
    float updateWz(float soll, float ist, uint16_t dt_ms);

    float KpVx() const { return _Kp_vx; }
    float KiVx() const { return _Ki_vx; }
    float KpVy() const { return _Kp_vy; }
    float KiVy() const { return _Ki_vy; }
    float KpWz() const { return _Kp_wz; }
    float KiWz() const { return _Ki_wz; }

private:
    float _Kp_vx, _Ki_vx;
    float _Kp_vy, _Ki_vy;
    float _Kp_wz, _Ki_wz;

    float _integral_vx;
    float _integral_vy;
    float _integral_wz;
};

#endif // VEHICLE_PI_CONTROLLER_H