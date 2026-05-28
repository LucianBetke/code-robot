// ============================================================
// VehicleController.h
// Einheit:
//  - vx, vy, Radgeschwindigkeiten: cm/s
//  - wz: rad/s
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

    void cmd(float vx_cms, float vy_cms, float wz_rad_s);
    void updateIst(float v0_cms, float v1_cms, float v2_cms, float v3_cms);
    void update(uint32_t now);
    void stop();

    float getWheelSoll(WheelVehicle w) const;

    float vxIst() const { return _vx_ist; }
    float vyIst() const { return _vy_ist; }
    float wzIst() const { return _wz_ist; }

    float vxSoll() const { return _vx; }
    float vySoll() const { return _vy; }
    float wzSoll() const { return _wz; }

    float KpVx() const { return _regler.KpVx(); }
    float KiVx() const { return _regler.KiVx(); }
    float KpVy() const { return _regler.KpVy(); }
    float KiVy() const { return _regler.KiVy(); }
    float KpWz() const { return _regler.KpWz(); }
    float KiWz() const { return _regler.KiWz(); }

private:
    void applyDriveMode(float& vx_cms, float& vy_cms, float wz_rad_s);
    void limitTranslation(float& vx_cms, float& vy_cms);
    void applyMixer(float vx_cms, float vy_cms, float wz_rad_s);

    float _vx = 0.0f;
    float _vy = 0.0f;
    float _wz = 0.0f;

    float _vx_ist = 0.0f;
    float _vy_ist = 0.0f;
    float _wz_ist = 0.0f;

    float _wheelSoll[WHEEL_VEHICLE_COUNT] = { 0 };

    VehicleRegler _regler;
    uint32_t _lastUpdateMs = 0;

    bool _turnOnly = false;
};

#endif