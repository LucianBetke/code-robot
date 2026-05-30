// ============================================================
// File: ChassisController.h
// Zweck:
//  - Aeusserer Chassis-Regler
//  - Korrigiert vx/vy/wz anhand von Chassisfehlern
//  - Paket 5: erste echte Regelwirkung phi -> wz
//
// Einheiten:
//  - vx/vy: cm/s
//  - wz: rad/s
//  - x/y/path: cm
//  - phi: rad
// ============================================================

#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

struct ChassisState
{
    float x_body_cm;
    float y_body_cm;
    float path_cm;
    float phi_rad;
};

class ChassisController
{
public:
    void reset();

    void updateState(const ChassisState& state);

    void update(
        float vx_soll_cms,
        float vy_soll_cms,
        float wz_soll_rad_s,
        float& vx_out_cms,
        float& vy_out_cms,
        float& wz_out_rad_s);

    float xBodyCm() const { return _state.x_body_cm; }
    float yBodyCm() const { return _state.y_body_cm; }
    float pathCm() const { return _state.path_cm; }
    float phiRad() const { return _state.phi_rad; }

private:
    float calculatePhiCorrection(float wz_soll_rad_s) const;
    static float limitSymmetric(float value, float limitAbs);

    ChassisState _state;
};

#endif // CHASSIS_CONTROLLER_H