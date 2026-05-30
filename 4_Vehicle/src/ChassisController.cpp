// ============================================================
// File: ChassisController.cpp
// Zweck:
//  - Paket 4: Chassis-Istwerte aufnehmen
//  - Noch keine Chassisregelung aktiv
// ============================================================

#include "ChassisController.h"

void ChassisController::reset()
{
    _state.x_body_cm = 0.0f;
    _state.y_body_cm = 0.0f;
    _state.path_cm = 0.0f;
    _state.phi_rad = 0.0f;
}

void ChassisController::updateState(const ChassisState& state)
{
    _state = state;
}

void ChassisController::update(
    float vx_soll_cms,
    float vy_soll_cms,
    float wz_soll_rad_s,
    float& vx_out_cms,
    float& vy_out_cms,
    float& wz_out_rad_s)
{
    // Noch neutraler Durchschleifbetrieb.
    // Die gespeicherten Chassis-Istwerte beeinflussen vx/vy/wz noch nicht.

    vx_out_cms = vx_soll_cms;
    vy_out_cms = vy_soll_cms;
    wz_out_rad_s = wz_soll_rad_s;
}