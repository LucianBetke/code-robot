// ============================================================
// File: ChassisController.cpp
// Zweck:
//  - Neutraler Durchschleifblock fuer Paket 3
//  - Noch keine Chassisregelung aktiv
// ============================================================

#include "ChassisController.h"

void ChassisController::reset()
{
    // Noch kein interner Zustand.
    // Spaeter kommen hier z. B. Integrator-Reset oder Fehlerzustand-Reset hin.
}

void ChassisController::update(
    float vx_soll_cms,
    float vy_soll_cms,
    float wz_soll_rad_s,
    float& vx_out_cms,
    float& vy_out_cms,
    float& wz_out_rad_s)
{
    // Neutraler Durchschleifbetrieb.
    // Fahrverhalten muss dadurch identisch zu vorher bleiben.

    vx_out_cms = vx_soll_cms;
    vy_out_cms = vy_soll_cms;
    wz_out_rad_s = wz_soll_rad_s;
}