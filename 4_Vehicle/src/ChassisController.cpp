// ============================================================
// File: ChassisController.cpp
// Zweck:
//  - Paket 5: erste Chassisregelung
//  - phi-Fehler erzeugt wz-Korrektur
//  - Diagnose: letzter phi-Reglerausgang und Radbeitrag werden gespeichert
//
// Regelidee:
//  - Odometrie wird bei CMDP-Start genullt.
//  - Gewuenschte Verdrehung im Fahrabschnitt: phi_soll = 0 rad
//  - Fehler: e_phi = phi_soll - phi_ist = -phi_ist
//  - Korrektur: wz_phi = Kp_phi * e_phi = -Kp_phi * phi_ist
// ============================================================

#include "ChassisController.h"
#include "ChassisControlConfig.h"
#include "src/RobotConfig.h"

#include <math.h>

static const float CHASSIS_WZ_COMMAND_EPS = 0.0001f;

void ChassisController::reset()
{
    _state.x_body_cm = 0.0f;
    _state.y_body_cm = 0.0f;
    _state.path_cm = 0.0f;
    _state.phi_rad = 0.0f;

    _lastPhiWzCorrectionRadS = 0.0f;
    _lastPhiWheelDeltaCms = 0.0f;
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
    vx_out_cms = vx_soll_cms;
    vy_out_cms = vy_soll_cms;

    const float wz_phi = calculatePhiCorrection(wz_soll_rad_s);

    _lastPhiWzCorrectionRadS = wz_phi;
    _lastPhiWheelDeltaCms = MECANUM_K_CM * wz_phi;

    wz_out_rad_s = wz_soll_rad_s + wz_phi;
}

float ChassisController::calculatePhiCorrection(float wz_soll_rad_s) const
{
    const ChassisControlConfig& cfg = ConfigChassisFront::CONFIG;

    if (!cfg.phiControlEnabled)
    {
        return 0.0f;
    }

    // Wenn eine Drehung bewusst vorgegeben ist, darf der phi-Regler
    // diese Drehung nicht auf phi = 0 zurueckregeln.
    if (fabsf(wz_soll_rad_s) > CHASSIS_WZ_COMMAND_EPS)
    {
        return 0.0f;
    }

    const float phi = _state.phi_rad;

    if (fabsf(phi) < cfg.phiDeadband_rad)
    {
        return 0.0f;
    }

    const float wz_phi = -cfg.Kp_phi * phi;

    return limitSymmetric(wz_phi, cfg.maxWzCorrection_rad_s);
}

float ChassisController::limitSymmetric(float value, float limitAbs)
{
    if (limitAbs <= 0.0f)
    {
        return 0.0f;
    }

    if (value > limitAbs)
    {
        return limitAbs;
    }

    if (value < -limitAbs)
    {
        return -limitAbs;
    }

    return value;
}