// ============================================================
// File: ChassisControlConfig.h
// Zweck:
//  - Parameter fuer die aeussere Chassisregelung
//
// Einheiten:
//  - phi: rad
//  - wz: rad/s
//  - Kp_phi: 1/s, weil wz_korr = -Kp_phi * phi
//
// Bedienung:
//  - Winkelwerte koennen hier bequem in Grad angegeben werden.
//  - Die Umrechnung nach rad erfolgt zur Compile-Zeit.
// ============================================================

#ifndef CHASSIS_CONTROL_CONFIG_H
#define CHASSIS_CONTROL_CONFIG_H

constexpr float CHASSIS_DEG_TO_RAD = 0.017453292519943295f;

struct ChassisControlConfig
{
    bool phiControlEnabled;

    float Kp_phi;
    float maxWzCorrection_rad_s;
    float phiDeadband_rad;
};

namespace ConfigChassisFront
{
    constexpr ChassisControlConfig CONFIG =
    {
        true,        // phiControlEnabled
//        false,        // phiControlDiabled
        // Startwert fuer ersten Test:
        // phi = 5 deg = 0.087 rad
        // wz_korr = -0.8 * 0.087 = -0.070 rad/s
        0.80f,      // Kp_phi [1/s]

        // Begrenzung der zusaetzlichen Drehkorrektur.
        0.50f,      // maxWzCorrection_rad_s

        // Totband in Grad eingeben.
        // 0.5 deg wird zur Compile-Zeit nach rad umgerechnet.
        0.5f * CHASSIS_DEG_TO_RAD
    };
}

#endif // CHASSIS_CONTROL_CONFIG_H