// ============================================================
// File: MecanumKinematics.h
// Zweck:
//  - Reine Mecanum-Kinematik fuer die Vehicle-Ebene
//  - Rueckrechnung Radgeschwindigkeiten -> vx/vy/wz
//  - Mischung vx/vy/wz -> Rad-Sollgeschwindigkeiten
//
// Einheiten:
//  - vx/vy/Radgeschwindigkeiten: cm/s
//  - wz: rad/s
// ============================================================

#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include "src/RobotConfig.h"
#include "src/WheelValues.h"

class MecanumKinematics
{
public:
    static void limitTranslation(float& vx_cms, float& vy_cms);

    static void inverse(
        float vx_cms,
        float vy_cms,
        float wz_rad_s,
        float wheelSoll[WHEEL_VEHICLE_COUNT]);

    static void forward(
        const WheelSpeedCms& wheelIst,
        float& vx_cms,
        float& vy_cms,
        float& wz_rad_s);
};

#endif // MECANUM_KINEMATICS_H