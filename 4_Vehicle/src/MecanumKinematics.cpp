// ============================================================
// File: MecanumKinematics.cpp
// Koordinatenkonvention:
// +x  = vorwaerts
// +y  = links
// +z  = oben
// +wz = Drehung gegen den Uhrzeigersinn, von oben betrachtet
//
// Einheiten:
// vx, vy, Radgeschwindigkeiten: cm/s
// wz: rad/s
// ============================================================

#include "MecanumKinematics.h"

#include <math.h>

void MecanumKinematics::limitTranslation(float& vx_cms, float& vy_cms)
{
    const float sum = fabsf(vx_cms) + fabsf(vy_cms);

    if (sum <= V_WHEEL_MAX)
    {
        return;
    }

    if (sum <= 0.0001f)
    {
        vx_cms = 0.0f;
        vy_cms = 0.0f;
        return;
    }

    const float scale = V_WHEEL_MAX / sum;

    vx_cms *= scale;
    vy_cms *= scale;
}

void MecanumKinematics::inverse(
    float vx_cms,
    float vy_cms,
    float wz_rad_s,
    float wheelSoll[WHEEL_VEHICLE_COUNT])
{
    float v[WHEEL_VEHICLE_COUNT];

    v[VoRe] = vx_cms + vy_cms + MECANUM_K_CM * wz_rad_s;
    v[VoLi] = vx_cms - vy_cms - MECANUM_K_CM * wz_rad_s;
    v[HiLi] = vx_cms + vy_cms - MECANUM_K_CM * wz_rad_s;
    v[HiRe] = vx_cms - vy_cms + MECANUM_K_CM * wz_rad_s;

    float maxVal = 0.0f;

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        const float a = fabsf(v[i]);
        if (a > maxVal) maxVal = a;
    }

    if (maxVal > V_WHEEL_MAX && maxVal > 0.0001f)
    {
        const float scale = V_WHEEL_MAX / maxVal;

        for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
        {
            v[i] *= scale;
        }
    }

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        if (fabsf(v[i]) < V_WHEEL_MIN)
        {
            v[i] = 0.0f;
        }
    }

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        wheelSoll[i] = v[i];
    }
}

void MecanumKinematics::forward(
    const WheelSpeedCms& wheelIst,
    float& vx_cms,
    float& vy_cms,
    float& wz_rad_s)
{
    const float voRe_cms = wheelIst.v[VoRe];
    const float voLi_cms = wheelIst.v[VoLi];
    const float hiLi_cms = wheelIst.v[HiLi];
    const float hiRe_cms = wheelIst.v[HiRe];

    vx_cms = (voRe_cms + voLi_cms + hiLi_cms + hiRe_cms) / 4.0f;

    vy_cms = (voRe_cms - voLi_cms + hiLi_cms - hiRe_cms) / 4.0f;

    wz_rad_s =
        (voRe_cms - voLi_cms - hiLi_cms + hiRe_cms) /
        (4.0f * MECANUM_K_CM);
}