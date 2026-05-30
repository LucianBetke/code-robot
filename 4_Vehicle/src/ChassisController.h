// ============================================================
// File: ChassisController.h
// Zweck:
//  - Spaeterer aeusserer Chassis-Regler
//  - Korrigiert spaeter vx/vy/wz anhand von Chassisfehlern
//  - In diesem Paket noch neutraler Durchschleifblock
//
// Einheiten:
//  - vx/vy: cm/s
//  - wz: rad/s
// ============================================================

#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

class ChassisController
{
public:
    void reset();

    void update(
        float vx_soll_cms,
        float vy_soll_cms,
        float wz_soll_rad_s,
        float& vx_out_cms,
        float& vy_out_cms,
        float& wz_out_rad_s);
};

#endif // CHASSIS_CONTROLLER_H
