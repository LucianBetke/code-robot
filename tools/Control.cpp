// ============================================================
// File: Control.cpp
// ============================================================

#include "Control.h"
#include "Hardware.h"
#include "globals.h"
#include "ControlParams.h"  // neu
#include "VehicleController.h"

VehicleController vehicle(motor[Re], motor[Li]);

// --- Mixer / Command ---
Mixer2W mixer2W;
VehicleCmd cmdVehicle{ 0.0f, 0.0f, 0.0f };

// --- Hinterachse ---
Achse_Hi achse_Hi(motor[Li], motor[Re]);


// --- SpeedWeg ---
SpeedWeg speed[WHEEL_COUNT] =
{
    SpeedWeg(enc[Li]),
    SpeedWeg(enc[Re])
};

// --- Regler / Rad ---
PIRegler regler[WHEEL_COUNT] =
{
    PIRegler(PI_PARAMS[Li].Kp, PI_PARAMS[Li].Ki, -MAX_PWM, MAX_PWM),
    PIRegler(PI_PARAMS[Re].Kp, PI_PARAMS[Re].Ki, -MAX_PWM, MAX_PWM)
};

Rad rad[WHEEL_COUNT] =
{
    Rad(motor[Li], speed[Li], regler[Li], RAD_REGEL_DT_MS, Li),
    Rad(motor[Re], speed[Re], regler[Re], RAD_REGEL_DT_MS, Re)
};

void control_begin(bool resetEnc)
{
    // Hardware-Polung der Hinterachse (konstant)
    mixer2W.setKW(1.0f);    // Testfaktor Rotation -> Gegenläufigkeit
    cmdVehicle = { 0.0f, 0.0f, 0.0f };

    speed[Re].reset();
    speed[Li].reset();
    speed[Re].setTimeoutMs(500);
    speed[Li].setTimeoutMs(500);
}

SideBinding bindSide(Side side)
{
    SideBinding b;

    if (side == Side::LI) {
        b.m = &motor[Li];
        b.e = &enc[Li];
        b.s = &speed[Li];
        b.r = &rad[Li];
    }
    else {
        b.m = &motor[Re];
        b.e = &enc[Re];
        b.s = &speed[Re];
        b.r = &rad[Re];
    }

    return b;
}