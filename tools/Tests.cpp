// ============================================================
// File: Tests.cpp
// ============================================================

#include "Tests.h"
#include "Control.h"
#include "Hardware.h"
#include "ControlParams.h"

// --- AutoTuner ---
AutoTuner tuner[WHEEL_COUNT] =
{
    AutoTuner(motor[Li], speed[Li], Li, true),
    AutoTuner(motor[Re], speed[Re], Re, true)
};

// --- Targets / Sequencer ---

void tests_begin()
{
    // aktuell nichts nötig
}
