// Tests.h
#pragma once

#include "TestTypes.h"
#include "AutoTuner.h"
//#include "Sequencer.h"


// --- AutoTuner ---
extern AutoTuner tuner[WHEEL_COUNT];

// --- Targets / Sequencer ---

// optional: falls du Tests gezielt initialisieren willst
void tests_begin();
