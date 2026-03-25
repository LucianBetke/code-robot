#pragma once
#include <Arduino.h>
#include "globals.h"
#include "TestTypes.h"

bool modeAllowsBoth(TestMode m);
int8_t cmd_dir_from_ux(float ux);
void start_time_limited(uint32_t& start_ms, bool& enabled);
Wheel wheelFromSide(Side side);
