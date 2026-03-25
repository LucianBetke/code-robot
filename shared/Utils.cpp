#include "Utils.h"

bool modeAllowsBoth(TestMode m)
{
// --- Helper: Side prüfen ---
    switch (m)
    {
    case TestMode::ENC_HAND:
    case TestMode::REGEL:
        return true;

    default:
        return false;
    }
}
// Nur Drehrichtung aus CMD_UX: + = vor, - = zurück
int8_t cmd_dir_from_ux(float ux)
{
    return (ux >= 0.0f) ? +1 : -1;
}

// Initialisiert Zeitlimit-Logik für zeitbegrenzte Testmodi
void start_time_limited(uint32_t& start_ms, bool& enabled)
{
    start_ms = millis();
    enabled = true;
}

Wheel wheelFromSide(Side side)
{
    return (side == Side::LI) ? Li : Re;
}