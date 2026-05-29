// ============================================================
// File: RadMessung.cpp
// ============================================================

#include "RadMessung.h"
#include "src/RobotConfig.h"
#include "src/Encoder.h"
#include "src/ScaleUtils.h"

namespace
{
    const float SPEED_ALPHA = 0.2f;
}

RadMessung::RadMessung(Enc& enc)
    : _enc(enc)
{
    _last_counts = _enc.getCounts();
    _counts_total = 0;

    _acc_counts = 0;
    _rps_filt = 0.0f;
    _last_time_ms = 0;
    _last_tick_ms = 0;
}

void RadMessung::reset()
{
    _counts_total = 0;

    _acc_counts = 0;
    _rps_filt = 0.0f;
    _last_time_ms = 0;
    _last_tick_ms = 0;

    _last_counts = _enc.getCounts();
}

void RadMessung::update(uint32_t now_ms)
{
    long cur = _enc.getCounts();
    long d = cur - _last_counts;

    if (d != 0)
    {
        if (d > 32767) d = 32767;
        if (d < -32768) d = -32768;

        updateFromTicks((int16_t)d, now_ms);

        _last_counts = cur;
        _last_tick_ms = now_ms;
    }

    timeoutCheck(now_ms);
}

void RadMessung::updateFromTicks(int16_t dcounts, uint32_t now_ms)
{
    _counts_total += dcounts;
    _acc_counts += dcounts;

    if (_last_time_ms == 0)
    {
        _last_time_ms = now_ms;
        return;
    }

    uint32_t dt_ms = now_ms - _last_time_ms;

    if (dt_ms < SPEED_MIN_DT_MS)
    {
        return;
    }

    _last_time_ms = now_ms;

    const float dt_s = (float)dt_ms * 0.001f;
    const float revs = (float)_acc_counts / (float)COUNTS_PER_REV;
    const float rps_i = revs / dt_s;

    _rps_filt += SPEED_ALPHA * (rps_i - _rps_filt);

    _acc_counts = 0;
}

void RadMessung::timeoutCheck(uint32_t now_ms)
{
    if (_last_tick_ms == 0)
    {
        return;
    }

    if ((uint32_t)(now_ms - _last_tick_ms) > SPEED_TIMEOUT_MS)
    {
        _rps_filt = 0.0f;
    }
}

float RadMessung::cms() const
{
    return _rps_filt * RAD_UMFANG_CM;
}

int16_t RadMessung::cmsInt() const
{
    return scaleRoundToInt16(cms());
}