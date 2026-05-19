// FrameScheduler.cpp
#include "FrameScheduler.h"

FrameScheduler::FrameScheduler()
    : _intervalMs(100),
    _startMs(0),
    _nextFrameMs(0),
    _running(false)
{
}

void FrameScheduler::begin(uint32_t intervalMs)
{
    _intervalMs = intervalMs;
    _startMs = 0;
    _nextFrameMs = 0;
    _running = false;
}

void FrameScheduler::start(uint32_t nowMs)
{
    _startMs = nowMs;
    _nextFrameMs = nowMs + _intervalMs;
    _running = true;
}

void FrameScheduler::stop()
{
    _running = false;
    _startMs = 0;
    _nextFrameMs = 0;
}

bool FrameScheduler::isRunning() const
{
    return _running;
}

bool FrameScheduler::due(uint32_t nowMs, uint32_t& frameTimeMs)
{
    if (!_running)
    {
        return false;
    }

    if (!timeReached(nowMs, _nextFrameMs))
    {
        return false;
    }

    frameTimeMs = _nextFrameMs - _startMs;
    _nextFrameMs += _intervalMs;

    return true;
}

bool FrameScheduler::timeReached(uint32_t nowMs, uint32_t targetMs)
{
    return (int32_t)(nowMs - targetMs) >= 0;
}