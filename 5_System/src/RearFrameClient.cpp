// RearFrameClient.cpp
#include "RearFrameClient.h"

RearFrameClient::RearFrameClient()
    : _nextFrameId(1),
    _frame()
{
}

void RearFrameClient::begin()
{
    _nextFrameId = 1;
    clearFrame();
}

uint16_t RearFrameClient::nextFrameId()
{
    uint16_t id = _nextFrameId++;

    if (_nextFrameId == 0) _nextFrameId = 1;

    return id;
}

RearPendingFrame& RearFrameClient::frame()
{
    return _frame;
}

const RearPendingFrame& RearFrameClient::frame() const
{
    return _frame;
}

void RearFrameClient::clearFrame()
{
    _frame.frameId = 0;
    _frame.t = 0;

    _frame.voLi_s = 0.0f;
    _frame.voLi_i = 0.0f;
    _frame.voLi_pwm = 0;

    _frame.voRe_s = 0.0f;
    _frame.voRe_i = 0.0f;
    _frame.voRe_pwm = 0;

    _frame.hiLi_s = 0.0f;
    _frame.hiRe_s = 0.0f;

    _frame.hasFrontSnapshot = false;
}