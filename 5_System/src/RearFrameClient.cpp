// RearFrameClient.cpp
#include "RearFrameClient.h"

#include <Arduino.h>
#include "src/CommProtocol.h"
#include "src/CommUtils.h"

RearFrameClient::RearFrameClient()
    : _nextFrameId(1),
    _frame(),
    _requestMs(0),
    _lastSendMs(0),
    _waitingVsolOk(false),
    _waitingVist(false),
    _stopSendCount(0),
    _stopSequenceArmed(false)
{
}

void RearFrameClient::begin()
{
    _nextFrameId = 1;
    _lastSendMs = 0;

    _stopSendCount = 0;
    _stopSequenceArmed = false;

    clearFrame();
    clearWaiting();
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

bool RearFrameClient::waitingVsolOk() const
{
    return _waitingVsolOk;
}

bool RearFrameClient::waitingVist() const
{
    return _waitingVist;
}

bool RearFrameClient::isBusy() const
{
    return _waitingVsolOk || _waitingVist;
}

uint32_t RearFrameClient::requestMs() const
{
    return _requestMs;
}

uint32_t RearFrameClient::lastSendMs() const
{
    return _lastSendMs;
}

bool RearFrameClient::requestFrame(
    Stream& out,
    uint32_t nowMs,
    uint32_t frameTimeMs,
    bool resetPi,
    float voLi_s,
    float voLi_i,
    int16_t voLi_pwm,
    float voRe_s,
    float voRe_i,
    int16_t voRe_pwm,
    float hiLi_s,
    float hiRe_s)
{
    if (isBusy())
    {
        return false;
    }

    const uint16_t frameId = nextFrameId();

    _frame.frameId = frameId;
    _frame.t = frameTimeMs;

    _frame.voLi_s = voLi_s;
    _frame.voLi_i = voLi_i;
    _frame.voLi_pwm = voLi_pwm;

    _frame.voRe_s = voRe_s;
    _frame.voRe_i = voRe_i;
    _frame.voRe_pwm = voRe_pwm;

    _frame.hiLi_s = hiLi_s;
    _frame.hiRe_s = hiRe_s;

    _frame.hasFrontSnapshot = true;

    startWaitingForVsolOk(nowMs);

    const int16_t hiLi_i100 = floatToInt100(hiLi_s);
    const int16_t hiRe_i100 = floatToInt100(hiRe_s);

    printVsol(out, frameId, resetPi, hiLi_i100, hiRe_i100);

    _lastSendMs = nowMs;

    return true;
}

void RearFrameClient::sendStop(Stream& out, uint32_t nowMs)
{
    const uint16_t frameId = nextFrameId();

    // resetPi=false reicht hier.
    // Bei Sollwert 0 fuehrt Rad::setSoll(0) hinten ohnehin einen harten Stop
    // mit Regler-Reset aus.
    printVsol(out, frameId, false, 0, 0);

    _lastSendMs = nowMs;
}

bool RearFrameClient::handleVsolOkLine(const char* line, uint32_t nowMs)
{
    VsolOkMessage vsolOk = {};

    if (!parseVsolOkLine(line, vsolOk))
    {
        return false;
    }

    if (!_waitingVsolOk)
    {
        return false;
    }

    if (!_frame.hasFrontSnapshot)
    {
        return false;
    }

    if (vsolOk.frameId != _frame.frameId)
    {
        return false;
    }

    startWaitingForVist(nowMs);
    return true;
}

bool RearFrameClient::handleVistLine(const char* line, VistMessage& msg)
{
    VistMessage vist = {};

    if (!parseVistLine(line, vist))
    {
        return false;
    }

    if (!_waitingVist)
    {
        return false;
    }

    if (!_frame.hasFrontSnapshot)
    {
        return false;
    }

    if (vist.frameId != _frame.frameId)
    {
        return false;
    }

    msg = vist;

    clearWaiting();
    _frame.hasFrontSnapshot = false;

    return true;
}

void RearFrameClient::armStopSequence()
{
    _stopSequenceArmed = true;
    _stopSendCount = 0;
}

void RearFrameClient::cancelStopSequence()
{
    _stopSequenceArmed = false;
    _stopSendCount = 0;
}

bool RearFrameClient::stopSequenceArmed() const
{
    return _stopSequenceArmed;
}

bool RearFrameClient::updateStopSequence(
    Stream& out,
    uint32_t nowMs,
    bool externalReady,
    uint32_t intervalMs)
{
    if (!_stopSequenceArmed)
    {
        return false;
    }

    if (!externalReady)
    {
        return false;
    }

    if (isBusy())
    {
        return false;
    }

    if (_stopSendCount >= STOP_SEND_MAX)
    {
        cancelStopSequence();
        return false;
    }

    if (_stopSendCount == 0 ||
        (uint32_t)(nowMs - _lastSendMs) >= intervalMs)
    {
        sendStop(out, nowMs);
        _stopSendCount++;

        if (_stopSendCount >= STOP_SEND_MAX)
        {
            _stopSequenceArmed = false;
        }

        return true;
    }

    return false;
}

void RearFrameClient::startWaitingForVsolOk(uint32_t nowMs)
{
    _waitingVsolOk = true;
    _waitingVist = false;
    _requestMs = nowMs;
}

void RearFrameClient::startWaitingForVist(uint32_t nowMs)
{
    _waitingVsolOk = false;
    _waitingVist = true;
    _requestMs = nowMs;
}

void RearFrameClient::clearWaiting()
{
    _waitingVsolOk = false;
    _waitingVist = false;
    _requestMs = 0;
}