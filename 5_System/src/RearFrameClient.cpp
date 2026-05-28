// RearFrameClient.cpp
#include "RearFrameClient.h"

#include <Arduino.h>
#include "src/CommProtocol.h"

namespace
{
    float cmsToMps(int16_t value_cms)
    {
        return (float)value_cms * 0.01f;
    }
}

RearFrameClient::RearFrameClient()
    : _nextFrameId(1),
    _frame(),
    _requestMs(0),
    _lastSendMs(0),
    _waitState(WAIT_NONE),
    _stopSendCount(0),
    _stopSequenceArmed(false),
    _hiLiIstCms(0),
    _hiReIstCms(0),
    _hiLiPwm(0),
    _hiRePwm(0),
    _hiLiCnt(0),
    _hiReCnt(0)
{
}

void RearFrameClient::begin()
{
    _nextFrameId = 1;
    _lastSendMs = 0;
    _stopSendCount = 0;
    _stopSequenceArmed = false;

    clearRearIst();
    clearFrame();
    clearWaiting();
}

uint16_t RearFrameClient::nextFrameId()
{
    const uint16_t id = _nextFrameId++;

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
    _frame.voLiCnt = 0;

    _frame.voRe_s = 0.0f;
    _frame.voRe_i = 0.0f;
    _frame.voRe_pwm = 0;
    _frame.voReCnt = 0;

    _frame.hiLi_s = 0.0f;
    _frame.hiRe_s = 0.0f;

    _frame.hiLiCnt = 0;
    _frame.hiReCnt = 0;

    _frame.hasFrontSnapshot = false;
}

void RearFrameClient::clearRearIst()
{
    _hiLiIstCms = 0;
    _hiReIstCms = 0;
    _hiLiPwm = 0;
    _hiRePwm = 0;
    _hiLiCnt = 0;
    _hiReCnt = 0;
}

bool RearFrameClient::waitingVsolOk() const
{
    return _waitState == WAIT_VSOL_OK;
}

bool RearFrameClient::waitingVist() const
{
    return _waitState == WAIT_VIST;
}

bool RearFrameClient::isBusy() const
{
    return _waitState != WAIT_NONE;
}

uint32_t RearFrameClient::requestMs() const
{
    return _requestMs;
}

uint32_t RearFrameClient::lastSendMs() const
{
    return _lastSendMs;
}

float RearFrameClient::hiLiIst() const
{
    return cmsToMps(_hiLiIstCms);
}

float RearFrameClient::hiReIst() const
{
    return cmsToMps(_hiReIstCms);
}

int16_t RearFrameClient::hiLiIstCms() const
{
    return _hiLiIstCms;
}

int16_t RearFrameClient::hiReIstCms() const
{
    return _hiReIstCms;
}

int16_t RearFrameClient::hiLiPwm() const
{
    return _hiLiPwm;
}

int16_t RearFrameClient::hiRePwm() const
{
    return _hiRePwm;
}

int32_t RearFrameClient::hiLiCnt() const
{
    return _hiLiCnt;
}

int32_t RearFrameClient::hiReCnt() const
{
    return _hiReCnt;
}

bool RearFrameClient::requestFrame(
    Stream& out,
    uint32_t nowMs,
    const RearFrameRequest& request)
{
    if (isBusy()) return false;

    const uint16_t frameId = nextFrameId();

    _frame.frameId = frameId;
    _frame.t = request.frameTimeMs;

    // Uebergang fuer den alten Printer:
    // RearFrameRequest ist bereits cm/s-Integer,
    // RearPendingFrame bleibt bis zum Printer-Umbau float/m/s.
    _frame.voLi_s = cmsToMps(request.voLi_s_cms);
    _frame.voLi_i = cmsToMps(request.voLi_i_cms);
    _frame.voLi_pwm = request.voLi_pwm;
    _frame.voLiCnt = request.voLiCnt;

    _frame.voRe_s = cmsToMps(request.voRe_s_cms);
    _frame.voRe_i = cmsToMps(request.voRe_i_cms);
    _frame.voRe_pwm = request.voRe_pwm;
    _frame.voReCnt = request.voReCnt;

    _frame.hiLi_s = cmsToMps(request.hiLi_s_cms);
    _frame.hiRe_s = cmsToMps(request.hiRe_s_cms);

    _frame.hiLiCnt = 0;
    _frame.hiReCnt = 0;

    _frame.hasFrontSnapshot = true;

    startWaitingForVsolOk(nowMs);

    printVsol(
        out,
        frameId,
        request.resetPi,
        request.hiLi_s_cms,
        request.hiRe_s_cms
    );

    _lastSendMs = nowMs;
    return true;
}

void RearFrameClient::sendStop(Stream& out, uint32_t nowMs)
{
    const uint16_t frameId = nextFrameId();

    printVsol(out, frameId, false, 0, 0);

    _lastSendMs = nowMs;
}

bool RearFrameClient::handleVsolOkLine(const char* line, uint32_t nowMs)
{
    VsolOkMessage vsolOk = {};

    if (!parseVsolOkLine(line, vsolOk)) return false;

    const bool valid =
        waitingVsolOk() &&
        _frame.hasFrontSnapshot &&
        vsolOk.frameId == _frame.frameId;

    if (!valid) return false;

    startWaitingForVist(nowMs);
    return true;
}

bool RearFrameClient::handleVistLine(const char* line)
{
    VistMessage vist = {};
    return handleVistLine(line, vist);
}

bool RearFrameClient::handleVistLine(const char* line, VistMessage& msg)
{
    VistMessage vist = {};

    if (!parseVistLine(line, vist)) return false;

    const bool valid =
        waitingVist() &&
        _frame.hasFrontSnapshot &&
        vist.frameId == _frame.frameId;

    if (!valid) return false;

    msg = vist;

    _hiLiIstCms = vist.hiLiIst;
    _hiReIstCms = vist.hiReIst;
    _hiLiPwm = vist.hiLiPwm;
    _hiRePwm = vist.hiRePwm;
    _hiLiCnt = vist.hiLiCnt;
    _hiReCnt = vist.hiReCnt;

    _frame.hiLiCnt = vist.hiLiCnt;
    _frame.hiReCnt = vist.hiReCnt;

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
    if (!_stopSequenceArmed || !externalReady || isBusy()) return false;

    if (_stopSendCount >= STOP_SEND_MAX)
    {
        cancelStopSequence();
        return false;
    }

    const bool sendDue =
        _stopSendCount == 0 ||
        (uint32_t)(nowMs - _lastSendMs) >= intervalMs;

    if (!sendDue) return false;

    sendStop(out, nowMs);
    _stopSendCount++;

    if (_stopSendCount >= STOP_SEND_MAX) _stopSequenceArmed = false;

    return true;
}

void RearFrameClient::startWaitingForVsolOk(uint32_t nowMs)
{
    _waitState = WAIT_VSOL_OK;
    _requestMs = nowMs;
}

void RearFrameClient::startWaitingForVist(uint32_t nowMs)
{
    _waitState = WAIT_VIST;
    _requestMs = nowMs;
}

void RearFrameClient::clearWaiting()
{
    _waitState = WAIT_NONE;
    _requestMs = 0;
}