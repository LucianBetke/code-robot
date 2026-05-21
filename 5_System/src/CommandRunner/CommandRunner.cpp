// ============================================================
// CommandRunner.cpp
// ============================================================
#include "CommandRunner.h"

#include <math.h>

namespace
{
    const float CMDP_TOLERANCE_CM = 1.0f;
    const float CMDP_ANGLE_TOLERANCE_DEG = 2.0f;

    const float CMDP_SPEED_EPS = 0.001f;
    const float CMDP_ROT_SPEED_EPS = 0.001f;

    // Timeout-Regel:
    // ideale Fahrzeit * 3 + 1000 ms Reserve
    const uint32_t CMDP_TIMEOUT_FACTOR = 3UL;
    const uint32_t CMDP_TIMEOUT_RESERVE_MS = 1000UL;
}

CommandRunner::CommandRunner(
    VehicleController& vehicle,
    MecanumOdometer& odometer,
    UartLink& uart,
    CommandParser& parser,
    GetCmdFn getCmd,
    SizeFn size)
    : _vehicle(vehicle),
    _odometer(odometer),
    _uart(uart),
    _parser(parser),
    _getCmd(getCmd),
    _size(size),
    _cmdIndex(0),
    _active(false),
    _finished(false),
    _startFramePending(false),
    _activeType(CMD_NONE),
    _pathMode(PATH_NONE),
    _startTime(0),
    _durationMs(0),
    _pathTargetCm(0.0f),
    _pathUnitX(0.0f),
    _pathUnitY(0.0f),
    _pathProgressCm(0.0f),
    _angleTargetDeg(0.0f),
    _angleDirection(0.0f),
    _angleProgressDeg(0.0f)
{
}

void CommandRunner::begin()
{
    _cmdIndex = 0;
    _active = false;
    _finished = false;
    _startFramePending = false;
    _activeType = CMD_NONE;
    _pathMode = PATH_NONE;

    _startTime = 0;
    _durationMs = 0;

    _pathTargetCm = 0.0f;
    _pathUnitX = 0.0f;
    _pathUnitY = 0.0f;
    _pathProgressCm = 0.0f;

    _angleTargetDeg = 0.0f;
    _angleDirection = 0.0f;
    _angleProgressDeg = 0.0f;
}

void CommandRunner::stopAll()
{
    _vehicle.cmd(0.0f, 0.0f, 0.0f);
}

void CommandRunner::startTimeCmd(const ParsedCommand& cmd, uint32_t now)
{
    _durationMs = (uint32_t)cmd.param * 1000UL;

    Serial.print(F("#EVENT,startCmd,vx="));
    Serial.print(cmd.vx);
    Serial.print(F(",vy="));
    Serial.print(cmd.vy);
    Serial.print(F(",wz="));
    Serial.print(cmd.wz);
    Serial.print(F(",duration="));
    Serial.print(cmd.param);
    Serial.print(F(",durationMs="));
    Serial.println(_durationMs);

    const float vx = (float)cmd.vx * 0.01f;
    const float vy = (float)cmd.vy * 0.01f;
    const float wz = (float)cmd.wz * DEG_TO_RAD;

    _vehicle.cmd(vx, vy, wz);

    _startTime = now;
    _active = true;
    _activeType = CMD_TIME;
    _pathMode = PATH_NONE;
    _startFramePending = true;
}

bool CommandRunner::startPathCmd(const ParsedCommand& cmd, uint32_t now)
{
    if (cmd.param == 0)
    {
        Serial.println(F("#ERROR,CMDP,ziel_muss_groesser_0_sein"));
        return false;
    }

    const bool hasTranslation = (cmd.vx != 0 || cmd.vy != 0);
    const bool hasRotation = (cmd.wz != 0);

    if (!hasTranslation && !hasRotation)
    {
        Serial.println(F("#ERROR,CMDP,vx_vy_wz_duerfen_nicht_alle_0_sein"));
        return false;
    }

    if (hasTranslation && hasRotation)
    {
        Serial.println(F("#ERROR,CMDP,translation_und_drehung_noch_nicht_erlaubt"));
        return false;
    }

    if (hasTranslation)
    {
        return startTranslationPathCmd(cmd, now);
    }

    return startRotationPathCmd(cmd, now);
}

bool CommandRunner::startTranslationPathCmd(const ParsedCommand& cmd, uint32_t now)
{
    const float vx_cms = (float)cmd.vx;
    const float vy_cms = (float)cmd.vy;

    const float v_abs_cms = sqrtf(vx_cms * vx_cms + vy_cms * vy_cms);

    if (v_abs_cms <= CMDP_SPEED_EPS)
    {
        Serial.println(F("#ERROR,CMDP,vx_vy_duerfen_nicht_beide_0_sein"));
        return false;
    }

    _pathMode = PATH_TRANSLATION;

    _pathUnitX = vx_cms / v_abs_cms;
    _pathUnitY = vy_cms / v_abs_cms;
    _pathTargetCm = (float)cmd.param;
    _pathProgressCm = 0.0f;

    _angleTargetDeg = 0.0f;
    _angleDirection = 0.0f;
    _angleProgressDeg = 0.0f;

    _durationMs = calcPathTimeoutMs(cmd.param, v_abs_cms);

    Serial.print(F("#EVENT,startCmdp,vx="));
    Serial.print(cmd.vx);
    Serial.print(F(",vy="));
    Serial.print(cmd.vy);
    Serial.print(F(",wz="));
    Serial.print(cmd.wz);
    Serial.print(F(",targetCm="));
    Serial.print(cmd.param);
    Serial.print(F(",unitX="));
    Serial.print(_pathUnitX, 3);
    Serial.print(F(",unitY="));
    Serial.print(_pathUnitY, 3);
    Serial.print(F(",timeoutMs="));
    Serial.println(_durationMs);

    const float vx = (float)cmd.vx * 0.01f;
    const float vy = (float)cmd.vy * 0.01f;
    const float wz = 0.0f;

    _vehicle.cmd(vx, vy, wz);

    _startTime = now;
    _active = true;
    _activeType = CMD_PATH;
    _startFramePending = true;

    return true;
}

bool CommandRunner::startRotationPathCmd(const ParsedCommand& cmd, uint32_t now)
{
    const float wz_deg_s = (float)cmd.wz;
    const float wz_abs_deg_s = fabsf(wz_deg_s);

    if (wz_abs_deg_s <= CMDP_ROT_SPEED_EPS)
    {
        Serial.println(F("#ERROR,CMDP,wz_darf_nicht_0_sein"));
        return false;
    }

    _pathMode = PATH_ROTATION;

    _pathTargetCm = 0.0f;
    _pathUnitX = 0.0f;
    _pathUnitY = 0.0f;
    _pathProgressCm = 0.0f;

    _angleTargetDeg = (float)cmd.param;
    _angleDirection = (wz_deg_s >= 0.0f) ? 1.0f : -1.0f;
    _angleProgressDeg = 0.0f;

    _durationMs = calcAngleTimeoutMs(cmd.param, wz_abs_deg_s);

    Serial.print(F("#EVENT,startCmdpTurn,wz="));
    Serial.print(cmd.wz);
    Serial.print(F(",targetDeg="));
    Serial.print(cmd.param);
    Serial.print(F(",dir="));
    Serial.print((int)_angleDirection);
    Serial.print(F(",timeoutMs="));
    Serial.println(_durationMs);

    const float vx = 0.0f;
    const float vy = 0.0f;
    const float wz = wz_deg_s * DEG_TO_RAD;

    _vehicle.cmd(vx, vy, wz);

    _startTime = now;
    _active = true;
    _activeType = CMD_PATH;
    _startFramePending = true;

    return true;
}

uint32_t CommandRunner::calcPathTimeoutMs(uint16_t targetCm, float speedCms) const
{
    if (speedCms < 1.0f)
    {
        speedCms = 1.0f;
    }

    const float idealMsF = ((float)targetCm * 1000.0f) / speedCms;
    const uint32_t idealMs = (uint32_t)(idealMsF + 0.5f);

    return idealMs * CMDP_TIMEOUT_FACTOR + CMDP_TIMEOUT_RESERVE_MS;
}

uint32_t CommandRunner::calcAngleTimeoutMs(uint16_t targetDeg, float speedDegS) const
{
    if (speedDegS < 1.0f)
    {
        speedDegS = 1.0f;
    }

    const float idealMsF = ((float)targetDeg * 1000.0f) / speedDegS;
    const uint32_t idealMs = (uint32_t)(idealMsF + 0.5f);

    return idealMs * CMDP_TIMEOUT_FACTOR + CMDP_TIMEOUT_RESERVE_MS;
}

void CommandRunner::update(uint32_t now)
{
    if (_finished)
    {
        return;
    }

    if (_active)
    {
        if (_activeType == CMD_TIME)
        {
            updateActiveTimeCmd(now);
            return;
        }

        if (_activeType == CMD_PATH)
        {
            updateActivePathCmd(now);
            return;
        }

        _active = false;
        _activeType = CMD_NONE;
        _pathMode = PATH_NONE;
    }

    while (_cmdIndex < _size())
    {
        ParsedCommand cmd;

        if (!CommandParser::parse(_getCmd(_cmdIndex), cmd))
        {
            Serial.print(F("#ERROR,parse,index="));
            Serial.println(_cmdIndex);
            _cmdIndex++;
            continue;
        }

        if (cmd.type == CMD_TIME)
        {
            startTimeCmd(cmd, now);
            return;
        }

        if (cmd.type == CMD_PATH)
        {
            if (startPathCmd(cmd, now))
            {
                return;
            }

            _cmdIndex++;
            continue;
        }

        _cmdIndex++;
    }

    stopAll();
    _finished = true;
}

void CommandRunner::updateActiveTimeCmd(uint32_t now)
{
    if (now - _startTime < _durationMs)
    {
        return;
    }

    finishTimeCmd();
}

void CommandRunner::updateActivePathCmd(uint32_t now)
{
    updatePathProgress();

    if (pathReached())
    {
        finishPathCmd();
        return;
    }

    if (pathTimedOut(now))
    {
        finishPathTimeoutCmd();
        return;
    }
}

void CommandRunner::finishTimeCmd()
{
    _active = false;
    _activeType = CMD_NONE;
    _pathMode = PATH_NONE;
    _cmdIndex++;

    if (_cmdIndex >= _size())
    {
        stopAll();
        _finished = true;
    }
}

void CommandRunner::finishPathCmd()
{
    if (_pathMode == PATH_ROTATION)
    {
        Serial.print(F("#EVENT,angleReached,targetDeg="));
        Serial.print(_angleTargetDeg, 2);
        Serial.print(F(",progressDeg="));
        Serial.println(_angleProgressDeg, 2);
    }
    else
    {
        Serial.print(F("#EVENT,pathReached,targetCm="));
        Serial.print(_pathTargetCm, 2);
        Serial.print(F(",progressCm="));
        Serial.println(_pathProgressCm, 2);
    }

    stopAll();

    _active = false;
    _activeType = CMD_NONE;
    _pathMode = PATH_NONE;
    _cmdIndex++;

    if (_cmdIndex >= _size())
    {
        _finished = true;
    }
}

void CommandRunner::finishPathTimeoutCmd()
{
    if (_pathMode == PATH_ROTATION)
    {
        Serial.print(F("#ERROR,CMDP,timeout,targetDeg="));
        Serial.print(_angleTargetDeg, 2);
        Serial.print(F(",progressDeg="));
        Serial.print(_angleProgressDeg, 2);
        Serial.print(F(",timeoutMs="));
        Serial.println(_durationMs);
    }
    else
    {
        Serial.print(F("#ERROR,CMDP,timeout,targetCm="));
        Serial.print(_pathTargetCm, 2);
        Serial.print(F(",progressCm="));
        Serial.print(_pathProgressCm, 2);
        Serial.print(F(",timeoutMs="));
        Serial.println(_durationMs);
    }

    stopAll();

    _active = false;
    _activeType = CMD_NONE;
    _pathMode = PATH_NONE;
    _cmdIndex++;

    if (_cmdIndex >= _size())
    {
        _finished = true;
    }
}

void CommandRunner::updatePathProgress()
{
    if (_pathMode == PATH_ROTATION)
    {
        _angleProgressDeg = _odometer.phiDeg() * _angleDirection;
        return;
    }

    _pathProgressCm =
        _odometer.xCm() * _pathUnitX +
        _odometer.yCm() * _pathUnitY;
}

bool CommandRunner::pathReached() const
{
    if (_pathMode == PATH_ROTATION)
    {
        const float threshold = _angleTargetDeg - CMDP_ANGLE_TOLERANCE_DEG;

        if (threshold <= 0.0f)
        {
            return _angleProgressDeg >= 0.0f;
        }

        return _angleProgressDeg >= threshold;
    }

    const float threshold = _pathTargetCm - CMDP_TOLERANCE_CM;

    if (threshold <= 0.0f)
    {
        return _pathProgressCm >= 0.0f;
    }

    return _pathProgressCm >= threshold;
}

bool CommandRunner::pathTimedOut(uint32_t now) const
{
    return (uint32_t)(now - _startTime) >= _durationMs;
}

float CommandRunner::getWheelSoll(WheelVehicle w) const
{
    return _vehicle.getWheelSoll(w);
}

bool CommandRunner::isActive() const
{
    return _active;
}

bool CommandRunner::isFinished() const
{
    return _finished;
}

bool CommandRunner::consumeStartFramePending()
{
    if (!_startFramePending)
    {
        return false;
    }

    _startFramePending = false;
    return true;
}