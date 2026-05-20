// ============================================================
// CommandRunner.cpp
// ============================================================
#include "CommandRunner.h"

#include <math.h>

namespace
{
    const float CMDP_TOLERANCE_CM = 1.0f;
    const float CMDP_SPEED_EPS = 0.001f;
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
    _startTime(0),
    _durationMs(0),
    _pathTargetCm(0.0f),
    _pathUnitX(0.0f),
    _pathUnitY(0.0f),
    _pathProgressCm(0.0f)
{
}

void CommandRunner::begin()
{
    _cmdIndex = 0;
    _active = false;
    _finished = false;
    _startFramePending = false;
    _activeType = CMD_NONE;

    _startTime = 0;
    _durationMs = 0;

    _pathTargetCm = 0.0f;
    _pathUnitX = 0.0f;
    _pathUnitY = 0.0f;
    _pathProgressCm = 0.0f;
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
    _startFramePending = true;
}

bool CommandRunner::startPathCmd(const ParsedCommand& cmd, uint32_t now)
{
    if (cmd.wz != 0)
    {
        Serial.println(F("#ERROR,CMDP,wz_muss_0_sein"));
        return false;
    }

    if (cmd.param == 0)
    {
        Serial.println(F("#ERROR,CMDP,zielweg_muss_groesser_0_sein"));
        return false;
    }

    const float vx_cms = (float)cmd.vx;
    const float vy_cms = (float)cmd.vy;

    const float v_abs_cms = sqrtf(vx_cms * vx_cms + vy_cms * vy_cms);

    if (v_abs_cms <= CMDP_SPEED_EPS)
    {
        Serial.println(F("#ERROR,CMDP,vx_vy_duerfen_nicht_beide_0_sein"));
        return false;
    }

    _pathUnitX = vx_cms / v_abs_cms;
    _pathUnitY = vy_cms / v_abs_cms;
    _pathTargetCm = (float)cmd.param;
    _pathProgressCm = 0.0f;

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
    Serial.println(_pathUnitY, 3);

    const float vx = (float)cmd.vx * 0.01f;
    const float vy = (float)cmd.vy * 0.01f;
    const float wz = 0.0f;

    _vehicle.cmd(vx, vy, wz);

    _startTime = now;
    _durationMs = 0;

    _active = true;
    _activeType = CMD_PATH;
    _startFramePending = true;

    return true;
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
            updateActivePathCmd();
            return;
        }

        _active = false;
        _activeType = CMD_NONE;
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

void CommandRunner::updateActivePathCmd()
{
    updatePathProgress();

    if (!pathReached())
    {
        return;
    }

    finishPathCmd();
}

void CommandRunner::finishTimeCmd()
{
    _active = false;
    _activeType = CMD_NONE;
    _cmdIndex++;

    if (_cmdIndex >= _size())
    {
        stopAll();
        _finished = true;
    }
}

void CommandRunner::finishPathCmd()
{
    Serial.print(F("#EVENT,pathReached,targetCm="));
    Serial.print(_pathTargetCm, 2);
    Serial.print(F(",progressCm="));
    Serial.println(_pathProgressCm, 2);

    stopAll();

    _active = false;
    _activeType = CMD_NONE;
    _cmdIndex++;

    if (_cmdIndex >= _size())
    {
        _finished = true;
    }
}

void CommandRunner::updatePathProgress()
{
    _pathProgressCm =
        _odometer.xCm() * _pathUnitX +
        _odometer.yCm() * _pathUnitY;
}

bool CommandRunner::pathReached() const
{
    const float threshold = _pathTargetCm - CMDP_TOLERANCE_CM;

    if (threshold <= 0.0f)
    {
        return _pathProgressCm >= 0.0f;
    }

    return _pathProgressCm >= threshold;
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