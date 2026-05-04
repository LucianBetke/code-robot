// ============================================================
// CommandRunner.cpp
// ============================================================
#include "CommandRunner.h"

CommandRunner::CommandRunner(VehicleController& vehicle, UartLink& uart, CommandParser& parser,
    GetCmdFn getCmd, SizeFn size)
    : _vehicle(vehicle), _uart(uart), _parser(parser),
    _getCmd(getCmd), _size(size),
    _cmdIndex(0), _active(false), _finished(false),
    _startTime(0), _durationMs(0)
{
}

void CommandRunner::begin()
{
    _cmdIndex = 0;
    _active = false;
    _finished = false;
    _startTime = 0;
    _durationMs = 0;
}

void CommandRunner::stopAll()
{
    _vehicle.cmd(0.0f, 0.0f, 0.0f);
    _uart.sendLine("VSOL,0,0");
}

void CommandRunner::startCmd(const ParsedCommand& cmd, uint32_t now)
{
    Serial.print(F("#startCmd param="));
    Serial.print(cmd.param);
    Serial.print(F(" durationMs="));
    Serial.println((uint32_t)(cmd.param * 1000.0f));
    _vehicle.cmd(cmd.vx, cmd.vy, cmd.wz);
    _startTime = now;
    _durationMs = (uint32_t)(cmd.param * 1000.0f);
    _active = true;
}

void CommandRunner::update(uint32_t now)
{
    if (!_finished)
    {
        if (_active)
        {
            if (now - _startTime >= _durationMs)
            {
                stopAll();
                _active = false;
                _cmdIndex++;
            }
        }
        else
        {
            if (_cmdIndex >= _size()) _finished = true;
            else
            {
                ParsedCommand cmd;
                if (!CommandParser::parse(_getCmd(_cmdIndex), cmd)) _cmdIndex++;
                else if (cmd.type == CMD_TIME) startCmd(cmd, now);
            }
        }
    }
}

float CommandRunner::getWheelSoll(WheelVehicle w) const
{
    return _vehicle.getWheelSoll(w);
}

bool CommandRunner::isActive() const
{
    return _active;
}