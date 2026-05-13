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
}

void CommandRunner::startCmd(const ParsedCommand& cmd, uint32_t now)
{
    _durationMs = (uint32_t)(cmd.param * 1000.0f);

    Serial.print(F("#EVENT,startCmd,param="));
    Serial.print(cmd.param, 2);
    Serial.print(F(",durationMs="));
    Serial.println(_durationMs);

    _vehicle.cmd(cmd.vx, cmd.vy, cmd.wz);

    _startTime = now;
    _active = true;
}

void CommandRunner::update(uint32_t now)
{
    if (_finished)
    {
        return;
    }

    // --------------------------------------------------------
    // Aktiver Befehl laeuft.
    // --------------------------------------------------------

    if (_active)
    {
        if (now - _startTime < _durationMs)
        {
            return;
        }

        // Befehl ist zeitlich fertig.
        // Wichtig:
        // NICHT sofort stopAll(), wenn danach noch ein weiterer Befehl kommt.
        // Sonst entsteht zwischen zwei CMDT-Befehlen ein kuenstlicher
        // Null-Sollwert.
        _active = false;
        _cmdIndex++;

        if (_cmdIndex >= _size())
        {
            // Nur wenn das ganze Script fertig ist, wirklich stoppen.
            stopAll();
            _finished = true;
        }

        return;
    }

    // --------------------------------------------------------
    // Kein aktiver Befehl:
    // Naechsten gueltigen Befehl suchen und starten.
    // --------------------------------------------------------

    while (_cmdIndex < _size())
    {
        ParsedCommand cmd;

        if (!CommandParser::parse(_getCmd(_cmdIndex), cmd))
        {
            _cmdIndex++;
            continue;
        }

        if (cmd.type == CMD_TIME)
        {
            startCmd(cmd, now);
            return;
        }

        _cmdIndex++;
    }

    // --------------------------------------------------------
    // Kein weiterer gueltiger Befehl vorhanden.
    // --------------------------------------------------------

    stopAll();
    _finished = true;
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