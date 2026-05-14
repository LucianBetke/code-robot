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

    // Integer-Protokoll:
    // vx, vy kommen aus dem Script als cm/s.
    // VehicleController arbeitet weiterhin mit m/s.
    //
    // wz kommt aus dem Script als Grad/s.
    // VehicleController arbeitet weiterhin mit rad/s.
    const float vx = (float)cmd.vx * 0.01f;
    const float vy = (float)cmd.vy * 0.01f;
    const float wz = (float)cmd.wz * 0.01745329252f;

    _vehicle.cmd(vx, vy, wz);

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