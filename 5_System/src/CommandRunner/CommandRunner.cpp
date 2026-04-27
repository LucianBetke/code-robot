// CommandRunner.cpp
#include "CommandRunner.h"

CommandRunner::CommandRunner(VehicleController& vehicle, UartLink& uart, CommandParser& parser)
    : _vehicle(vehicle),
    _uart(uart),
    _parser(parser),
    _cmdIndex(0),
    _active(false),
    _finished(false),
    _startTime(0),
    _durationMs(0)
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

void CommandRunner::update()
{
    // Logik kommt im nächsten Schritt.
}