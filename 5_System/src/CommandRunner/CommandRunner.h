// CommandRunner.h
#pragma once

#include <Arduino.h>
#include "src/Parser/CommandParser.h"
#include "src/UartLink.h"
#include "src/VehicleController.h"

class CommandRunner
{
public:
    CommandRunner(VehicleController& vehicle, UartLink& uart, CommandParser& parser);

    void begin();
    void update();

private:
    VehicleController& _vehicle;
    UartLink& _uart;
    CommandParser& _parser;

    uint8_t _cmdIndex;
    bool _active;
    bool _finished;
    uint32_t _startTime;
    uint32_t _durationMs;
};