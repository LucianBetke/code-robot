// ============================================================
// CommandRunner.h
// ============================================================
#ifndef COMMAND_RUNNER_H
#define COMMAND_RUNNER_H

#include <Arduino.h>
#include "src/Parser/CommandParser.h"
#include "src/UartLink.h"
#include "src/VehicleController.h"

typedef const char* (*GetCmdFn)(uint8_t index);
typedef uint8_t(*SizeFn)();

class CommandRunner
{
public:
    CommandRunner(VehicleController& vehicle, UartLink& uart, CommandParser& parser,
        GetCmdFn getCmd, SizeFn size);
    void begin();
    void update(uint32_t now);

    float getWheelSoll(WheelVehicle w) const;
    bool  isActive() const;

private:
    void startCmd(const ParsedCommand& cmd, uint32_t now);
    void stopAll();

    VehicleController& _vehicle;
    UartLink& _uart;
    CommandParser& _parser;
    GetCmdFn           _getCmd;
    SizeFn             _size;

    uint8_t   _cmdIndex;
    bool      _active;
    bool      _finished;
    uint32_t  _startTime;
    uint32_t  _durationMs;
};

#endif