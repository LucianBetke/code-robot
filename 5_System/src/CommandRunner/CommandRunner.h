// ============================================================
// CommandRunner.h
// ============================================================
#ifndef COMMAND_RUNNER_H
#define COMMAND_RUNNER_H

#include <Arduino.h>
#include "src/Parser/CommandParser.h"
#include "src/UartLink.h"
#include "src/VehicleController.h"
#include "src/MecanumOdometer.h"

typedef const char* (*GetCmdFn)(uint8_t index);
typedef uint8_t(*SizeFn)();

class CommandRunner
{
public:
    CommandRunner(
        VehicleController& vehicle,
        MecanumOdometer& odometer,
        UartLink& uart,
        CommandParser& parser,
        GetCmdFn getCmd,
        SizeFn size);

    void begin();
    void update(uint32_t now);

    float getWheelSoll(WheelVehicle w) const;
    bool  isActive() const;
    bool  isFinished() const;

    bool consumeStartFramePending();

    float pathProgressCm() const { return _pathProgressCm; }
    float pathTargetCm() const { return _pathTargetCm; }

private:
    void startTimeCmd(const ParsedCommand& cmd, uint32_t now);
    bool startPathCmd(const ParsedCommand& cmd, uint32_t now);

    void updateActiveTimeCmd(uint32_t now);
    void updateActivePathCmd();

    void finishTimeCmd();
    void finishPathCmd();

    bool pathReached() const;
    void updatePathProgress();

    void stopAll();

    VehicleController& _vehicle;
    MecanumOdometer& _odometer;
    UartLink& _uart;
    CommandParser& _parser;
    GetCmdFn _getCmd;
    SizeFn _size;

    uint8_t _cmdIndex;
    bool _active;
    bool _finished;
    bool _startFramePending;

    CmdType _activeType;

    uint32_t _startTime;
    uint32_t _durationMs;

    float _pathTargetCm;
    float _pathUnitX;
    float _pathUnitY;
    float _pathProgressCm;
};

#endif