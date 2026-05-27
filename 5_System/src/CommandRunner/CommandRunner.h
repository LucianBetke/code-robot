// ============================================================
// CommandRunner.h
// ============================================================
#ifndef COMMAND_RUNNER_H
#define COMMAND_RUNNER_H

#include <Arduino.h>
#include "src/Parser/CommandParser.h"
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
        GetCmdFn getCmd,
        SizeFn size);

    void begin();
    void update(uint32_t now);

    float getWheelSoll(WheelVehicle w) const;
    bool  isActive() const;
    bool  isFinished() const;

    bool consumeStartFramePending();

    float pathProgressCm() const { return _pathProgressCm; }
    float pathTargetCm() const { return (float)_pathTargetCm; }

    float angleProgressDeg() const { return _angleProgressDeg; }
    float angleTargetDeg() const { return (float)_angleTargetDeg; }

    uint16_t activeCmdpId() const { return _activeCmdpId; }
    bool hasActivePathCommand() const;

private:
    enum PathMode : uint8_t
    {
        PATH_NONE = 0,
        PATH_TRANSLATION = 1,
        PATH_ROTATION = 2
    };

    bool startPathCmd(const ParsedCommand& cmd, uint32_t now);

    bool startTranslationPathCmd(const ParsedCommand& cmd, uint32_t now);
    bool startRotationPathCmd(const ParsedCommand& cmd, uint32_t now);

    void updateActivePathCmd(uint32_t now);
    void updateSettlePhase(uint32_t now);

    void finishPathCmd(uint32_t now);
    void finishPathTimeoutCmd(uint32_t now);

    bool pathReached() const;
    bool pathTimedOut(uint32_t now) const;
    void updatePathProgress();

    uint32_t calcPathTimeoutMs(uint16_t targetCm, float speedCms) const;
    uint32_t calcAngleTimeoutMs(uint16_t targetDeg, float speedDegS) const;

    void stopAll();

    void startSettlePhase(uint32_t now);

    uint16_t nextCmdpId();
    void startCmdpProtocol(const ParsedCommand& cmd);
    void clearCmdpProtocol();

    VehicleController& _vehicle;
    MecanumOdometer& _odometer;
    GetCmdFn _getCmd;
    SizeFn _size;

    uint8_t _cmdIndex;
    bool _active;
    bool _finished;
    bool _settleActive;
    bool _startFramePending;

    PathMode _pathMode;

    uint32_t _startTime;
    uint32_t _durationMs;

    uint16_t _pathTargetCm;
    float _pathUnitX;
    float _pathUnitY;
    float _pathProgressCm;

    uint16_t _angleTargetDeg;
    int8_t _angleDirection;
    float _angleProgressDeg;

    uint16_t _nextCmdpId;
    uint16_t _activeCmdpId;
};

#endif