// ============================================================
// CommandRunner.cpp
// ============================================================

#include "CommandRunner.h"

#include "src/TelemetryPrinterConfig.h"
#include "src/RobotConfig.h"
#include "src/ScaleUtils.h"

#include <math.h>

namespace
{
    const uint8_t CMDP_TOLERANCE_CM = 2;
    const uint8_t CMDP_ANGLE_TOLERANCE_DEG = 2;

    const float CMDP_SPEED_EPS = 0.001f;

    // Timeout-Regel:
    // ideale Fahrzeit * 3 + 1000 ms Reserve
    const uint32_t CMDP_TIMEOUT_FACTOR = 3UL;
    const uint32_t CMDP_TIMEOUT_RESERVE_MS = 1000UL;
}

CommandRunner::CommandRunner(
    VehicleController& vehicle,
    MecanumOdometer& odometer,
    GetCmdFn getCmd,
    SizeFn size,
    Print& out)
    : _vehicle(vehicle),
    _odometer(odometer),
    _getCmd(getCmd),
    _size(size),
    _out(out),
    _cmdIndex(0),
    _active(false),
    _finished(false),
    _settleActive(false),
    _startFramePending(false),
    _pathMode(PATH_NONE),
    _startTime(0),
    _durationMs(0),
    _pathTargetCm(0),
    _pathUnitX(0.0f),
    _pathUnitY(0.0f),
    _pathProgressCm(0.0f),
    _angleTargetDeg(0),
    _angleDirection(0),
    _angleProgressDeg(0.0f),
    _nextCmdpId(1),
    _activeCmdpId(0)
{
}

void CommandRunner::begin()
{
    _cmdIndex = 0;
    _active = false;
    _finished = false;
    _settleActive = false;
    _startFramePending = false;
    _pathMode = PATH_NONE;

    _startTime = 0;
    _durationMs = 0;

    _pathTargetCm = 0;
    _pathUnitX = 0.0f;
    _pathUnitY = 0.0f;
    _pathProgressCm = 0.0f;

    _angleTargetDeg = 0;
    _angleDirection = 0;
    _angleProgressDeg = 0.0f;

    _nextCmdpId = 1;
    _activeCmdpId = 0;
}

void CommandRunner::stopAll()
{
    _vehicle.cmd(0.0f, 0.0f, 0.0f);
}

void CommandRunner::startSettlePhase(uint32_t now)
{
    if (CMDP_SETTLE_MS == 0)
    {
        return;
    }

    stopAll();

    _active = false;
    _pathMode = PATH_NONE;

    clearCmdpProtocol();

    _startTime = now;
    _durationMs = CMDP_SETTLE_MS;

    _settleActive = true;

    // Wichtig:
    // Die Settle-Phase ist KEIN neuer CMDP-Fahrabschnitt.
    // Deshalb darf sie keinen Startframe und keinen Messframe erzeugen.
    _startFramePending = false;
}

void CommandRunner::updateSettlePhase(uint32_t now)
{
    if ((uint32_t)(now - _startTime) < _durationMs)
    {
        return;
    }

    _settleActive = false;
    _durationMs = 0;
}

uint16_t CommandRunner::nextCmdpId()
{
    const uint16_t id = _nextCmdpId++;

    if (_nextCmdpId == 0)
    {
        _nextCmdpId = 1;
    }

    return id;
}

void CommandRunner::startCmdpProtocol(const ParsedCommand& cmd)
{
    _activeCmdpId = nextCmdpId();

#if PRINTER_ENABLE_ODOM
    _out.print(F("#CMDP_BEGIN,"));
    _out.print((unsigned int)_activeCmdpId);
    _out.print(',');
    _out.print((int)cmd.vx);
    _out.print(',');
    _out.print((int)cmd.vy);
    _out.print(',');
    _out.print((int)cmd.wz);
    _out.print(',');
    _out.println((unsigned int)cmd.param);
#endif
}

void CommandRunner::clearCmdpProtocol()
{
    _activeCmdpId = 0;
}

bool CommandRunner::startPathCmd(const ParsedCommand& cmd, uint32_t now)
{
    if (cmd.param == 0)
    {
#if PRINTER_ENABLE_ERRORS
        _out.println(F("#ERROR,CMDP,ziel_muss_groesser_0_sein"));
#endif
        return false;
    }

    const bool hasTranslation = (cmd.vx != 0 || cmd.vy != 0);
    const bool hasRotation = (cmd.wz != 0);

    if (!hasTranslation && !hasRotation)
    {
#if PRINTER_ENABLE_ERRORS
        _out.println(F("#ERROR,CMDP,vx_vy_wz_duerfen_nicht_alle_0_sein"));
#endif
        return false;
    }

    if (hasTranslation && hasRotation)
    {
#if PRINTER_ENABLE_ERRORS
        _out.println(F("#ERROR,CMDP,translation_und_drehung_noch_nicht_erlaubt"));
#endif
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
#if PRINTER_ENABLE_ERRORS
        _out.println(F("#ERROR,CMDP,vx_vy_duerfen_nicht_beide_0_sein"));
#endif
        return false;
    }

    startCmdpProtocol(cmd);

    _pathMode = PATH_TRANSLATION;

    _pathUnitX = vx_cms / v_abs_cms;
    _pathUnitY = vy_cms / v_abs_cms;
    _pathTargetCm = cmd.param;
    _pathProgressCm = 0.0f;

    _angleTargetDeg = 0;
    _angleDirection = 0;
    _angleProgressDeg = 0.0f;

    _durationMs = calcPathTimeoutMs(cmd.param, v_abs_cms);

#if PRINTER_ENABLE_EVENTS
    _out.print(F("#EVENT,startCmdp,vx="));
    _out.print(cmd.vx);
    _out.print(F(",vy="));
    _out.print(cmd.vy);
    _out.print(F(",wz="));
    _out.print(cmd.wz);
    _out.print(F(",targetCm="));
    _out.print(_pathTargetCm);
    _out.print(F(",unitX1000="));
    _out.print(scaleFloatToInt1000(_pathUnitX));
    _out.print(F(",unitY1000="));
    _out.print(scaleFloatToInt1000(_pathUnitY));
    _out.print(F(",timeoutMs="));
    _out.println(_durationMs);
#endif

    const float vx = (float)cmd.vx;
    const float vy = (float)cmd.vy;
    const float wz = 0.0f;

    _vehicle.cmd(vx, vy, wz);

    _startTime = now;
    _active = true;
    _startFramePending = true;

    return true;
}

bool CommandRunner::startRotationPathCmd(const ParsedCommand& cmd, uint32_t now)
{
    const int16_t wz_deg_s = cmd.wz;

    uint16_t wz_abs_deg_s = 0;

    if (wz_deg_s < 0)
    {
        wz_abs_deg_s = (uint16_t)(-(int32_t)wz_deg_s);
    }
    else
    {
        wz_abs_deg_s = (uint16_t)wz_deg_s;
    }

    if (wz_abs_deg_s == 0)
    {
#if PRINTER_ENABLE_ERRORS
        _out.println(F("#ERROR,CMDP,wz_darf_nicht_0_sein"));
#endif
        return false;
    }

    startCmdpProtocol(cmd);

    _pathMode = PATH_ROTATION;

    _pathTargetCm = 0;
    _pathUnitX = 0.0f;
    _pathUnitY = 0.0f;
    _pathProgressCm = 0.0f;

    _angleTargetDeg = cmd.param;
    _angleDirection = (wz_deg_s >= 0) ? 1 : -1;
    _angleProgressDeg = 0.0f;

    _durationMs = calcAngleTimeoutMs(cmd.param, wz_abs_deg_s);

#if PRINTER_ENABLE_EVENTS
    _out.print(F("#EVENT,startCmdpTurn,wz="));
    _out.print(cmd.wz);
    _out.print(F(",targetDeg="));
    _out.print(_angleTargetDeg);
    _out.print(F(",dir="));
    _out.print((int)_angleDirection);
    _out.print(F(",timeoutMs="));
    _out.println(_durationMs);
#endif

    const float vx = 0.0f;
    const float vy = 0.0f;
    const float wz = (float)wz_deg_s * DEG_TO_RAD;

    _vehicle.cmd(vx, vy, wz);

    _startTime = now;
    _active = true;
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

uint32_t CommandRunner::calcAngleTimeoutMs(uint16_t targetDeg, uint16_t speedDegS) const
{
    if (speedDegS == 0)
    {
        speedDegS = 1;
    }

    const uint32_t idealMs =
        ((uint32_t)targetDeg * 1000UL + ((uint32_t)speedDegS / 2UL)) /
        (uint32_t)speedDegS;

    return idealMs * CMDP_TIMEOUT_FACTOR + CMDP_TIMEOUT_RESERVE_MS;
}

void CommandRunner::update(uint32_t now)
{
    if (_finished)
    {
        return;
    }

    if (_settleActive)
    {
        updateSettlePhase(now);

        if (_settleActive)
        {
            return;
        }
    }

    if (_active)
    {
        updateActivePathCmd(now);
        return;
    }

    while (_cmdIndex < _size())
    {
        ParsedCommand cmd;

        if (!CommandParser::parse(_getCmd(_cmdIndex), cmd))
        {
#if PRINTER_ENABLE_ERRORS
            _out.print(F("#ERROR,parse,index="));
            _out.println(_cmdIndex);
#endif
            _cmdIndex++;
            continue;
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
    clearCmdpProtocol();
    _finished = true;
}

void CommandRunner::updateActivePathCmd(uint32_t now)
{
    updatePathProgress();

    if (pathReached())
    {
        finishPathCmd(now);
        return;
    }

    if (pathTimedOut(now))
    {
        finishPathTimeoutCmd(now);
        return;
    }
}

void CommandRunner::finishPathCmd(uint32_t now)
{
#if PRINTER_ENABLE_EVENTS
    if (_pathMode == PATH_ROTATION)
    {
        _out.print(F("#EVENT,angleReached,targetDeg="));
        _out.print(_angleTargetDeg);
        _out.print(F(",progressDeg100="));
        _out.println(scaleFloatToInt100(_angleProgressDeg));
    }
    else
    {
        _out.print(F("#EVENT,pathReached,targetCm="));
        _out.print(_pathTargetCm);
        _out.print(F(",progressCm100="));
        _out.println(scaleFloatToInt100(_pathProgressCm));
    }
#endif

    stopAll();

    _active = false;
    _pathMode = PATH_NONE;
    clearCmdpProtocol();
    _cmdIndex++;

    if (_cmdIndex >= _size())
    {
        _finished = true;
        return;
    }

    startSettlePhase(now);
}

void CommandRunner::finishPathTimeoutCmd(uint32_t now)
{
#if PRINTER_ENABLE_ERRORS
    if (_pathMode == PATH_ROTATION)
    {
        _out.print(F("#ERROR,CMDP,timeout,targetDeg="));
        _out.print(_angleTargetDeg);
        _out.print(F(",progressDeg100="));
        _out.print(scaleFloatToInt100(_angleProgressDeg));
        _out.print(F(",timeoutMs="));
        _out.println(_durationMs);
    }
    else
    {
        _out.print(F("#ERROR,CMDP,timeout,targetCm="));
        _out.print(_pathTargetCm);
        _out.print(F(",progressCm100="));
        _out.print(scaleFloatToInt100(_pathProgressCm));
        _out.print(F(",timeoutMs="));
        _out.println(_durationMs);
    }
#endif

    stopAll();

    _active = false;
    _pathMode = PATH_NONE;
    clearCmdpProtocol();
    _cmdIndex++;

    if (_cmdIndex >= _size())
    {
        _finished = true;
        return;
    }

    startSettlePhase(now);
}

void CommandRunner::updatePathProgress()
{
    if (_pathMode == PATH_ROTATION)
    {
        _angleProgressDeg = _odometer.phiDeg() * (float)_angleDirection;
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
        const int16_t threshold =
            (int16_t)_angleTargetDeg - (int16_t)CMDP_ANGLE_TOLERANCE_DEG;

        if (threshold <= 0)
        {
            return _angleProgressDeg >= 0.0f;
        }

        return _angleProgressDeg >= (float)threshold;
    }

    const int16_t threshold =
        (int16_t)_pathTargetCm - (int16_t)CMDP_TOLERANCE_CM;

    if (threshold <= 0)
    {
        return _pathProgressCm >= 0.0f;
    }

    return _pathProgressCm >= (float)threshold;
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
    return _active || _settleActive;
}

bool CommandRunner::isFinished() const
{
    return _finished;
}

bool CommandRunner::hasActivePathCommand() const
{
    return _active && _activeCmdpId != 0;
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