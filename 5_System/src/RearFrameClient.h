// RearFrameClient.h
#pragma once

#include <stdint.h>

// ============================================================
// Gemeinsamer Log-Datensatz fuer einen Zeitpunkt
// ============================================================

struct RearPendingFrame
{
    uint16_t frameId;
    uint32_t t;

    float voLi_s;
    float voLi_i;
    int16_t voLi_pwm;

    float voRe_s;
    float voRe_i;
    int16_t voRe_pwm;

    float hiLi_s;
    float hiRe_s;

    bool hasFrontSnapshot;
};

// ============================================================
// RearFrameClient
// Verwaltet den aktuellen Hinterachs-Messframe,
// die Frame-ID und den Wartezustand auf VSOL_OK / VIST.
// ============================================================

class RearFrameClient
{
public:
    RearFrameClient();

    void begin();

    uint16_t nextFrameId();

    RearPendingFrame& frame();
    const RearPendingFrame& frame() const;

    void clearFrame();

    bool waitingVsolOk() const;
    bool waitingVist() const;
    bool isBusy() const;
    uint32_t requestMs() const;

    void startWaitingForVsolOk(uint32_t nowMs);
    void startWaitingForVist(uint32_t nowMs);
    void clearWaiting();

private:
    uint16_t _nextFrameId;
    RearPendingFrame _frame;

    uint32_t _requestMs;
    bool _waitingVsolOk;
    bool _waitingVist;
};