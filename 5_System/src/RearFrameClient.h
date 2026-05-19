// RearFrameClient.h
#ifndef REAR_FRAME_CLIENT_H
#define REAR_FRAME_CLIENT_H

#include <stdint.h>

class Stream;
struct VistMessage;

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
// Anfrage-Datensatz fuer einen neuen RearFrame
// ============================================================

struct RearFrameRequest
{
    uint32_t frameTimeMs;
    bool resetPi;

    float voLi_s;
    float voLi_i;
    int16_t voLi_pwm;

    float voRe_s;
    float voRe_i;
    int16_t voRe_pwm;

    float hiLi_s;
    float hiRe_s;
};

// ============================================================
// RearFrameClient
// Verwaltet:
//  - aktuellen Hinterachs-Messframe
//  - Frame-ID
//  - Wartezustand auf VSOL_OK / VIST
//  - Stop-Sequenz fuer hinten
//  - letzte gueltige Rear-Istwerte
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
    uint32_t lastSendMs() const;

    float hiLiIst() const;
    float hiReIst() const;
    int16_t hiLiPwm() const;
    int16_t hiRePwm() const;

    bool requestFrame(
        Stream& out,
        uint32_t nowMs,
        const RearFrameRequest& request);

    void sendStop(Stream& out, uint32_t nowMs);

    bool handleVsolOkLine(const char* line, uint32_t nowMs);

    // Variante fuer vorne.ino:
    // VIST wird intern verarbeitet und die Rear-Istwerte werden gespeichert.
    bool handleVistLine(const char* line);

    // Variante bleibt erhalten, falls spaeter ein Aufrufer die Rohdaten braucht.
    bool handleVistLine(const char* line, VistMessage& msg);

    void armStopSequence();
    void cancelStopSequence();
    bool stopSequenceArmed() const;

    bool updateStopSequence(
        Stream& out,
        uint32_t nowMs,
        bool externalReady,
        uint32_t intervalMs);

    void startWaitingForVsolOk(uint32_t nowMs);
    void startWaitingForVist(uint32_t nowMs);
    void clearWaiting();

private:
    static const uint8_t STOP_SEND_MAX = 3;

    void clearRearIst();

    uint16_t _nextFrameId;
    RearPendingFrame _frame;

    uint32_t _requestMs;
    uint32_t _lastSendMs;

    bool _waitingVsolOk;
    bool _waitingVist;

    uint8_t _stopSendCount;
    bool _stopSequenceArmed;

    float _hiLiIst;
    float _hiReIst;
    int16_t _hiLiPwm;
    int16_t _hiRePwm;
};

#endif // REAR_FRAME_CLIENT_H