// RearFrameClient.h
#ifndef REAR_FRAME_CLIENT_H
#define REAR_FRAME_CLIENT_H

#include <stdint.h>

class Stream;
struct VistMessage;

// ============================================================
// Gemeinsamer Log-Datensatz fuer einen Zeitpunkt
// ============================================================
//
// WICHTIG:
// Dieser Datensatz bleibt in diesem Zwischenschritt noch float/m/s,
// weil Printer.cpp die Felder noch direkt mit Serial.print(x, 2)
// ausgibt.
//
// Wenn diese Felder sofort auf int16_t geaendert wuerden,
// wuerde Serial.print(int, 2) nicht "2 Nachkommastellen" bedeuten,
// sondern Ausgabe zur Basis 2.
// Der Printer wird im naechsten Block separat umgebaut.

struct RearPendingFrame
{
    uint16_t frameId;
    uint32_t t;

    float voLi_s;
    float voLi_i;
    int16_t voLi_pwm;
    int32_t voLiCnt;

    float voRe_s;
    float voRe_i;
    int16_t voRe_pwm;
    int32_t voReCnt;

    float hiLi_s;
    float hiRe_s;

    int32_t hiLiCnt;
    int32_t hiReCnt;

    bool hasFrontSnapshot;
};

// ============================================================
// Anfrage-Datensatz fuer einen neuen RearFrame
// ============================================================
//
// Diese Struktur fuehrt die Geschwindigkeiten bereits als Integer
// in cm/s. Die Regelung bleibt intern vorerst float in m/s.

struct RearFrameRequest
{
    uint32_t frameTimeMs;
    bool resetPi;

    int16_t voLi_s_cms;
    int16_t voLi_i_cms;
    int16_t voLi_pwm;
    int32_t voLiCnt;

    int16_t voRe_s_cms;
    int16_t voRe_i_cms;
    int16_t voRe_pwm;
    int32_t voReCnt;

    int16_t hiLi_s_cms;
    int16_t hiRe_s_cms;
};

// ============================================================
// RearFrameClient
// Verwaltet:
//  - aktuellen Hinterachs-Messframe
//  - Frame-ID
//  - Wartezustand auf VSOL_OK / VIST
//  - Stop-Sequenz fuer hinten
//  - letzte gueltige Rear-Istwerte
//  - hintere Encoder-Counts fuer Odometrie
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

    int16_t hiLiIstCms() const;
    int16_t hiReIstCms() const;

    int16_t hiLiPwm() const;
    int16_t hiRePwm() const;
    int32_t hiLiCnt() const;
    int32_t hiReCnt() const;

    bool requestFrame(
        Stream& out,
        uint32_t nowMs,
        const RearFrameRequest& request);

    void sendStop(Stream& out, uint32_t nowMs);

    bool handleVsolOkLine(const char* line, uint32_t nowMs);

    bool handleVistLine(const char* line);
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

    enum WaitState : uint8_t
    {
        WAIT_NONE = 0,
        WAIT_VSOL_OK = 1,
        WAIT_VIST = 2
    };

    void clearRearIst();

    uint16_t _nextFrameId;
    RearPendingFrame _frame;

    uint32_t _requestMs;
    uint32_t _lastSendMs;

    WaitState _waitState;

    uint8_t _stopSendCount;
    bool _stopSequenceArmed;

    int16_t _hiLiIstCms;
    int16_t _hiReIstCms;
    int16_t _hiLiPwm;
    int16_t _hiRePwm;
    int32_t _hiLiCnt;
    int32_t _hiReCnt;
};

#endif // REAR_FRAME_CLIENT_H