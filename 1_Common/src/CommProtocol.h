// CommProtocol.h
#pragma once

#include <stdint.h>

class Stream;

// ============================================================
// UART-Protokolltypen
// ============================================================
//
// VSOL:
//   Front -> Rear
//   VSOL,<frameId>,<resetPi>,<hiLiSoll>,<hiReSoll>
//
// VSOL_OK:
//   Rear -> Front
//   VSOL_OK,<frameId>
//
// VIST:
//   Rear -> Front
//   VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>,<hiLiCnt>,<hiReCnt>
//
// US:
//   Rear -> Front
//   US,<sequence>,<frontMm>,<leftMm>,<rightMm>,<validMask>,
//      <frontAgeMs>,<leftAgeMs>,<rightAgeMs>
//
// US-Gueltigkeitsmaske:
//   Bit 0 = Front
//   Bit 1 = Links
//   Bit 2 = Rechts
//
// Geschwindigkeiten werden als int16_t in cm/s uebertragen.
// Encoder-Counts werden als int32_t Rohwerte uebertragen.
// Ultraschallentfernungen und Alter werden als uint16_t uebertragen.
// ============================================================

struct VsolMessage
{
    uint16_t frameId;
    bool resetPi;
    int16_t hiLiSoll;
    int16_t hiReSoll;
};

struct VsolOkMessage
{
    uint16_t frameId;
};

struct VistMessage
{
    uint16_t frameId;
    int16_t hiLiIst;
    int16_t hiReIst;
    int16_t hiLiPwm;
    int16_t hiRePwm;
    int32_t hiLiCnt;
    int32_t hiReCnt;
};

struct UsMessage
{
    uint16_t sequence;

    uint16_t frontMm;
    uint16_t leftMm;
    uint16_t rightMm;

    uint8_t validMask;

    uint16_t frontAgeMs;
    uint16_t leftAgeMs;
    uint16_t rightAgeMs;
};

// ============================================================
// Parserfunktionen
// ============================================================

bool parseVsolLine(const char* line, VsolMessage& msg);
bool parseVsolOkLine(const char* line, VsolOkMessage& msg);
bool parseVistLine(const char* line, VistMessage& msg);
bool parseUsLine(const char* line, UsMessage& msg);

// ============================================================
// Ausgabe-Funktionen
// ============================================================

void printVsol(
    Stream& out,
    uint16_t frameId,
    bool resetPi,
    int16_t hiLiSoll,
    int16_t hiReSoll);

void printVsolOk(Stream& out, uint16_t frameId);

void printVist(
    Stream& out,
    uint16_t frameId,
    int16_t hiLiIst,
    int16_t hiReIst,
    int16_t hiLiPwm,
    int16_t hiRePwm,
    int32_t hiLiCnt,
    int32_t hiReCnt);

void printUs(
    Stream& out,
    uint16_t sequence,
    uint16_t frontMm,
    uint16_t leftMm,
    uint16_t rightMm,
    uint8_t validMask,
    uint16_t frontAgeMs,
    uint16_t leftAgeMs,
    uint16_t rightAgeMs);