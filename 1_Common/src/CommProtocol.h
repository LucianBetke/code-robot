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
// Geschwindigkeiten werden als int16_t im Format:
//   Wert = m/s * 100
// uebertragen.
// Beispiel:
//   0.30 m/s  ->  30
//  -0.20 m/s  -> -20
//
// Encoder-Counts werden als int32_t Rohwerte uebertragen.
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

// ============================================================
// Parserfunktionen
// ============================================================

bool parseVsolLine(const char* line, VsolMessage& msg);
bool parseVsolOkLine(const char* line, VsolOkMessage& msg);
bool parseVistLine(const char* line, VistMessage& msg);

// ============================================================
// Ausgabe-Funktionen
// ============================================================

void printVsol(Stream& out, uint16_t frameId, bool resetPi, int16_t hiLiSoll, int16_t hiReSoll);
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