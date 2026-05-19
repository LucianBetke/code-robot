// CommProtocol.cpp
#include <Arduino.h>
#include "CommProtocol.h"

// ============================================================
// Interne Parser-Hilfsfunktionen
// ============================================================

static void skipSpaces(const char*& p)
{
    while (*p == ' ' || *p == '\t') p++;
}

static bool expectChar(const char*& p, char expected)
{
    if (*p != expected) return false;

    p++;
    return true;
}

static bool expectText(const char*& p, const char* text)
{
    while (*text != '\0')
    {
        if (*p != *text) return false;

        p++;
        text++;
    }

    return true;
}

static bool parseUInt16(const char*& p, uint16_t& out)
{
    skipSpaces(p);

    if (*p < '0' || *p > '9') return false;

    uint32_t value = 0;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10UL + (uint32_t)(*p - '0');

        if (value > 65535UL) return false;

        p++;
    }

    out = (uint16_t)value;
    return true;
}

static bool parseInt16(const char*& p, int16_t& out)
{
    skipSpaces(p);

    bool negative = false;

    if (*p == '-')
    {
        negative = true;
        p++;
    }

    if (*p < '0' || *p > '9') return false;

    int32_t value = 0;
    const int32_t limit = negative ? 32768L : 32767L;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10L + (int32_t)(*p - '0');

        if (value > limit) return false;

        p++;
    }

    if (negative) value = -value;

    out = (int16_t)value;
    return true;
}

static bool parseInt32(const char*& p, int32_t& out)
{
    skipSpaces(p);

    bool negative = false;

    if (*p == '-')
    {
        negative = true;
        p++;
    }

    if (*p < '0' || *p > '9') return false;

    uint32_t value = 0;
    const uint32_t limit = negative ? 2147483648UL : 2147483647UL;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10UL + (uint32_t)(*p - '0');

        if (value > limit) return false;

        p++;
    }

    if (negative)
    {
        if (value == 2147483648UL)
        {
            out = (int32_t)(-2147483647L - 1L);
        }
        else
        {
            out = -(int32_t)value;
        }
    }
    else
    {
        out = (int32_t)value;
    }

    return true;
}

// ============================================================
// VSOL Parser
// Format:
//   VSOL,<frameId>,<resetPi>,<hiLiSoll>,<hiReSoll>
// ============================================================

bool parseVsolLine(const char* line, VsolMessage& msg)
{
    if (!line) return false;

    const char* p = line;

    uint16_t frameIdTmp = 0;
    uint16_t resetTmp = 0;
    int16_t hiLiSollTmp = 0;
    int16_t hiReSollTmp = 0;

    if (!expectText(p, "VSOL,")) return false;
    if (!parseUInt16(p, frameIdTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseUInt16(p, resetTmp)) return false;
    if (resetTmp > 1) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, hiLiSollTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, hiReSollTmp)) return false;

    skipSpaces(p);
    if (*p != '\0') return false;

    msg.frameId = frameIdTmp;
    msg.resetPi = (resetTmp != 0);
    msg.hiLiSoll = hiLiSollTmp;
    msg.hiReSoll = hiReSollTmp;

    return true;
}

// ============================================================
// VSOL_OK Parser
// Format:
//   VSOL_OK,<frameId>
// ============================================================

bool parseVsolOkLine(const char* line, VsolOkMessage& msg)
{
    if (!line) return false;

    const char* p = line;
    uint16_t frameIdTmp = 0;

    if (!expectText(p, "VSOL_OK,")) return false;
    if (!parseUInt16(p, frameIdTmp)) return false;

    skipSpaces(p);
    if (*p != '\0') return false;

    msg.frameId = frameIdTmp;

    return true;
}

// ============================================================
// VIST Parser
// Format:
//   VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>,<hiLiCnt>,<hiReCnt>
// ============================================================

bool parseVistLine(const char* line, VistMessage& msg)
{
    if (!line) return false;

    const char* p = line;

    uint16_t frameIdTmp = 0;
    int16_t hiLiIstTmp = 0;
    int16_t hiReIstTmp = 0;
    int16_t hiLiPwmTmp = 0;
    int16_t hiRePwmTmp = 0;
    int32_t hiLiCntTmp = 0;
    int32_t hiReCntTmp = 0;

    if (!expectText(p, "VIST,")) return false;
    if (!parseUInt16(p, frameIdTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, hiLiIstTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, hiReIstTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, hiLiPwmTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, hiRePwmTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt32(p, hiLiCntTmp)) return false;

    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt32(p, hiReCntTmp)) return false;

    skipSpaces(p);
    if (*p != '\0') return false;

    msg.frameId = frameIdTmp;
    msg.hiLiIst = hiLiIstTmp;
    msg.hiReIst = hiReIstTmp;
    msg.hiLiPwm = hiLiPwmTmp;
    msg.hiRePwm = hiRePwmTmp;
    msg.hiLiCnt = hiLiCntTmp;
    msg.hiReCnt = hiReCntTmp;

    return true;
}

// ============================================================
// Ausgabe: VSOL
// Format:
//   VSOL,<frameId>,<resetPi>,<hiLiSoll>,<hiReSoll>
// ============================================================

void printVsol(Stream& out, uint16_t frameId, bool resetPi, int16_t hiLiSoll, int16_t hiReSoll)
{
    out.print(F("VSOL,"));
    out.print((unsigned int)frameId);
    out.print(',');
    out.print(resetPi ? 1 : 0);
    out.print(',');
    out.print((int)hiLiSoll);
    out.print(',');
    out.println((int)hiReSoll);
}

// ============================================================
// Ausgabe: VSOL_OK
// Format:
//   VSOL_OK,<frameId>
// ============================================================

void printVsolOk(Stream& out, uint16_t frameId)
{
    out.print(F("VSOL_OK,"));
    out.println((unsigned int)frameId);
}

// ============================================================
// Ausgabe: VIST
// Format:
//   VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>,<hiLiCnt>,<hiReCnt>
// ============================================================

void printVist(
    Stream& out,
    uint16_t frameId,
    int16_t hiLiIst,
    int16_t hiReIst,
    int16_t hiLiPwm,
    int16_t hiRePwm,
    int32_t hiLiCnt,
    int32_t hiReCnt)
{
    out.print(F("VIST,"));
    out.print((unsigned int)frameId);
    out.print(',');
    out.print((int)hiLiIst);
    out.print(',');
    out.print((int)hiReIst);
    out.print(',');
    out.print((int)hiLiPwm);
    out.print(',');
    out.print((int)hiRePwm);
    out.print(',');
    out.print((long)hiLiCnt);
    out.print(',');
    out.println((long)hiReCnt);
}