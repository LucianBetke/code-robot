// CommProtocol.cpp
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
//   VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>
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
    if (*p != '\0') return false;

    msg.frameId = frameIdTmp;
    msg.hiLiIst = hiLiIstTmp;
    msg.hiReIst = hiReIstTmp;
    msg.hiLiPwm = hiLiPwmTmp;
    msg.hiRePwm = hiRePwmTmp;

    return true;
}