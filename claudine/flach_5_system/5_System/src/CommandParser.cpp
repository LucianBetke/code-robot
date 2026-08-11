// ============================================================
// CommandParser.cpp
// ============================================================

#include "CommandParser.h"
#include <string.h>

// ============================================================
// Hilfsfunktionen
// ============================================================

static void skipSpaces(const char*& p)
{
    while (*p == ' ' || *p == '\t')
    {
        p++;
    }
}

static bool expectChar(const char*& p, char expected)
{
    if (*p != expected)
    {
        return false;
    }

    p++;
    return true;
}

static bool parseInt16(const char*& p, int16_t& out)
{
    bool negative = false;

    if (*p == '-')
    {
        negative = true;
        p++;
    }

    if (*p < '0' || *p > '9')
    {
        return false;
    }

    int32_t value = 0;
    const int32_t limit = negative ? 32768L : 32767L;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10L + (int32_t)(*p - '0');

        if (value > limit)
        {
            return false;
        }

        p++;
    }

    if (negative)
    {
        value = -value;
    }

    out = (int16_t)value;
    return true;
}

static bool parseUInt16(const char*& p, uint16_t& out)
{
    if (*p < '0' || *p > '9')
    {
        return false;
    }

    uint32_t value = 0;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10UL + (uint32_t)(*p - '0');

        if (value > 65535UL)
        {
            return false;
        }

        p++;
    }

    out = (uint16_t)value;
    return true;
}

static bool parseVxVyWzParam(const char* p, ParsedCommand& cmd)
{
    skipSpaces(p);

    if (!parseInt16(p, cmd.vx)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;
    skipSpaces(p);

    if (!parseInt16(p, cmd.vy)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;
    skipSpaces(p);

    if (!parseInt16(p, cmd.wz)) return false;
    skipSpaces(p);
    if (!expectChar(p, ')')) return false;
    skipSpaces(p);

    if (!parseUInt16(p, cmd.param)) return false;
    skipSpaces(p);
    if (!expectChar(p, ';')) return false;
    skipSpaces(p);

    return *p == '\0';
}

// ============================================================
// CommandParser
// ============================================================

bool CommandParser::parse(const char* line, ParsedCommand& cmd)
{
    cmd.type = CMD_NONE;
    cmd.vx = 0;
    cmd.vy = 0;
    cmd.wz = 0;
    cmd.param = 0;

    if (!line)
    {
        return false;
    }

    if (strncmp(line, "CMDP(", 5) == 0)
    {
        if (!parseVxVyWzParam(line + 5, cmd))
        {
            return false;
        }

        cmd.type = CMD_PATH;
        return true;
    }

    return false;
}