// CommandParser.cpp    
#include "CommandParser.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

static bool readToken(const char*& p, char delim, char* buf, uint8_t maxLen)
{
    uint8_t i = 0;
    while (*p != delim && *p != '\0' && i < maxLen - 1)
        buf[i++] = *p++;
    buf[i] = '\0';
    if (*p != delim) return false;
    p++;
    return true;
}

static bool parseVxVyWzParam(const char* p, ParsedCommand& cmd)
{
    char buf[32];
    if (!readToken(p, ',', buf, sizeof(buf))) return false;
    cmd.vx = atof(buf);
    if (!readToken(p, ',', buf, sizeof(buf))) return false;
    cmd.vy = atof(buf);
    if (!readToken(p, ')', buf, sizeof(buf))) return false;
    cmd.wz = atof(buf);
    if (*p == ' ') p++;
    if (!readToken(p, ';', buf, sizeof(buf))) return false;
    cmd.param = atof(buf);
    return true;
}

bool CommandParser::parse(const char* line, ParsedCommand& cmd)
{
    cmd.type = CMD_NONE;
    if (!line) return false;

    if (strncmp(line, "CMDT(", 5) == 0)
    {
        if (!parseVxVyWzParam(line + 5, cmd)) return false;
        cmd.type = CMD_TIME;
        return true;
    }
    if (strncmp(line, "CMDP(", 5) == 0)
    {
        if (!parseVxVyWzParam(line + 5, cmd)) return false;
        cmd.type = CMD_PATH;
        return true;
    }
    return false;
}