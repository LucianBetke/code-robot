#include "CommandParser.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// ------------------------------------------------------------
// Token lesen
// ------------------------------------------------------------
static bool readToken(const char*& p, char delim, char* buf, uint8_t maxLen)
{
    uint8_t i = 0;

    while (*p != delim && *p != '\0' && i < maxLen - 1)
        buf[i++] = *p++;

    buf[i] = '\0';

    if (*p != delim)
        return false;

    p++;  // Trennzeichen überspringen
    return true;
}

// ------------------------------------------------------------
// Parser
// ------------------------------------------------------------
bool CommandParser::parseTimeCommand(const char* line, TimeCommand& cmd)
{
    if (!line || strncmp(line, "CMDT(", 5) != 0)
        return false;

    const char* p = line + 5;
    char buf[32];

    // vx
    if (!readToken(p, ',', buf, sizeof(buf))) return false;
    cmd.vx = atof(buf);

    // vy
    if (!readToken(p, ',', buf, sizeof(buf))) return false;
    cmd.vy = atof(buf);

    // wz
    if (!readToken(p, ')', buf, sizeof(buf))) return false;
    cmd.wz = atof(buf);

    // Leerzeichen überspringen
    if (*p == ' ') p++;

    // Zeit
    if (!readToken(p, ';', buf, sizeof(buf))) return false;
    cmd.t = atof(buf);

    return true;
}