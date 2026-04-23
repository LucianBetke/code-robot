// CommandParser.cpp
#include "CommandParser.h"
#include <string.h>
#include <stdio.h>

bool CommandParser::parseTimeCommand(const char* line, TimeCommand& cmd)
{
    if (!line) return false;

    if (strncmp(line, "CMDT(", 5) != 0)
        return false;

    float vx, vy, wz, t;

    int parsed = sscanf(line, "CMDT(%f,%f,%f) %f;", &vx, &vy, &wz, &t);

    if (parsed != 4)
        return false;

    cmd.vx = vx;
    cmd.vy = vy;
    cmd.wz = wz;
    cmd.t = t;

    return true;
}
