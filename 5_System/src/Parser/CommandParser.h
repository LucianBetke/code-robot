// CommandParser.h
#pragma once

struct TimeCommand
{
    float vx;
    float vy;
    float wz;
    float t;
};

class CommandParser
{
public:
    static bool parseTimeCommand(const char* line, TimeCommand& cmd);
};
