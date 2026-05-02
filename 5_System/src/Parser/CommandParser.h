// ============================================================
// CommandParser.h
// ============================================================
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

enum CmdType : uint8_t
{
    CMD_NONE = 0,
    CMD_TIME = 1,
    CMD_PATH = 2,   // später
};

struct ParsedCommand
{
    CmdType type;
    float vx;
    float vy;
    float wz;
    float param;     // CMD_TIME: Sekunden, CMD_PATH: cm
};

class CommandParser
{
public:
    static bool parse(const char* line, ParsedCommand& cmd);
};

#endif