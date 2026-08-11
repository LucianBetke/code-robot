// ============================================================
// CommandParser.h
// ============================================================
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

enum CmdType : uint8_t
{
    CMD_NONE = 0,
    CMD_PATH = 1
};

struct ParsedCommand
{
    CmdType type;

    // Integer-Protokoll:
    // vx    = Geschwindigkeit in cm/s
    // vy    = Geschwindigkeit in cm/s
    // wz    = Winkelgeschwindigkeit in Grad/s
    // param = Zielwert bei CMDP:
    //         Translation: Strecke in cm
    //         Rotation: Winkel in Grad
    int16_t  vx;
    int16_t  vy;
    int16_t  wz;
    uint16_t param;
};

class CommandParser
{
public:
    static bool parse(const char* line, ParsedCommand& cmd);
};

#endif