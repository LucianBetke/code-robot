// CommandScript.cpp
#include "CommandScript.h"

static const char* _script[] = {
    // reine Vorwärtsfahrt
    "CMDT(15,0,0) 2;",
    "CMDT(0,0,0) 2;",

    // reine Rückwärtsfahrt
    "CMDT(-15,0,0) 2;",
    "CMDT(0,0,0) 2;",

    // vorwärts + links
    "CMDT(15,15,0) 2;",
    "CMDT(0,0,0) 2;",

    // vorwärts + rechts
    "CMDT(15,-15,0) 2;",
    "CMDT(0,0,0) 2;",

    // rückwärts + links
    "CMDT(-15,15,0) 2;",
    "CMDT(0,0,0) 2;",

    // rückwärts + rechts
    "CMDT(-15,-15,0) 2;",
    "CMDT(0,0,0) 2;"
};

const char* CommandScript::get(uint8_t index)
{
    return _script[index];
}

uint8_t CommandScript::size()
{
    return sizeof(_script) / sizeof(_script[0]);
}