// CommandScript.cpp
#include "CommandScript.h"

static const char* _script[] = {
    "CMDP(30,0,0) 100;",
    /*"CMDP(0,30,0) 100;",
    "CMDP(-30,0,0) 100;",
    "CMDP(0,-30,0) 100;",
    "CMDP(0,0,90) 360;",*/
};

const char* CommandScript::get(uint8_t index)
{
    if (index >= size()) return nullptr;
    return _script[index];
}

uint8_t CommandScript::size()
{
    return sizeof(_script) / sizeof(_script[0]);
}