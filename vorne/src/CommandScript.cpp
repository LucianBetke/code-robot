// CommandScript.cpp
#include "CommandScript.h"

static const char* _script[] = {
    "CMDT(30,0,0) 2;",
    "CMDT(20,0,0) 3;",
    "CMDT(0,0,0) 2;",
    "CMDT(-20,0,0) 3;",
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