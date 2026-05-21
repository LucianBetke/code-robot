// CommandScript.cpp
#include "CommandScript.h"

static const char* _script[] = {
    "CMDP(0,0,65) 90;",
    "CMDT(0,0,0) 2;"
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