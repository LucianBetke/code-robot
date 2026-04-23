// CommandScript.h
#pragma once
#include <Arduino.h>

class CommandScript
{
public:
    static const char* get(uint8_t index);
    static uint8_t size();
};
