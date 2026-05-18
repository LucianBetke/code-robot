// CommandScript.cpp
#include "CommandScript.h"

static const char* _script[] = {
    // ========================================================
    // Reine Drehung positiv
    // wz im Script = Grad/s
    //
    // Erwartung bei wz > 0:
    // VoLi negativ, VoRe positiv, HiLi negativ, HiRe positiv
    // ========================================================

    // 30 Grad/s -> Radgeschwindigkeit ca. 0.079 m/s
    "CMDT(0,0,30) 2;",
    "CMDT(0,0,0) 2;",

    // 60 Grad/s -> Radgeschwindigkeit ca. 0.157 m/s
    "CMDT(0,0,60) 2;",
    "CMDT(0,0,0) 2;",

    // 90 Grad/s -> Radgeschwindigkeit ca. 0.236 m/s
    "CMDT(0,0,90) 2;",
    "CMDT(0,0,0) 2;",

    // 120 Grad/s -> Radgeschwindigkeit ca. 0.314 m/s
    "CMDT(0,0,120) 2;",
    "CMDT(0,0,0) 2;",

    // ========================================================
    // Reine Drehung negativ
    // wz im Script = Grad/s
    //
    // Erwartung bei wz < 0:
    // VoLi positiv, VoRe negativ, HiLi positiv, HiRe negativ
    // ========================================================

    // -30 Grad/s -> Radgeschwindigkeit ca. 0.079 m/s
    "CMDT(0,0,-30) 2;",
    "CMDT(0,0,0) 2;",

    // -60 Grad/s -> Radgeschwindigkeit ca. 0.157 m/s
    "CMDT(0,0,-60) 2;",
    "CMDT(0,0,0) 2;",

    // -90 Grad/s -> Radgeschwindigkeit ca. 0.236 m/s
    "CMDT(0,0,-90) 2;",
    "CMDT(0,0,0) 2;",

    // -120 Grad/s -> Radgeschwindigkeit ca. 0.314 m/s
    "CMDT(0,0,-120) 2;",
    "CMDT(0,0,0) 2;",

    // ========================================================
    // Mischbefehle absichtlich:
    // Wenn die Drehtrennung aktiv ist, müssen vx und vy
    // bei wz != 0 ignoriert werden.
    //
    // Diese Befehle müssen aussehen wie reine Drehung.
    // ========================================================

    // Muss aussehen wie CMDT(0,0,60)
    "CMDT(30,20,60) 2;",
    "CMDT(0,0,0) 2;",

    // Muss ebenfalls aussehen wie CMDT(0,0,60)
    "CMDT(-30,-20,60) 2;",
    "CMDT(0,0,0) 2;",

    // Muss aussehen wie CMDT(0,0,-60)
    "CMDT(30,20,-60) 2;",
    "CMDT(0,0,0) 2;",

    // Muss ebenfalls aussehen wie CMDT(0,0,-60)
    "CMDT(-30,-20,-60) 2;",
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