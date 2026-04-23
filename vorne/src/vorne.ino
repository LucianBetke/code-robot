/*
 Name:      vorne.ino
 Created:   11.03.2026 22:17:58
 Author:    Acer
*/
#include "CommandScript.h"
#include "Control.h"
#include "Hardware.h"
#include "../1_Common/src/globals.h"
#include "../2_Hardware/src/UartLink.h"
#include "../4_Vehicle/src/VehicleController.h"
#include "../5_System/src/Connection/ConnectionMonitor.h"
#include "../5_System/src/Parser/CommandParser.h"
VehicleController vehicle;

UartLink uart(Serial, true);   // Initiator
ConnectionMonitor conn(uart, 13);
CommandParser parser;

void setup()
{
    Serial.begin(115200);
    hardware_begin();
    hardware_enableMotors();
    control_begin();

/*    uart.begin();
    conn.begin(true); */  
}

void loop()
{
    Serial.println("Test läuft");

    motor[Li].vor(150);   // linkes Rad vorwärts
    motor[Re].vor(150);   // rechtes Rad vorwärts

    delay(2000);

    motor[Li].bremse(true);
    motor[Re].bremse(true);

    delay(2000);
}