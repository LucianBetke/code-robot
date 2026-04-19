/*
 Name:      vorne.ino
 Created:   11.03.2026 22:17:58
 Author:    Acer
*/

#include "../1_Common/src/globals.h"
#include "../4_Vehicle/src/VehicleController.h"
#include "../2_Hardware/src/UartLink.h"
#include "../5_System/src/ConnectionMonitor.h"

VehicleController vehicle;

UartLink uart(Serial, true);   // Initiator
ConnectionMonitor conn(uart, 13);

void setup()
{
    Serial.begin(115200);

    uart.begin();
    conn.begin(true);   // 🔥 fertig
}

void loop()
{
    uart.update();
    conn.update();

    // später:
    // vehicle.update();
    // control_update();
}