/*
 Name:		vorne.ino
 Created:	11.03.2026 22:17:58
 Author:	Acer
*/

#include "../1_Common/src/globals.h"
#include "../4_Vehicle/src/VehicleController.h"


VehicleController vehicle;

// the setup function runs once when you press reset or power the board
void setup()
{
    Serial.begin(115200);
    Serial.println("Front Nano gestartet");

    vehicle.cmd(0.3f, 0.3f, 0.0f);

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        Serial.print(WHEEL_VEHICLE_NAME[i]);
        Serial.print(": ");
        Serial.print(vehicle.getWheelSoll((WheelVehicle)i));
        Serial.print("  ");
    }

    Serial.println();
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
