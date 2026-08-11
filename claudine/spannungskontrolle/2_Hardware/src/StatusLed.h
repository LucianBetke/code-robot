// ============================================================
// File: StatusLed.h
// Zweck:
//  - Status-LED zwischen zwei GPIOs ansteuern
//  - Dauerlicht und blockierendes Blinken bereitstellen
// ============================================================

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

// ============================================================
// Aufbau
// ============================================================
//
//   Anode --[ LED ]--[ 330 Ohm ]-- Kathode
//
// Die LED haengt zwischen zwei Ausgaengen statt zwischen Pin
// und Masse: die Anodenseite fuehrt HIGH, die Kathodenseite
// liegt LOW und uebernimmt die Rolle der Masse. Welcher Pin
// welche Rolle hat, steht in HardwarePins.h.
//
// Strom: bei etwa 4,0 V wirksamer Spannung ueber LED und
// Widerstand fliessen mit 330 Ohm rund 6 mA (rote LED) bis
// 3 mA (blau/weiss). Der ATmega darf 20 mA je Pin dauerhaft,
// es ist also reichlich Luft.
//
// Vorteil dieser Verdrahtung: die Polaritaet laesst sich in
// Software korrigieren. Leuchtet die LED nicht, genuegt es,
// die beiden Pinnummern in PinsRear zu tauschen.
//
// Im Ruhezustand liegen beide Pins LOW. Damit ist die LED
// stromlos, und waehrend Reset und Bootloader sind beide
// Pins ohnehin hochohmig.

void statusLed_begin(
    uint8_t anodePin,
    uint8_t cathodePin);

void statusLed_on();
void statusLed_off();

// Blockierend - nur fuer den Sperrzustand gedacht, in dem
// ohnehin nichts anderes mehr laeuft.
void statusLed_blinkBlocking(
    uint8_t count,
    uint16_t onMs,
    uint16_t offMs);

#endif // STATUS_LED_H
