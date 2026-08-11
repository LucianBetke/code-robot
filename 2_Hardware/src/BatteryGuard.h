// ============================================================
// File: BatteryGuard.h
// Zweck:
//  - Akkuspannung ueber Spannungsteiler messen
//  - Startfreigabe bei zu leerem Akku verweigern
// ============================================================

#ifndef BATTERY_GUARD_H
#define BATTERY_GUARD_H

#include <Arduino.h>

// ============================================================
// Dimensionierung
// ============================================================
//
//   Akku (+) --[ R1 = 100,2k ]--+--[ R2 = 9,84k ]-- GND
//                               |
//                              A7  (zusaetzlich 100nF nach GND)
//
// Gemessen wird gegen die interne 1,1-V-Referenz, nicht gegen
// die 5-V-Schiene. Die 5 V kommen aus einem Step-Down und
// bleiben ueber den gesamten Akkubereich stabil - eine Messung
// gegen VCC haengt dafuer an der Ausgangsgenauigkeit des
// Wandlers. Die ueblichen Module liegen je nach Last irgendwo
// zwischen 4,9 und 5,2 V, und dieser Fehler ginge unbemerkt
// direkt in die Schwelle ein. Die Bandgap-Referenz haengt an
// nichts davon.
//
// Abgegriffen wird vor dem Step-Down, hinter dem Hauptschalter.
//
// A6 und A7 sind beim Nano reine ADC-Eingaenge. Sie brauchen
// kein pinMode() und kosten keinen digitalen Pin.

// Teilerverhaeltnis (R1 + R2) / R2, als Bruch mal 100.
//
// Am gebauten Aufbau ermittelt, auf zwei Wegen:
//   Widerstaende: (100,2k + 9,84k) / 9,84k = 11,183
//   Spannungen:          7,56 V / 0,675 V  = 11,200
// Die 0,15 Prozent Unterschied liegen in der Anzeigeaufloesung
// und in der Belastung des Abgriffs durch das Multimeter.
// Eingetragen ist der Widerstandswert.
constexpr uint16_t BATTERY_DIVIDER_NUM = 1118;
constexpr uint16_t BATTERY_DIVIDER_DEN = 100;

// Interne Bandgap-Referenz in mV, kalibriert.
//
// Der Nennwert 1100 streut fertigungsbedingt zwischen etwa
// 1000 und 1200 mV. Gilt nur fuer diesen Chip: wird der
// hintere Nano getauscht, ist neu zu kalibrieren.
//
// Kalibriert am 11.08.2026 auf dem hinteren Nano:
//   gemeldet mit 1100:  7465 mV  (621 ADC-Digits)
//   Multimeter Knoten A: 7550 mV
//   1100 * 7550 / 7465 = 1112,5 -> 1113
// Gegenprobe: dieselben 621 Digits ergeben damit 7553 mV.
// Ein ADC-Digit entspricht rund 12 mV, naeher geht nicht.
//
// Vorgehen fuer eine Neukalibrierung: siehe EINBAU.md.
constexpr uint16_t BATTERY_REF_MV = 1113;

// 2S LiPo: 8,4 V voll, 7,4 V Nennspannung.
// Es gibt bewusst keine Ueberwachung waehrend der Fahrt.
// Die Startschwelle ist deshalb konservativ gewaehlt: was
// hier durchgeht, muss eine ganze Fahrt lang reichen.
// 7200 mV entspricht 3,6 V je Zelle im Leerlauf.
constexpr uint16_t BATTERY_MIN_START_MV = 7200;

// Die ersten Wandlungen nach dem Umschalten der Referenz
// sind unbrauchbar und werden verworfen.
constexpr uint8_t BATTERY_DISCARD_SAMPLES = 4;

// Gemessen wird im Leerlauf, ein einfacher Mittelwert
// reicht deshalb aus.
constexpr uint8_t BATTERY_AVERAGE_SAMPLES = 16;

// ============================================================
// Schnittstelle
// ============================================================

// Referenz umschalten und Messpin merken.
// Muss vor der ersten Messung laufen.
void batteryGuard_begin(uint8_t sensePin);

// Gemittelte Akkuspannung in mV.
uint16_t batteryGuard_readMillivolts();

// true, wenn der Akku fuer eine Fahrt reicht.
// measuredMv erhaelt in jedem Fall den gemessenen Wert.
bool batteryGuard_isStartAllowed(uint16_t& measuredMv);

#endif // BATTERY_GUARD_H
