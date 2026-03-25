// ============================================================
// File: VehicleController.h
// Zweck:
//  - Zentrale Frontdoor für Fahrzeugansteuerung
//  - Aktuell: SPEED (open loop PWM) 
//						+ 2-Rad-Sollwertbildung aus (vx,vy,omega)
// ============================================================

#pragma once
#include <Arduino.h>
#include "TestTypes.h"   // enum class Side
#include "Motor.h"


struct Twist {
	float vx;     // m/s
	float vy;     // m/s (bei 2 Rädern aktuell ignoriert)
	float omega;  // rad/s (Drehgeschwindigkeit um z)
};

class VehicleController {
public:
	// ctor bekommt die beiden Motoren
	VehicleController(Motor& motorRe, Motor& motorLi);

	// Setzt den aktuellen Fahrbefehl des Fahrzeugs.
	// Eingabegrößen:
	//   vx     – Vorwärts-/Rückwärtsgeschwindigkeit in m/s
	//   vy     – Seitwärtsgeschwindigkeit in m/s
	//            (bei 2-Rad-Konfiguration derzeit ignoriert)
	//   omega  – Rotationsgeschwindigkeit um die Hochachse in rad/s
	// Hinweis:
	//   Der Befehl wird intern gespeichert und sofort über den
	//   aktuellen Kinematik-Mixer in radbezogene Sollwerte übersetzt.
	void setCmd(float vx, float vy, float omega);

	// Liefert den Sollwert (Geschwindigkeit) für ein einzelnes Rad.
	// Rückgabewert:
	//   Sollgeschwindigkeit des angegebenen Rads in m/s.
	// Hinweis:
	//   Der Wert stammt aus dem zuletzt gesetzten Fahrbefehl (vx,vy,omega)
	//   und der aktuell aktiven Kinematik (z. B. 2-Rad, später 4-Rad).
	float getWheelSollMps(Side s) const;

	// Setzt normierten Fahrbefehl (dimensionslos).
	// ux, uy, uOmega typ. im Bereich [-1..+1].
	// Translationsanteil (ux,uy) wird bei Bedarf auf Länge 1 normiert,
	// damit diagonale Eingaben nicht schneller als "max" sind.
	void setCmdNorm(float ux, float uy, float uOmega);

	// Konfiguration (einmal beim Teststart)
	void selectSide(Side s);

	// Open-Loop PWM (SPEED)
	void setPWM(int16_t pwm);

	// zyklischer Aufruf
	void update();

private:
	void mix2W();

	Motor& _motorRe;
	Motor& _motorLi;

	Side   _side = Side::RE;
	int16_t _pwm = 0;

	Twist _cmd{ 0,0,0 };
	float _vSollRe = 0.0f;
	float _vSollLi = 0.0f;
};

