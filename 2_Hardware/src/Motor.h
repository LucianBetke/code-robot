// ============================================================
// File: Motor.h
// Zweck:
//  - Steuerung eines einzelnen Motors (DRV8833/TB6612, 2-Pin-Modus)
//  - Richtung, PWM-Leistung, Bremse
//  - Optionale Encoder-Anbindung
// Abhängigkeiten: Arduino.h, Enc forward-deklariert
// ============================================================

#ifndef MOTOR_H
#define MOTOR_H

#pragma once
#include <Arduino.h>

class Enc;

/**
 * @class Motor
 * @brief Steuert einen Motor mit 2-Pin-Ansteuerung (z. B. DRV8833).
 *
 * Unterstützt:
 *  - Vorwärts- und Rückwärtsfahrt über PWM
 *  - Bremse (Brake oder Coast)
 *  - Optionale Verbindung zu einem Encoder (Enc)
 *
 * Beispiel:
 * @code
 * Motor motor[Re](5, 6);
 * motor[Re].vor(180);
 * delay(1000);
 * motor[Re].bremse(true);
 * @endcode
 */
class Motor {
public:
    Motor();                                            // NEU: leerer Konstruktor
    Motor(uint8_t pin1, uint8_t pin2);                  // bleibt bis _alt-Migration
    Motor(uint8_t pin1, uint8_t pin2, Enc& enc);        // bleibt bis _alt-Migration

    void begin(uint8_t pin1, uint8_t pin2, Enc& enc);   // NEU

    void init();
    void vor(uint8_t pwm);
    void rueck(uint8_t pwm);
    void bremse(bool art);
    void attachEnc(Enc& enc);
    Enc* enc() const;

private:
    uint8_t _pin1;
    uint8_t _pin2;
    Enc* _enc = nullptr;
};

#endif // MOTOR_H