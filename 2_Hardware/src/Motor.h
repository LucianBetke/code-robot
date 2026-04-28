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
    /**
     * @brief Konstruktor ohne Encoder.
     * @param pin1 PWM-Pin für Richtung A
     * @param pin2 PWM-Pin für Richtung B
     */
    Motor(uint8_t pin1, uint8_t pin2);

    /**
     * @brief Konstruktor mit Encoder-Referenz.
     * @param pin1 PWM-Pin für Richtung A
     * @param pin2 PWM-Pin für Richtung B
     * @param enc  Referenz auf zugehörigen Encoder
     */
    Motor(uint8_t pin1, uint8_t pin2, Enc& enc);

    /** @brief Initialisiert die Pins (pinMode-Aufrufe). */
    void init();

    /** @brief Motor vorwärts mit PWM 0–255. */
    void vor(uint8_t pwm);

    /** @brief Motor rückwärts mit PWM 0–255. */
    void rueck(uint8_t pwm);

    /**
     * @brief Bremst den Motor.
     * @param art true = Brake (HIGH/HIGH), false = Coast (LOW/LOW)
     */
    void bremse(bool art);

    /**
     * @brief Encoder anhängen (nachträglich).
     * @param enc Referenz auf Encoder
     */
    void attachEnc(Enc& enc);

    /**
     * @brief Gibt Zeiger auf Encoder zurück (oder nullptr, falls keiner).
     * @return Encoder-Zeiger oder nullptr
     */
    Enc* enc() const;

private:
    uint8_t _pin1;  ///< PWM-Pin 1 (Richtung A)
    uint8_t _pin2;  ///< PWM-Pin 2 (Richtung B)
    Enc* _enc = nullptr; ///< Optionaler Encoder-Zeiger
};

#endif // MOTOR_H