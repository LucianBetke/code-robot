// ============================================================
/* File: Encoder.h
 * Zweck:
 *  - Quadratur-Encoder (Port C / PCINT1: A0..A5) zählen
 *  - Atomarer Zugriff auf Zählerstand
 *  - Dispatcher ruft handleIsr() mit PINC-Snapshot
 * Abhängigkeiten: Arduino.h, <avr/io.h>, <avr/interrupt.h>
 */
 // ============================================================

#pragma once
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/**
 * @class Enc
 * @brief Zählt einen Quadratur-Encoder über PCINT1 (Port C).
 *
 * Ablauf:
 *  - begin(): Pins konfigurieren, PCINT aktivieren, Startzustand erfassen
 *  - ISR (Vektor PCINT1_vect) liest PINC und übergibt Snapshot an Dispatcher
 *  - Dispatcher ruft handleIsr() → Zustandsübergang via Lookup-Tabelle
 *
 * Thread-Sicherheit:
 *  - getCounts()/setCounts() sind atomar (kritische Abschnitte).
 */
class Enc {
public:
    /**
     * @brief Konstruktor.
     * @param pinA Arduino-Pin für Kanal A (A0..A5)
     * @param pinB Arduino-Pin für Kanal B (A0..A5)
     */
    Enc(uint8_t pinA, uint8_t pinB);

    /**
     * @brief Pins einrichten, PCINT aktivieren, optional Counts auf 0.
     * @param resetCounts true → Zähler auf 0 setzen
     */
    void begin(bool resetCounts = true);

    /** @brief Zähler auf 0 setzen (alias zu setCounts(0)). */
    void resetCounts();

    /**
     * @brief Aktuellen Zählerstand atomar lesen.
     * @return Encoder-Ticks (vorzeichenbehaftet)
     */
    int32_t getCounts() const;

    /**
     * @brief Zählerstand atomar setzen.
     * @param v Neuer Ticks-Wert
     */
    void setCounts(int32_t v);

    /**
     * @brief Zustandsübergang verarbeiten (vom Dispatcher gerufen).
     * @param pinc_snapshot Momentanbild von PINC
     */
    void handleIsr(uint8_t pinc_snapshot);

private:
    // Quadratur-Lookup: prev(0..3) × curr(0..3) → -1/0/+1/Fehler
    static const int8_t s_qlut[4][4];

    uint8_t  _pinA;   ///< Arduino-Pin A
    uint8_t  _pinB;   ///< Arduino-Pin B
    uint8_t  _bitA;   ///< Bitindex 0..5 (PC0..PC5)
    uint8_t  _bitB;   ///< Bitindex 0..5 (PC0..PC5)
    uint8_t  _maskA;  ///< Bitmaske für A
    uint8_t  _maskB;  ///< Bitmaske für B

    volatile int32_t _counts;    ///< Tick-Zähler (atomar zugreifbar)
    volatile uint8_t _prevState; ///< letzter 2-Bit Zustand (B<<1|A)

    /** @brief PCINT1 nur für die gewählten Pins aktivieren. */
    void enablePcintGroup1_SelectedPins();
};

/**
 * @namespace PCINT1_Dispatcher
 * @brief Registriert bis zu 2 Enc-Instanzen und verteilt PINC-Snapshots.
 *
 * Verwendung:
 *  - add(&enc)
 *  - ISR ruft isr(PINC_snapshot)
 */
namespace PCINT1_Dispatcher {
    /** @brief Encoder beim Dispatcher registrieren (max. 2). */
    void add(Enc* enc);

    /** @brief An die registrierten Encoder weiterreichen. */
    void isr(uint8_t pinc_snapshot);
}
