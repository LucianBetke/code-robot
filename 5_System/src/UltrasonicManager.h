// ============================================================
// File: UltrasonicManager.h
// Zweck:
//  - HC-SR04 nicht blockierend ansteuern
//  - Timeout, Guard-Time, Plausibilitaet und Medianfilter
//  - Aktuellen Datensatz fuer den UART-Transfer bereitstellen
//  - Messbetrieb gezielt ein- und ausschalten
//
// Die Sensoren sind auf beide Nanos verteilt. Nicht vorhandene
// Sensoren werden an begin() mit ULTRASONIC_NO_PIN uebergeben;
// der Messzyklus entsteht daraus zur Laufzeit:
//  - alle drei:      Front -> Links -> Front -> Rechts
//  - nur Front:      Front
//  - nur die Seiten: Links -> Rechts
// ============================================================

#ifndef ULTRASONIC_MANAGER_H
#define ULTRASONIC_MANAGER_H

#include <Arduino.h>

#include "src/UltrasonicEchoCapture.h"

enum class UltrasonicSensor : uint8_t
{
    Front = 0,
    Left = 1,
    Right = 2,
    Count = 3
};

struct UltrasonicSnapshot
{
    uint16_t sequence;

    uint16_t frontMm;
    uint16_t leftMm;
    uint16_t rightMm;

    uint8_t validMask;

    uint16_t frontAgeMs;
    uint16_t leftAgeMs;
    uint16_t rightAgeMs;
};

class UltrasonicManager
{
public:
    UltrasonicManager();

    void begin(
        uint8_t frontTriggerPin,
        uint8_t frontEchoPin,
        uint8_t leftTriggerPin,
        uint8_t leftEchoPin,
        uint8_t rightTriggerPin,
        uint8_t rightEchoPin);

    void setEnabled(bool enabled);
    bool isEnabled() const;

    void update(uint32_t nowMs);
    void makeSnapshot(
        uint32_t nowMs,
        UltrasonicSnapshot& snapshot) const;

private:
    enum class State : uint8_t
    {
        Idle = 0,
        WaitEcho = 1,
        Guard = 2
    };

    struct SensorData
    {
        uint16_t filterValues[3];
        uint8_t filterCount;
        uint8_t filterNext;

        uint16_t filteredMm;
        uint32_t lastValidMs;

        bool hasValidValue;
        bool latestAttemptValid;
    };

    static const uint8_t SENSOR_COUNT = 3;
    static const uint8_t CYCLE_LENGTH = 4;

    static const uint16_t MIN_DISTANCE_MM = 20;
    static const uint16_t MAX_DISTANCE_MM = 4000;

    static const uint32_t ECHO_TIMEOUT_US = 25000UL;

    // Mindest-Ruhezeit nach Abschluss einer Messung.
    static const uint32_t GUARD_TIME_US = 15000UL;

    // Fester Messplatz ab Triggerbeginn.
    static const uint32_t MEASUREMENT_SLOT_US = 40000UL;

    UltrasonicEchoCapture _capture;

    uint8_t _triggerPins[SENSOR_COUNT];
    SensorData _sensorData[SENSOR_COUNT];

    // Messfolge, zur Laufzeit aus den vorhandenen Sensoren
    // gebildet. _cycleLength == 0 bedeutet: kein Sensor an
    // diesem Nano, der Manager bleibt dann untaetig.
    UltrasonicSensor _cycle[CYCLE_LENGTH];
    uint8_t _cycleLength;

    bool _enabled;

    State _state;
    UltrasonicSensor _activeSensor;

    uint8_t _cycleIndex;

    uint32_t _stateStartedUs;
    uint32_t _slotStartedUs;

    uint16_t _sequence;

    static uint8_t indexOf(UltrasonicSensor sensor);

    static UltrasonicEchoChannel channelOf(
        UltrasonicSensor sensor);

    static uint16_t clampAgeMs(uint32_t ageMs);

    bool hasSensor(UltrasonicSensor sensor) const;
    void buildMeasurementCycle();

    void clearMeasurements();
    void forceIdle();

    void startMeasurement(UltrasonicSensor sensor);
    void finishTimeout();
    void enterGuard();
    void startNextMeasurement();

    void acceptDistance(
        UltrasonicSensor sensor,
        uint16_t distanceMm,
        uint32_t nowMs);

    void rejectMeasurement(UltrasonicSensor sensor);

    static uint16_t filteredValue(
        const SensorData& data);
};

#endif // ULTRASONIC_MANAGER_H