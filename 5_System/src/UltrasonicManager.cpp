// ============================================================
// File: UltrasonicManager.cpp
// ============================================================

#include "UltrasonicManager.h"

UltrasonicManager::UltrasonicManager()
    : _capture(),
    _triggerPins{
        ULTRASONIC_NO_PIN,
        ULTRASONIC_NO_PIN,
        ULTRASONIC_NO_PIN },
    _sensorData{},
    _cycle{},
    _cycleLength(0),
    _enabled(false),
    _state(State::Idle),
    _activeSensor(UltrasonicSensor::Front),
    _cycleIndex(0),
    _stateStartedUs(0),
    _slotStartedUs(0),
    _sequence(0)
{}

void UltrasonicManager::begin(
    uint8_t frontTriggerPin,
    uint8_t frontEchoPin,
    uint8_t leftTriggerPin,
    uint8_t leftEchoPin,
    uint8_t rightTriggerPin,
    uint8_t rightEchoPin)
{
    _triggerPins[indexOf(UltrasonicSensor::Front)] =
        frontTriggerPin;

    _triggerPins[indexOf(UltrasonicSensor::Left)] =
        leftTriggerPin;

    _triggerPins[indexOf(UltrasonicSensor::Right)] =
        rightTriggerPin;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        if (_triggerPins[i] == ULTRASONIC_NO_PIN)
        {
            continue;
        }

        pinMode(_triggerPins[i], OUTPUT);
        digitalWrite(_triggerPins[i], LOW);
    }

    buildMeasurementCycle();

    clearMeasurements();

    _capture.begin(
        frontEchoPin,
        leftEchoPin,
        rightEchoPin
    );

    // Nach dem Einschalten des Nanos bleibt Ultraschall zunächst aus.
    _enabled = false;
    _sequence = 0;

    forceIdle();
}

bool UltrasonicManager::hasSensor(
    UltrasonicSensor sensor) const
{
    return _triggerPins[indexOf(sensor)] !=
        ULTRASONIC_NO_PIN;
}

void UltrasonicManager::buildMeasurementCycle()
{
    const bool hasFront =
        hasSensor(UltrasonicSensor::Front);

    const bool hasLeft =
        hasSensor(UltrasonicSensor::Left);

    const bool hasRight =
        hasSensor(UltrasonicSensor::Right);

    _cycleLength = 0;

    // Der Frontsensor zeigt in Fahrtrichtung und wird
    // deshalb doppelt so oft gemessen wie die Seiten -
    // aber nur, wenn er auf diesem Nano ueberhaupt sitzt.
    if (hasFront)
    {
        _cycle[_cycleLength++] = UltrasonicSensor::Front;
    }

    if (hasLeft)
    {
        _cycle[_cycleLength++] = UltrasonicSensor::Left;
    }

    if (hasFront && hasRight)
    {
        _cycle[_cycleLength++] = UltrasonicSensor::Front;
    }

    if (hasRight)
    {
        _cycle[_cycleLength++] = UltrasonicSensor::Right;
    }

    _cycleIndex = 0;
}

void UltrasonicManager::setEnabled(bool enabled)
{
    if (_enabled == enabled)
    {
        return;
    }

    _enabled = enabled;

    // Eine eventuell laufende Echo-Erfassung beenden.
    // Alle Triggerausgänge werden auf LOW gesetzt.
    forceIdle();

    if (_enabled)
    {
        // Jeder neue Fahrbefehl beginnt mit frischen Messwerten.
        // Alte Medianfilterwerte dürfen den neuen Fahrabschnitt
        // nicht beeinflussen.
        clearMeasurements();
    }
}

bool UltrasonicManager::isEnabled() const
{
    return _enabled;
}

void UltrasonicManager::update(uint32_t nowMs)
{
    if (!_enabled || _cycleLength == 0)
    {
        return;
    }

    const uint32_t nowUs = micros();

    switch (_state)
    {
    case State::Idle:
        startMeasurement(
            _cycle[_cycleIndex]
        );
        break;

    case State::WaitEcho:
    {
        EchoCaptureResult result = {};

        if (_capture.takeCompleted(result))
        {
            const uint32_t pulseUs =
                (uint32_t)(
                    result.fallUs -
                    result.riseUs
                    );

            // Entfernung in Millimetern:
            // pulse_us * 343 m/s / 2
            // = pulse_us * 343 / 2000 mm
            const uint32_t distanceMm =
                (pulseUs * 343UL + 1000UL) /
                2000UL;

            if (distanceMm > MAX_DISTANCE_MM ||
                pulseUs == 0)
            {
                rejectMeasurement(
                    _activeSensor
                );
            }
            else
            {
                uint16_t acceptedMm =
                    (uint16_t)distanceMm;

                // Werte unterhalb der Mindestdistanz
                // werden auf 20 mm begrenzt.
                if (acceptedMm < MIN_DISTANCE_MM)
                {
                    acceptedMm =
                        MIN_DISTANCE_MM;
                }

                acceptDistance(
                    _activeSensor,
                    acceptedMm,
                    nowMs
                );
            }

            _sequence++;

            enterGuard();
        }
        else if (
            (uint32_t)(
                nowUs -
                _stateStartedUs
                ) >= ECHO_TIMEOUT_US)
        {
            finishTimeout();
        }

        break;
    }

    case State::Guard:
    {
        const bool guardElapsed =
            (uint32_t)(
                nowUs -
                _stateStartedUs
                ) >= GUARD_TIME_US;

        const bool slotElapsed =
            (uint32_t)(
                nowUs -
                _slotStartedUs
                ) >= MEASUREMENT_SLOT_US;

        if (guardElapsed &&
            slotElapsed)
        {
            startNextMeasurement();
        }

        break;
    }
    }
}

void UltrasonicManager::makeSnapshot(
    uint32_t nowMs,
    UltrasonicSnapshot& snapshot) const
{
    const SensorData& front =
        _sensorData[
            indexOf(
                UltrasonicSensor::Front
            )
        ];

    const SensorData& left =
        _sensorData[
            indexOf(
                UltrasonicSensor::Left
            )
        ];

    const SensorData& right =
        _sensorData[
            indexOf(
                UltrasonicSensor::Right
            )
        ];

    snapshot.sequence = _sequence;

    snapshot.frontMm =
        front.filteredMm;

    snapshot.leftMm =
        left.filteredMm;

    snapshot.rightMm =
        right.filteredMm;

    snapshot.validMask = 0;

    if (front.hasValidValue &&
        front.latestAttemptValid)
    {
        snapshot.validMask |= 0x01;
    }

    if (left.hasValidValue &&
        left.latestAttemptValid)
    {
        snapshot.validMask |= 0x02;
    }

    if (right.hasValidValue &&
        right.latestAttemptValid)
    {
        snapshot.validMask |= 0x04;
    }

    snapshot.frontAgeMs =
        front.hasValidValue
        ? clampAgeMs(
            (uint32_t)(
                nowMs -
                front.lastValidMs
                )
        )
        : 65535U;

    snapshot.leftAgeMs =
        left.hasValidValue
        ? clampAgeMs(
            (uint32_t)(
                nowMs -
                left.lastValidMs
                )
        )
        : 65535U;

    snapshot.rightAgeMs =
        right.hasValidValue
        ? clampAgeMs(
            (uint32_t)(
                nowMs -
                right.lastValidMs
                )
        )
        : 65535U;
}

void UltrasonicManager::clearMeasurements()
{
    for (uint8_t i = 0;
        i < SENSOR_COUNT;
        i++)
    {
        _sensorData[i].filterValues[0] = 0;
        _sensorData[i].filterValues[1] = 0;
        _sensorData[i].filterValues[2] = 0;

        _sensorData[i].filterCount = 0;
        _sensorData[i].filterNext = 0;

        _sensorData[i].filteredMm = 0;
        _sensorData[i].lastValidMs = 0;

        _sensorData[i].hasValidValue = false;
        _sensorData[i].latestAttemptValid = false;
    }
}

void UltrasonicManager::forceIdle()
{
    _capture.cancel();

    for (uint8_t i = 0;
        i < SENSOR_COUNT;
        i++)
    {
        if (_triggerPins[i] == ULTRASONIC_NO_PIN)
        {
            continue;
        }

        digitalWrite(
            _triggerPins[i],
            LOW
        );
    }

    _state = State::Idle;

    // Der erste Sensor des Zyklus, nicht zwingend Front -
    // vorne gibt es nur die beiden Seitensensoren.
    _activeSensor =
        _cycleLength > 0
        ? _cycle[0]
        : UltrasonicSensor::Front;

    _cycleIndex = 0;

    _stateStartedUs = micros();
    _slotStartedUs = _stateStartedUs;
}

uint8_t UltrasonicManager::indexOf(
    UltrasonicSensor sensor)
{
    return static_cast<uint8_t>(
        sensor
        );
}

UltrasonicEchoChannel
UltrasonicManager::channelOf(
    UltrasonicSensor sensor)
{
    switch (sensor)
    {
    case UltrasonicSensor::Front:
        return
            UltrasonicEchoChannel::Front;

    case UltrasonicSensor::Left:
        return
            UltrasonicEchoChannel::Left;

    case UltrasonicSensor::Right:
        return
            UltrasonicEchoChannel::Right;

    default:
        return
            UltrasonicEchoChannel::None;
    }
}

uint16_t UltrasonicManager::clampAgeMs(
    uint32_t ageMs)
{
    return ageMs > 65535UL
        ? 65535U
        : (uint16_t)ageMs;
}

void UltrasonicManager::startMeasurement(
    UltrasonicSensor sensor)
{
    _activeSensor = sensor;

    _capture.arm(
        channelOf(sensor)
    );

    digitalWrite(
        _triggerPins[
            indexOf(sensor)
        ],
        HIGH
    );

    // Nur der 10-us-Triggerimpuls ist
    // bewusst kurz blockierend.
    delayMicroseconds(10);

    digitalWrite(
        _triggerPins[
            indexOf(sensor)
        ],
        LOW
    );

    _state = State::WaitEcho;

    _stateStartedUs = micros();

    // Beginn des festen Messplatzes.
    _slotStartedUs =
        _stateStartedUs;
}

void UltrasonicManager::finishTimeout()
{
    _capture.cancel();

    rejectMeasurement(
        _activeSensor
    );

    _sequence++;

    enterGuard();
}

void UltrasonicManager::enterGuard()
{
    if (hasSensor(_activeSensor))
    {
        digitalWrite(
            _triggerPins[
                indexOf(
                    _activeSensor
                )
            ],
            LOW
        );
    }

    _state = State::Guard;

    _stateStartedUs = micros();
}

void UltrasonicManager::startNextMeasurement()
{
    if (_cycleLength == 0)
    {
        return;
    }

    _cycleIndex++;

    if (_cycleIndex >= _cycleLength)
    {
        _cycleIndex = 0;
    }

    startMeasurement(
        _cycle[
            _cycleIndex
        ]
    );
}

void UltrasonicManager::acceptDistance(
    UltrasonicSensor sensor,
    uint16_t distanceMm,
    uint32_t nowMs)
{
    SensorData& data =
        _sensorData[
            indexOf(sensor)
        ];

    data.filterValues[
        data.filterNext
    ] = distanceMm;

    data.filterNext++;

    if (data.filterNext >= 3)
    {
        data.filterNext = 0;
    }

    if (data.filterCount < 3)
    {
        data.filterCount++;
    }

    data.filteredMm =
        filteredValue(data);

    data.lastValidMs = nowMs;

    data.hasValidValue = true;
    data.latestAttemptValid = true;
}

void UltrasonicManager::rejectMeasurement(
    UltrasonicSensor sensor)
{
    SensorData& data =
        _sensorData[
            indexOf(sensor)
        ];

    data.latestAttemptValid = false;
}

uint16_t UltrasonicManager::filteredValue(
    const SensorData& data)
{
    if (data.filterCount == 0)
    {
        return 0;
    }

    if (data.filterCount == 1)
    {
        return data.filterValues[0];
    }

    if (data.filterCount == 2)
    {
        return (uint16_t)(
            (
                (uint32_t)
                data.filterValues[0] +
                (uint32_t)
                data.filterValues[1]
                ) /
            2UL
            );
    }

    const uint16_t a =
        data.filterValues[0];

    const uint16_t b =
        data.filterValues[1];

    const uint16_t c =
        data.filterValues[2];

    if ((a <= b && b <= c) ||
        (c <= b && b <= a))
    {
        return b;
    }

    if ((b <= a && a <= c) ||
        (c <= a && a <= b))
    {
        return a;
    }

    return c;
}