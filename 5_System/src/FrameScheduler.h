// FrameScheduler.h
#ifndef FRAME_SCHEDULER_H
#define FRAME_SCHEDULER_H

#include <stdint.h>

// ============================================================
// FrameScheduler
//
// Verwaltet das Messraster eines aktiven CMDT-Befehls.
//
// Aufgabe:
//  - Startzeit eines Befehls merken
//  - naechsten Messzeitpunkt verwalten
//  - pruefen, ob ein Frame faellig ist
//  - Frame-Zeit relativ zum Befehlsstart liefern
//
// Bewusst NICHT enthalten:
//  - UART
//  - RearFrameClient
//  - CommandRunner
//  - Printer
//  - VehicleController
// ============================================================

class FrameScheduler
{
public:
    FrameScheduler();

    void begin(uint32_t intervalMs);

    void start(uint32_t nowMs);
    void stop();

    bool isRunning() const;

    bool due(uint32_t nowMs, uint32_t& frameTimeMs);

private:
    static bool timeReached(uint32_t nowMs, uint32_t targetMs);

    uint32_t _intervalMs;
    uint32_t _startMs;
    uint32_t _nextFrameMs;
    bool _running;
};

#endif // FRAME_SCHEDULER_H