// UartLink.h
#ifndef UARTLINK_H
#define UARTLINK_H

#include <Arduino.h>

class UartLink
{
public:
    UartLink(Stream& link, bool initiator = false);

    void begin();
    void update();

    void sendLine(const char* msg);

    bool isConnected() const;
    bool availableLine() const;
    const char* getLine();

private:
    Stream& _link;
    bool _initiator;
    bool _connected;

    unsigned long _lastPing;
    unsigned long _lastSeen;

    static constexpr uint8_t LINE_LENGTH = 64;

    char _buf[LINE_LENGTH];
    uint8_t _idx;

    // Zwei Nutzdatenzeilen werden gepuffert.
    // So koennen US und VSOL_OK/VIST kurz nacheinander eintreffen.
    static constexpr uint8_t LINE_QUEUE_DEPTH = 2;

    char _lineQueue[LINE_QUEUE_DEPTH][LINE_LENGTH];

    uint8_t _lineHead;
    uint8_t _lineTail;
    uint8_t _lineCount;

    static constexpr unsigned long PING_INTERVAL = 500;
    static constexpr unsigned long TIMEOUT = 2000;

    void enqueueLine(const char* line);
};

#endif // UARTLINK_H