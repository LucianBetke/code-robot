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

    // Timing
    unsigned long _lastPing;
    unsigned long _lastSeen;

    // Buffer fuer eingehende Zeichen
    char _buf[64];
    uint8_t _idx;

    // Empfangene vollstaendige Nutzdatenzeile
    char _line[64];
    bool _lineAvailable;

    static constexpr unsigned long PING_INTERVAL = 500;
    static constexpr unsigned long TIMEOUT = 2000;
};

#endif // UARTLINK_H