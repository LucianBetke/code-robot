// UartLink.h

#pragma once
#include <Arduino.h>

class UartLink
{
public:
    UartLink(Stream& link);

    void begin(unsigned long baud);
    void update();

    void sendLine(const char* msg);
    bool isConnected() const;

private:
    Stream& _link;

    bool _connected;

    // Handshake-Status
    bool _gotPong;
    bool _gotAck;

    // Timing
    unsigned long _lastPing;
    unsigned long _lastSeen;

    // Buffer
    char _buf[64];

    static constexpr unsigned long PING_INTERVAL = 500;
    static constexpr unsigned long TIMEOUT = 2000;
};