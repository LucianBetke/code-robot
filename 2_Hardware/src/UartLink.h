//UartLink.h
#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>

class UartLink
{
public:
    UartLink(uint8_t rx, uint8_t tx);

    void begin(unsigned long baud);   // startet UART + Handshake
    void update();                    // muss in loop() laufen
    bool isConnected() const;

private:
    SoftwareSerial _link;

    bool _connected;
    unsigned long _lastPing;
    unsigned long _lastSeen;

    static const unsigned long PING_INTERVAL = 500;
    static const unsigned long TIMEOUT = 1500;

    char _buf[32];

    void _handshake();
};