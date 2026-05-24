#include "UartLink.h"

#include <string.h>
#include <avr/pgmspace.h>

static bool equalsProgmem(const char* text, PGM_P pattern)
{
    return strcmp_P(text, pattern) == 0;
}

static void copyLine(char* dst, const char* src, uint8_t dstSize)
{
    if (dstSize == 0) return;

    uint8_t i = 0;
    const uint8_t maxIndex = dstSize - 1;

    while (i < maxIndex && src[i] != '\0')
    {
        dst[i] = src[i];
        i++;
    }

    dst[i] = '\0';
}

UartLink::UartLink(Stream& link, bool initiator)
    : _link(link),
    _initiator(initiator),
    _connected(false),
    _lastPing(0),
    _lastSeen(0),
    _lastActivity(0),
    _idx(0),
    _lineAvailable(false)
{
}

void UartLink::begin()
{
    const unsigned long now = millis();

    _connected = false;
    _lastPing = 0;
    _lastSeen = now;
    _lastActivity = now;

    _idx = 0;
    _lineAvailable = false;

    _buf[0] = '\0';
    _line[0] = '\0';
}

void UartLink::update()
{
    const unsigned long now = millis();

    // ========================================================
    // RX: Zeichen einlesen
    // ========================================================
    while (_link.available())
    {
        const char c = (char)_link.read();

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            _buf[_idx] = '\0';

            // Jede vollstaendige empfangene Zeile beweist:
            // Die Gegenstelle lebt.
            _lastSeen = now;
            _lastActivity = now;

            // ------------------------------------------------
            // Handshake: Front -> Rear
            // ------------------------------------------------
            if (!_initiator && equalsProgmem(_buf, PSTR("PING")))
            {
                _link.println(F("PONG"));
                _lastActivity = now;
            }

            // ------------------------------------------------
            // Handshake: Rear -> Front
            // ------------------------------------------------
            else if (_initiator && equalsProgmem(_buf, PSTR("PONG")) && !_connected)
            {
                _link.println(F("ACK"));
                _connected = true;
                _lastActivity = now;
            }

            // ------------------------------------------------
            // Handshake-Abschluss am Rear
            // ------------------------------------------------
            else if (!_initiator && equalsProgmem(_buf, PSTR("ACK")))
            {
                _connected = true;
            }

            // ------------------------------------------------
            // KA:
            // Wird intern verarbeitet, aber NICHT als Nutzdaten
            // an availableLine() / getLine() weitergegeben.
            // Dadurch sieht Python/Plot/PC-Log diese Zeile nicht.
            // ------------------------------------------------
            else if (equalsProgmem(_buf, PSTR("KA")))
            {
                // absichtlich leer
            }

            // ------------------------------------------------
            // Debug-Zeilen der Gegenstelle nicht als Protokoll-
            // Nutzdaten behandeln.
            // ------------------------------------------------
            else if (_buf[0] == '#')
            {
                // absichtlich leer
            }

            // ------------------------------------------------
            // Normale Nutzdatenzeile
            // ------------------------------------------------
            else
            {
                copyLine(_line, _buf, sizeof(_line));
                _lineAvailable = true;
            }

            _idx = 0;
        }
        else if (_idx < sizeof(_buf) - 1)
        {
            _buf[_idx] = c;
            _idx++;
        }
        else
        {
            // Ueberlange Zeile verwerfen
            _idx = 0;
        }
    }

    // ========================================================
    // TX: Verbindungsaufbau
    //
    // Nur der Initiator sendet PING, solange noch keine
    // Verbindung besteht. Das ist dein Front-Nano.
    // ========================================================
    if (!_connected)
    {
        if (_initiator && (now - _lastPing > PING_INTERVAL))
        {
            _lastPing = now;
            _link.println(F("PING"));
            _lastActivity = now;
        }

        return;
    }

    // ========================================================
    // TX: KeepAlive
    //
    // Wichtige Aenderung:
    //
    // Der Initiator sendet im verbundenen Zustand KEIN KA mehr.
    // Bei deinem Aufbau ist der Initiator der Front-Nano, und
    // dessen Serial-TX ist im PC-Monitor sichtbar.
    //
    // Deshalb sendet nur der Rear-Nano KA, und auch nur dann,
    // wenn laenger keine andere UART-Aktivitaet lief.
    // Front empfaengt KA, aktualisiert _lastSeen, gibt KA aber
    // nicht als Nutzdaten weiter.
    // ========================================================
    if (!_initiator && (now - _lastActivity > KEEPALIVE_INTERVAL))
    {
        _link.println(F("KA"));
        _lastActivity = now;
    }

    // ========================================================
    // Timeout
    //
    // Der Front-Nano erkennt Verbindungsverlust sichtbar ueber
    // ConnectionMonitor/#DISCONNECTED.
    //
    // Beim Rear-Nano ist zusaetzlich updateVsolTimeout() wichtig:
    // Wenn waehrend aktiver Fahrt keine VSOL-Frames mehr kommen,
    // werden die hinteren Raeder gestoppt.
    // ========================================================
    if (_initiator && _connected && (now - _lastSeen > TIMEOUT))
    {
        _connected = false;
    }
}

void UartLink::sendLine(const char* msg)
{
    if (!_connected) return;
    if (!msg) return;

    _link.println(msg);
    _lastActivity = millis();
}

bool UartLink::availableLine() const
{
    return _lineAvailable;
}

const char* UartLink::getLine()
{
    _lineAvailable = false;
    return _line;
}

bool UartLink::isConnected() const
{
    return _connected;
}