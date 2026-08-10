// ============================================================
// File: RadioProtocol.h
// Zweck:
//  - Gemeinsame Definition des Funkpakets fuer Sender und
//    Empfaenger
//  - Enthaelt keine Hardwarezugriffe und keine Telemetrie-
//    inhalte
//
// Paketformat, genau 32 Byte:
//
//   Byte  0     version         Protokollversion, hier 1
//   Byte  1-2   messageId       fortlaufende Nachrichtennummer,
//                               beginnt bei 1, ueberspringt 0
//   Byte  3     fragmentIndex   Teilnummer, beginnend bei 0
//   Byte  4     fragmentCount   Gesamtzahl Teile, 1 bis 5
//   Byte  5     payloadLength   gueltige Nutzbytes, 1 bis 26
//   Byte  6-31  payload         Ausschnitt der Telemetriezeile
//                               ohne Zeilenende
//
// Alle Fragmente einer Nachricht ausser dem letzten sind
// vollstaendig gefuellt. Der Empfaenger darf sich darauf
// verlassen und muss es pruefen.
// ============================================================

#ifndef RADIO_PROTOCOL_H
#define RADIO_PROTOCOL_H

#include <Arduino.h>

namespace RadioProtocol
{
    static const uint8_t VERSION = 1;

    static const uint8_t PAYLOAD_SIZE = 26;

    // 5 Fragmente reichen fuer jede Zeile, die die vorhandene
    // Firmware erzeugt. Die laengste ist
    //   #EVENT,startCmdp,...  mit 114 Zeichen im Extremfall.
    // Eine kleinere Obergrenze begrenzt zugleich die maximale
    // Sendedauer einer einzelnen Zeile.
    static const uint8_t MAX_FRAGMENT_COUNT = 5;

    static const uint8_t MAX_LINE_LENGTH =
        PAYLOAD_SIZE * MAX_FRAGMENT_COUNT;   // 130

    // 5 Byte Adresse plus abschliessendes Nullbyte.
    // RF24 wertet nur die ersten 5 Byte aus.
    static const uint8_t ADDRESS[6] = "IGOR1";

    struct __attribute__((packed)) Packet
    {
        uint8_t  version;
        uint16_t messageId;
        uint8_t  fragmentIndex;
        uint8_t  fragmentCount;
        uint8_t  payloadLength;
        char     payload[PAYLOAD_SIZE];
    };

    static_assert(
        sizeof(Packet) == 32,
        "nRF24 Paket muss genau 32 Byte gross sein");
}

#endif // RADIO_PROTOCOL_H
