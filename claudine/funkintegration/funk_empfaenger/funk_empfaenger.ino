// ============================================================
// File: funk_empfaenger.ino
// Zweck:
//  - Empfaengt die Telemetriefragmente des vorderen Nano
//  - Setzt sie zu vollstaendigen Zeilen zusammen
//  - Gibt sie ueber Serial aus, im Format des Frontprogramms
//
// Der Empfaenger sendet keine Steuerbefehle zurueck.
//
// Verdrahtung wie beim Sender:
//   CE   D7
//   CSN  D8
//   MOSI D11
//   MISO D12
//   SCK  D13
//
// Das Projekt bindet 1_Common ein und benutzt dieselbe
// RadioProtocol.h wie der Sender. Es gibt bewusst keine Kopie
// im Sketchordner, die man bei Protokollaenderungen vergessen
// koennte.
// ============================================================

#include <SPI.h>
#include <RF24.h>

#include "src/RadioProtocol.h"

RF24 radio(7, 8);

// ------------------------------------------------------------
// Zusammensetzen
// ------------------------------------------------------------

char lineBuffer[RadioProtocol::MAX_LINE_LENGTH + 1];

uint16_t activeMessageId = 0;
uint8_t expectedFragment = 0;
uint8_t expectedCount = 0;
uint8_t lineLength = 0;
bool assembling = false;

// ------------------------------------------------------------
// Zaehler
// ------------------------------------------------------------

uint32_t receivedPackets = 0;
uint32_t droppedPackets = 0;
uint32_t duplicatePackets = 0;
uint32_t incompleteMessages = 0;
uint32_t completeMessages = 0;

const uint32_t STATUS_INTERVAL_MS = 5000;
uint32_t lastStatusMs = 0;

bool radioReady = false;

// ------------------------------------------------------------

static void discardAssembly()
{
    if (assembling)
    {
        ++incompleteMessages;
    }

    assembling = false;
    expectedFragment = 0;
    expectedCount = 0;
    lineLength = 0;
}

static bool packetIsPlausible(const RadioProtocol::Packet& packet)
{
    if (packet.version != RadioProtocol::VERSION)
    {
        return false;
    }

    if (packet.fragmentCount == 0 ||
        packet.fragmentCount > RadioProtocol::MAX_FRAGMENT_COUNT)
    {
        return false;
    }

    if (packet.fragmentIndex >= packet.fragmentCount)
    {
        return false;
    }

    if (packet.payloadLength == 0 ||
        packet.payloadLength > RadioProtocol::PAYLOAD_SIZE)
    {
        return false;
    }

    // Alle Fragmente ausser dem letzten sind voll. Ist das
    // nicht so, stimmt der Versatz beim Zusammensetzen nicht.
    if (packet.fragmentIndex + 1 < packet.fragmentCount &&
        packet.payloadLength != RadioProtocol::PAYLOAD_SIZE)
    {
        return false;
    }

    return true;
}

// Der Sender wiederholt ein Paket, wenn das ACK verloren geht.
// Ein bereits eingebautes Fragment darf deshalb nicht dazu
// fuehren, dass die halbe Nachricht weggeworfen wird.
static bool isDuplicate(const RadioProtocol::Packet& packet)
{
    if (!assembling)
    {
        return false;
    }

    if (packet.messageId != activeMessageId)
    {
        return false;
    }

    return packet.fragmentIndex < expectedFragment;
}

static void printCompleteMessage()
{
    lineBuffer[lineLength] = '\0';
    Serial.println(lineBuffer);

    ++completeMessages;
}

static void processPacket(const RadioProtocol::Packet& packet)
{
    ++receivedPackets;

    if (!packetIsPlausible(packet))
    {
        ++droppedPackets;
        discardAssembly();
        return;
    }

    if (isDuplicate(packet))
    {
        ++duplicatePackets;
        return;
    }

    if (packet.fragmentIndex == 0)
    {
        // Neue Nachricht. Eine noch offene alte gilt damit als
        // unvollstaendig.
        discardAssembly();

        assembling = true;
        activeMessageId = packet.messageId;
        expectedCount = packet.fragmentCount;
        expectedFragment = 0;
        lineLength = 0;
    }

    if (!assembling ||
        packet.messageId != activeMessageId ||
        packet.fragmentCount != expectedCount ||
        packet.fragmentIndex != expectedFragment ||
        (uint16_t)lineLength + packet.payloadLength >
        RadioProtocol::MAX_LINE_LENGTH)
    {
        ++droppedPackets;
        discardAssembly();
        return;
    }

    memcpy(
        lineBuffer + lineLength,
        packet.payload,
        packet.payloadLength);

    lineLength = (uint8_t)(lineLength + packet.payloadLength);
    ++expectedFragment;

    if (expectedFragment == expectedCount)
    {
        printCompleteMessage();
        assembling = false;
    }
}

static void printStatus()
{
    Serial.print(F("#RSTAT,"));
    Serial.print(receivedPackets);   Serial.print(',');
    Serial.print(completeMessages);  Serial.print(',');
    Serial.print(droppedPackets);    Serial.print(',');
    Serial.print(duplicatePackets);  Serial.print(',');
    Serial.println(incompleteMessages);
}

// ------------------------------------------------------------

static bool initRadio()
{
    if (!radio.begin())
    {
        return false;
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(76);
    radio.setPayloadSize(sizeof(RadioProtocol::Packet));

    radio.openReadingPipe(1, RadioProtocol::ADDRESS);
    radio.startListening();

    return true;
}

void setup()
{
    Serial.begin(115200);

    radioReady = initRadio();

    if (radioReady)
    {
        Serial.println(F("#RADIO,ready"));
    }
    else
    {
        Serial.println(F("#RADIO,error,init"));
    }

    lastStatusMs = millis();
}

void loop()
{
    const uint32_t now = millis();

    if (!radioReady)
    {
        // Nicht stumm bleiben: der Fehler muss am Monitor
        // sichtbar sein.
        if ((uint32_t)(now - lastStatusMs) >= STATUS_INTERVAL_MS)
        {
            lastStatusMs = now;
            Serial.println(F("#RADIO,error,init"));
        }

        return;
    }

    while (radio.available())
    {
        RadioProtocol::Packet packet;
        radio.read(&packet, sizeof(packet));

        processPacket(packet);
    }

    if ((uint32_t)(now - lastStatusMs) >= STATUS_INTERVAL_MS)
    {
        lastStatusMs = now;
        printStatus();
    }
}
