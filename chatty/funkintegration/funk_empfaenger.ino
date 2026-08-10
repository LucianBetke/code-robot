#include <SPI.h>
#include <RF24.h>
#include "RadioProtocol.h"

RF24 radio(7, 8);

char lineBuffer[RadioProtocol::MAX_LINE_LENGTH + 1];
uint16_t activeMessageId = 0;
uint8_t expectedFragment = 0;
uint8_t expectedCount = 0;
uint8_t lineLength = 0;
bool assembling = false;

uint32_t receivedPackets = 0;
uint32_t droppedPackets = 0;
uint32_t incompleteMessages = 0;
uint32_t duplicatePackets = 0;
uint32_t completeMessages = 0;

static void discardAssembly()
{
    if (assembling) ++incompleteMessages;
    assembling = false;
    expectedFragment = 0;
    expectedCount = 0;
    lineLength = 0;
}

static void processPacket(const RadioProtocol::Packet& packet)
{
    ++receivedPackets;
    if (packet.version != RadioProtocol::VERSION ||
        packet.fragmentCount == 0 ||
        packet.fragmentCount > RadioProtocol::MAX_FRAGMENT_COUNT ||
        packet.fragmentIndex >= packet.fragmentCount ||
        packet.payloadLength == 0 ||
        packet.payloadLength > RadioProtocol::PAYLOAD_SIZE ||
        (packet.fragmentIndex + 1 < packet.fragmentCount &&
         packet.payloadLength != RadioProtocol::PAYLOAD_SIZE))
    {
        ++droppedPackets;
        discardAssembly();
        return;
    }

    if (assembling &&
        packet.messageId == activeMessageId &&
        packet.fragmentIndex < expectedFragment)
    {
        ++duplicatePackets;
        return;
    }

    if (packet.fragmentIndex == 0)
    {
        discardAssembly();
        assembling = true;
        activeMessageId = packet.messageId;
        expectedCount = packet.fragmentCount;
        expectedFragment = 0;
        lineLength = 0;
    }

    if (!assembling || packet.messageId != activeMessageId ||
        packet.fragmentCount != expectedCount ||
        packet.fragmentIndex != expectedFragment ||
        (uint16_t)lineLength + packet.payloadLength > RadioProtocol::MAX_LINE_LENGTH)
    {
        ++droppedPackets;
        discardAssembly();
        return;
    }

    memcpy(lineBuffer + lineLength, packet.payload, packet.payloadLength);
    lineLength += packet.payloadLength;
    ++expectedFragment;

    if (expectedFragment == expectedCount)
    {
        lineBuffer[lineLength] = '\0';
        Serial.println(lineBuffer);
        ++completeMessages;
        assembling = false;
    }
}

void setup()
{
    Serial.begin(115200);
    if (!radio.begin())
    {
        Serial.println(F("#RADIO_ERROR,init"));
        return;
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(76);
    radio.setPayloadSize(sizeof(RadioProtocol::Packet));
    radio.openReadingPipe(1, RadioProtocol::ADDRESS);
    radio.startListening();
    Serial.println(F("#RADIO_READY"));
}

void loop()
{
    if (!radio.isChipConnected()) return;
    while (radio.available())
    {
        RadioProtocol::Packet packet;
        radio.read(&packet, sizeof(packet));
        processPacket(packet);
    }
}
