#include "RadioTelemetrySender.h"
#include "Nrf24Radio.h"

RadioTelemetrySender::RadioTelemetrySender(Nrf24Radio& radio)
    : _radio(radio), _writePos(0), _lenPos(0), _pending(0),
      _lineOpen(false), _lineOverflow(false), _readPos(0),
      _committed(0), _sending(false), _dataPos(0), _lineLength(0),
      _fragmentIndex(0), _fragmentCount(0), _messageId(0),
      _sentLines(0), _droppedLines(0), _ringPeak(0)
{}

uint8_t RadioTelemetrySender::nextPos(uint8_t pos)
{
    ++pos;
    return pos >= RING_SIZE ? 0 : pos;
}

void RadioTelemetrySender::beginLine()
{
    if (_lineOpen)
    {
        rollbackLine();
        ++_droppedLines;
    }

    _lineOpen = true;
    _lineOverflow = false;
    _pending = 0;

    if ((uint16_t)_committed + 1u > RING_SIZE)
    {
        _lineOverflow = true;
        return;
    }

    _lenPos = _writePos;
    _writePos = nextPos(_writePos);
    _pending = 1;
}

void RadioTelemetrySender::addLineByte(char value)
{
    if (!_lineOpen || _lineOverflow) return;

    if ((uint8_t)(_pending - 1) >= RadioProtocol::MAX_LINE_LENGTH ||
        (uint16_t)_committed + _pending >= RING_SIZE)
    {
        _lineOverflow = true;
        return;
    }

    _ring[_writePos] = value;
    _writePos = nextPos(_writePos);
    ++_pending;
}

void RadioTelemetrySender::commitLine()
{
    if (!_lineOpen) return;
    _lineOpen = false;

    if (_lineOverflow || _pending <= 1)
    {
        rollbackLine();
        ++_droppedLines;
        return;
    }

    _ring[_lenPos] = (char)(uint8_t)(_pending - 1);
    _committed = (uint8_t)(_committed + _pending);
    _pending = 0;
    if (_committed > _ringPeak) _ringPeak = _committed;
}

void RadioTelemetrySender::rollbackLine()
{
    if (_pending > 0) _writePos = _lenPos;
    _pending = 0;
    _lineOpen = false;
    _lineOverflow = false;
}

bool RadioTelemetrySender::startNextMessage()
{
    if (_committed == 0) return false;

    _lineLength = (uint8_t)_ring[_readPos];
    _dataPos = nextPos(_readPos);
    _fragmentCount = (uint8_t)((_lineLength + RadioProtocol::PAYLOAD_SIZE - 1) /
                               RadioProtocol::PAYLOAD_SIZE);
    _fragmentIndex = 0;
    if (++_messageId == 0) _messageId = 1;
    _sending = true;
    return true;
}

void RadioTelemetrySender::finishMessage(bool success)
{
    _readPos = (uint8_t)(((uint16_t)_dataPos + _lineLength) % RING_SIZE);
    _committed = (uint8_t)(_committed - (_lineLength + 1));
    _sending = false;
    if (success) ++_sentLines;
    else ++_droppedLines;
}

void RadioTelemetrySender::update()
{
    if (!_sending && !startNextMessage()) return;

    RadioProtocol::Packet packet = {};
    packet.version = RadioProtocol::VERSION;
    packet.messageId = _messageId;
    packet.fragmentIndex = _fragmentIndex;
    packet.fragmentCount = _fragmentCount;

    const uint8_t offset =
        (uint8_t)(_fragmentIndex * RadioProtocol::PAYLOAD_SIZE);
    uint8_t length = (uint8_t)(_lineLength - offset);
    if (length > RadioProtocol::PAYLOAD_SIZE)
        length = RadioProtocol::PAYLOAD_SIZE;
    packet.payloadLength = length;

    uint8_t pos =
        (uint8_t)(((uint16_t)_dataPos + offset) % RING_SIZE);
    for (uint8_t i = 0; i < length; ++i)
    {
        packet.payload[i] = _ring[pos];
        pos = nextPos(pos);
    }

    if (!_radio.send(packet))
    {
        finishMessage(false);
        return;
    }

    ++_fragmentIndex;
    if (_fragmentIndex >= _fragmentCount) finishMessage(true);
}

bool RadioTelemetrySender::busy() const
{
    return _sending || _committed > 0;
}

uint32_t RadioTelemetrySender::sentLines() const { return _sentLines; }
uint32_t RadioTelemetrySender::droppedLines() const { return _droppedLines; }
uint8_t RadioTelemetrySender::ringPeak() const { return _ringPeak; }
