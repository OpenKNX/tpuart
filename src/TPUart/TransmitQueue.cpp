#include "TPUart/TransmitQueue.h"

#include <string.h>

#include "TPUart/Frame.h"

namespace TPUart
{

bool TransmitQueue::push(const Frame &frame)
{
    size_t length = frame.length();
    if (length == 0) return false;

    const uint8_t *data = (const uint8_t *)frame.data();
    uint8_t rank = telegramPriorityRank(data[0]);
    if (rank >= TP_PRIORITY_COUNT) return false; // kann nicht eintreten, siehe telegramPriorityRank

    if (length > freeFor(rank)) return false;

    // Am Ende des eigenen Rangs einfügen - Eingangsreihenfolge je Rang, Low ohne Verschiebung.
    size_t offset = _end[rank];

    memmove(_buffer + offset + length, _buffer + offset, _end[TP_PRIORITY_COUNT - 1] - offset);
    memcpy(_buffer + offset, data, length);

    for (uint8_t i = rank; i < TP_PRIORITY_COUNT; i++)
        _end[i] += length;

    return true;
}

const uint8_t *TransmitQueue::front(size_t &length)
{
    length = 0;

    if (empty()) return nullptr;

    size_t available = _end[TP_PRIORITY_COUNT - 1] - _head;
    size_t size = Frame::sizeOf(_buffer + _head, available);

    // Unplausibel: ohne Längenpräfix ist alles dahinter nicht mehr deutbar - verwerfen und melden.
    if (size == 0 || size > available)
    {
        _corrupted = true;
        clear();
        return nullptr;
    }

    length = size;
    return _buffer + _head;
}

void TransmitQueue::pin()
{
    if (_pinned) return;

    size_t length = 0;
    if (front(length) == nullptr) return;

    _pinned = true;
    _head += length;

    // Ränge vor dem gepinnten Eintrag nachziehen - sonst landete eine Aufnahme mitten darin.
    for (uint8_t i = 0; i < TP_PRIORITY_COUNT; i++)
        if (_end[i] < _head) _end[i] = _head;
}

void TransmitQueue::pop()
{
    if (!_pinned) return;

    _pinned = false;
    compact();
}

// Nur aus pop(), wenn nichts gepinnt ist. Danach ist _head == 0 - totes Gebiet ließe Aufnahmen trotz Platz scheitern.
void TransmitQueue::compact()
{
    if (_head == 0) return;

    size_t total = _end[TP_PRIORITY_COUNT - 1];

    memmove(_buffer, _buffer + _head, total - _head);

    for (uint8_t i = 0; i < TP_PRIORITY_COUNT; i++)
        _end[i] = _end[i] > _head ? _end[i] - _head : 0;

    _head = 0;
}

void TransmitQueue::clear()
{
    // Der gepinnte Eintrag bleibt - der Tick liest womöglich noch daraus.
    for (uint8_t i = 0; i < TP_PRIORITY_COUNT; i++)
        _end[i] = _head;
}

bool TransmitQueue::empty() const
{
    return _end[TP_PRIORITY_COUNT - 1] == _head;
}

bool TransmitQueue::pinned() const
{
    return _pinned;
}

size_t TransmitQueue::used() const
{
    return _end[TP_PRIORITY_COUNT - 1];
}

size_t TransmitQueue::freeFor(uint8_t rank) const
{
    size_t limit = TPUART_TX_BUFFER_SIZE;

    // Nur Low wird begrenzt.
    if (rank == TP_PRIORITY_LOW)
    {
        if (limit <= (size_t)TPUART_TX_PRIORITY_RESERVE) return 0;
        limit -= (size_t)TPUART_TX_PRIORITY_RESERVE;
    }

    size_t occupied = _end[TP_PRIORITY_COUNT - 1];

    return occupied >= limit ? 0 : limit - occupied;
}

bool TransmitQueue::corrupted()
{
    if (!_corrupted) return false;

    _corrupted = false;
    return true;
}

} // namespace TPUart
