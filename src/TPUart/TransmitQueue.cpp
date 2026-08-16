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

    // Eingefügt wird am ENDE des eigenen Rangs - damit bleibt die Reihenfolge innerhalb einer Klasse die
    // Eingangsreihenfolge, und Low landet ganz hinten, also ohne jede Verschiebung. Nur ein höherpriores
    // Telegramm schiebt den Rest nach rechts, und das ist der seltene Fall.
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

    // UNPLAUSIBEL HEISST HIER: der Puffer ist nicht mehr zu deuten. Ohne Längenpräfix beginnt der nächste
    // Eintrag an der Stelle, die dieser hier ausrechnet - stimmt sie nicht, ist alles dahinter Unsinn.
    // Erreichbar ist das nur über einen Rechenfehler in der Verschiebung, denn hereingelassen wird
    // ausschließlich Geprüftes. Genau deshalb wird nicht weitergelesen, sondern verworfen und gemeldet -
    // dasselbe Vorgehen wie bei einem korrupten Eintrag im Empfangsring.
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

    // Leere Ränge zeigen sonst weiterhin vor den gepinnten Eintrag, und eine spätere Aufnahme landete
    // mitten darin. Das trifft jeden Rang, der VOR dem gepinnten liegt - war der gepinnte Low, sind das
    // alle drei oberen.
    for (uint8_t i = 0; i < TP_PRIORITY_COUNT; i++)
        if (_end[i] < _head) _end[i] = _head;
}

void TransmitQueue::pop()
{
    if (!_pinned) return;

    _pinned = false;
    compact();
}

// HIER UND NUR HIER wird kompaktiert, und der Zeitpunkt ist mit Bedacht gewählt: pop() hebt den Pin
// gerade auf, es liegt also nichts fest, was sich nicht bewegen dürfte.
//
// Der frühere Entwurf kompaktierte erst unmittelbar vor dem nächsten Vorlegen. Dazwischen lag totes
// Gebiet am linken Rand, und eine Aufnahme in diesem Fenster konnte abgelehnt werden, obwohl insgesamt
// Platz frei war - ein Überlauf, den es nicht geben darf. So bleibt die Invariante lückenlos: nach pop()
// ist _head == 0, und totes Gebiet gibt es zu keinem Zeitpunkt.
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
    // Der gepinnte Eintrag BLEIBT: der Tick kann noch daraus lesen, und ihn hier wegzuräumen wäre genau
    // der Wettlauf, den die Vorlage vermeidet. Er fällt beim nächsten pop().
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

    // Nur die niedrigste Klasse wird begrenzt. Alles darüber darf den Puffer voll ausschöpfen - sonst
    // schützte die Reserve niemanden, sie verschöbe den Engpass nur.
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
