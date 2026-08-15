#include "Dummy.h"

#include <Arduino.h>

namespace TPUart
{
namespace Interface
{

void Dummy::drainByTime()
{
    uint32_t now = micros();

    if (!_running || _writeUsed == 0 || _byteTimeUs == 0)
    {
        _lastDrainAt = now;
        return;
    }

    size_t drained = (size_t)((uint32_t)(now - _lastDrainAt) / _byteTimeUs);
    if (drained == 0) return;

    // Nur die vollen Bytezeiten verbrauchen, der Rest bleibt für die nächste Runde stehen.
    _lastDrainAt += (uint32_t)(drained * _byteTimeUs);
    _writeUsed = drained >= _writeUsed ? 0 : _writeUsed - drained;
}

void Dummy::addByte(char data, uint32_t pauseUs)
{
    _queue.push_back({data, pauseUs});
}

void Dummy::clearData()
{
    _queue.clear();
    _pos = 0;
    _nextAvailableAt = micros(); // Fahrplan neu aufsetzen, sonst wirkt der alte auf die neuen Bytes nach
}

const std::vector<uint8_t> &Dummy::writtenBytes() const
{
    return _written;
}

void Dummy::clearWrittenBytes()
{
    _written.clear();
}

void Dummy::forceOverflow(bool state)
{
    _forcedOverflow = state;
}

// Der Wert schrumpft beim Schreiben tatsächlich mit (wie bei den echten Interfaces) - sonst könnte ein
// Aufrufer, der mehr Bytes schreibt als er reserviert hat, im Test nie auffallen.
void Dummy::setWriteCapacity(size_t capacity)
{
    _writeCapacity = capacity;
    _writeUsed = 0;
}

void Dummy::drainWritten()
{
    _writeUsed = 0;
}

void Dummy::begin(uint32_t baud)
{
    // Aus der Baudrate die Bytezeit ableiten (8E1 = 11 Bit je Zeichen), damit der Sendepuffer im
    // selben Tempo leerläuft wie bei echter Hardware.
    _byteTimeUs = baud > 0 ? (uint32_t)(11000000UL / baud) : 0;

    _nextAvailableAt = micros();
    _lastDrainAt = _nextAvailableAt;
    _writeUsed = 0; // ein Neustart verwirft den Sendepuffer, wie end()/begin() bei der Hardware
    _running = true;
}

void Dummy::end()
{
    _running = false;
}

void Dummy::flush()
{
    _pos = _queue.size();
}

size_t Dummy::available()
{
    if (!_running) return 0;

    uint32_t now = micros();
    uint32_t arrivesAt = _nextAvailableAt;
    size_t count = 0;

    for (size_t i = _pos; i < _queue.size(); i++)
    {
        if ((int32_t)(now - arrivesAt) < 0) break;
        count++;
        arrivesAt += _queue[i]._pauseUs;
    }

    return count;
}

size_t Dummy::availableForWrite()
{
    drainByTime();
    return _running ? _writeCapacity - _writeUsed : 0;
}

int Dummy::read()
{
    if (!available()) return -1;

    // Der Fahrplan wird FORTGESCHRIEBEN, nicht auf jetzt neu bezogen: die Bytes kommen zu festen Zeiten
    // auf dem Bus an, unabhängig davon, wann wir sie abholen. Andernfalls würde sich ein aufgelaufener
    // Rückstand mit dem ersten gelesenen Byte auflösen und wäre nicht mehr nachstellbar.
    const QueuedByte &entry = _queue[_pos++];
    _nextAvailableAt += entry._pauseUs;
    return (unsigned char)entry._data;
}

bool Dummy::write(char value)
{
    if (!_running) return false;
    drainByTime();

    if (_writeUsed >= _writeCapacity) return false; // voll - wie bei den echten Interfaces

    _writeUsed++;
    if (_written.size() < WRITTEN_LIMIT) _written.push_back((uint8_t)value);
    return true;
}

bool Dummy::overflow()
{
    if (!_running) return false; // wie bei allen anderen Implementierungen
    if (!_forcedOverflow) return false;

    _forcedOverflow = false;
    return true;
}

} // namespace Interface
} // namespace TPUart
