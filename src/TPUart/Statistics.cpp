#include "TPUart/Statistics.h"

#include <Arduino.h>

// Die Zähler stehen im Byte-Pfad von tick() und sind damit der am häufigsten durchlaufene Code der Library.
// Sie sind trotzdem hier und nicht als inline-Rümpfe im Header: ein Inkrement ist ein Speicherzugriff plus
// Funktionsaufruf, gegen die 500µs zwischen zwei Ticks ist das nicht messbar. Ein Tick kostet gemessen
// 2-3µs, davon geht das hier im Rauschen unter.
namespace TPUart
{

void Statistics::reset()
{
    _rxReceivedBytes = 0;
    _rxFrames = 0;
    _rxInvalidFrames = 0;
    _rxFrameBytes = 0;
    _rxControlBytes = 0;
    _rxDroppedBytes = 0;
    _rxRepetitions = 0;
    _txControlBytes = 0;
    _txFrames = 0;
    _txBytes = 0;
    _acknowledgesSent = 0;
    _acknowledgesSuppressed = 0;
    _interfaceOverflows = 0;
    _rxQueueOverflows = 0;
    _ctrlQueueOverflows = 0;
    _txQueueOverflows = 0;
    for (uint8_t i = 0; i < BUS_LOAD_WINDOW; i++)
        _busLoadSamples[i] = {};

    _busLoadIndex = 0;
    _busLoadCount = 0;
}

void Statistics::incrementRxReceivedBytes(uint32_t increment)
{
    _rxReceivedBytes += increment;
}

void Statistics::incrementRxFrames(uint32_t increment)
{
    _rxFrames += increment;
}

void Statistics::incrementRxInvalidFrames(uint32_t increment)
{
    _rxInvalidFrames += increment;
}

void Statistics::incrementRxFrameBytes(uint32_t increment)
{
    _rxFrameBytes += increment;
}

void Statistics::incrementRxControlBytes(uint32_t increment)
{
    _rxControlBytes += increment;
}

void Statistics::incrementRxDroppedBytes(uint32_t increment)
{
    _rxDroppedBytes += increment;
}

void Statistics::incrementRxRepetitions(uint32_t increment)
{
    _rxRepetitions += increment;
}

void Statistics::incrementTxControlBytes(uint32_t increment)
{
    _txControlBytes += increment;
}

void Statistics::incrementTxFrames(uint32_t increment)
{
    _txFrames += increment;
}

void Statistics::incrementAcknowledgesSent(uint32_t increment)
{
    _acknowledgesSent += increment;
}

void Statistics::incrementAcknowledgesSuppressed(uint32_t increment)
{
    _acknowledgesSuppressed += increment;
}

void Statistics::incrementInterfaceOverflows(uint32_t increment)
{
    _interfaceOverflows += increment;
}

void Statistics::incrementRxQueueOverflows(uint32_t increment)
{
    _rxQueueOverflows += increment;
}

void Statistics::incrementCtrlQueueOverflows(uint32_t increment)
{
    _ctrlQueueOverflows += increment;
}

void Statistics::incrementTxQueueOverflows(uint32_t increment)
{
    _txQueueOverflows += increment;
}

uint32_t Statistics::getRxReceivedBytes() const
{
    return _rxReceivedBytes;
}

uint32_t Statistics::getRxFrames() const
{
    return _rxFrames;
}

uint32_t Statistics::getRxInvalidFrames() const
{
    return _rxInvalidFrames;
}

uint32_t Statistics::getRxFrameBytes() const
{
    return _rxFrameBytes;
}

uint32_t Statistics::getRxControlBytes() const
{
    return _rxControlBytes;
}

uint32_t Statistics::getRxDroppedBytes() const
{
    return _rxDroppedBytes;
}

uint32_t Statistics::getRxRepetitions() const
{
    return _rxRepetitions;
}

void Statistics::incrementTxBytes(uint32_t increment)
{
    _txBytes += increment;
}

uint32_t Statistics::getTxBytes() const
{
    return _txBytes;
}

uint32_t Statistics::getTxControlBytes() const
{
    return _txControlBytes;
}

uint32_t Statistics::getTxFrames() const
{
    return _txFrames;
}

uint32_t Statistics::getAcknowledgesSent() const
{
    return _acknowledgesSent;
}

uint32_t Statistics::getAcknowledgesSuppressed() const
{
    return _acknowledgesSuppressed;
}

uint32_t Statistics::getInterfaceOverflows() const
{
    return _interfaceOverflows;
}

uint32_t Statistics::getRxQueueOverflows() const
{
    return _rxQueueOverflows;
}

uint32_t Statistics::getCtrlQueueOverflows() const
{
    return _ctrlQueueOverflows;
}

uint32_t Statistics::getTxQueueOverflows() const
{
    return _txQueueOverflows;
}

// --- KOMPAT, siehe Header -----------------------------------------------------------------------------

uint32_t Statistics::getRxDiscardedBytes() const
{
    return getRxDroppedBytes();
}

uint32_t Statistics::getRxUartOverflow() const
{
    return getInterfaceOverflows();
}

uint32_t Statistics::getRxFrameBufferOverflow() const
{
    return getRxQueueOverflows();
}

uint32_t Statistics::getTxOverflowFrameBuffer() const
{
    return getTxQueueOverflows();
}

// DUMMY: den SearchBuffer gibt es in dieser Library nicht mehr - hier kann nichts überlaufen.
uint32_t Statistics::getRxSearchBufferOverflow() const
{
    return 0;
}

uint32_t Statistics::getRxBusBytes() const
{
    return _rxFrameBytes;
}

void Statistics::sampleBusLoad()
{
    uint32_t now = millis();

    // Der zuletzt abgelegte Eintrag gibt den Takt vor. Ist der Ring noch leer, wird sofort einer angelegt -
    // er ist der Startpunkt, aus dem sich die erste Spanne ergibt.
    if (_busLoadCount > 0)
    {
        uint8_t last = (uint8_t)((_busLoadIndex + BUS_LOAD_WINDOW - 1) % BUS_LOAD_WINDOW);
        if ((uint32_t)(now - _busLoadSamples[last]._at) < BUS_LOAD_SLICE_MS) return;
    }

    // Der älteste Eintrag fällt hier heraus - das Fenster wandert um eine Scheibe weiter.
    _busLoadSamples[_busLoadIndex] = {getRxBusBytes(), now};

    _busLoadIndex = (uint8_t)((_busLoadIndex + 1) % BUS_LOAD_WINDOW);
    if (_busLoadCount < BUS_LOAD_WINDOW) _busLoadCount++;
}

uint32_t Statistics::getBusLoad() const
{
    // Noch keine Momentaufnahme, also auch kein Anfangspunkt.
    if (_busLoadCount == 0) return 0;

    // Der ÄLTESTE Eintrag: bei vollem Ring der Platz, der als Nächstes überschrieben wird; solange er
    // erst anläuft, der Platz 0. Das Ende der Messung ist kein Eintrag, sondern das JETZT.
    uint8_t oldest = _busLoadCount == BUS_LOAD_WINDOW ? _busLoadIndex : 0;

    uint32_t elapsed = millis() - _busLoadSamples[oldest]._at;
    if (elapsed == 0) return 0;

    // Eine Division über die volle Fensterbreite - derselbe gleitende Mittelwert, den die einzeln
    // gemittelten Teilraten ergäben, nur ohne die Zwischenwerte und mit einer Rundung statt dreien.
    // Geteilt wird durch die GEMESSENE Spanne, die Breite muss also nicht exakt stimmen. Der
    // Zwischenwert bleibt in 32 Bit, solange die Differenz unter 4,29 Mio Bytes liegt - bei 738 Byte/s
    // Buslast wären das über anderthalb Stunden innerhalb EINES Fensters von drei Sekunden.
    uint32_t delta = getRxBusBytes() - _busLoadSamples[oldest]._bytes;
    return (delta * 1000 + elapsed / 2) / elapsed;
}

} // namespace TPUart
