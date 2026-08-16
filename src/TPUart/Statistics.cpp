#include "TPUart/Statistics.h"

#include <Arduino.h>

// Die Zähler stehen im Byte-Pfad des Ticks und sind damit der am häufigsten durchlaufene Code der Library.
// Sie sind trotzdem hier und nicht als inline-Rümpfe im Header: ein Inkrement ist ein Speicherzugriff plus
// Funktionsaufruf, gegen die 500µs zwischen zwei Ticks ist das nicht messbar. Ein Tick kostet gemessen
// 2-3µs, davon geht das hier im Rauschen unter.
namespace TPUart
{

void Statistics::reset()
{
    _rxBytes = 0;
    _rxFrames = 0;
    _rxInvalidFrames = 0;
    _rxFrameBytes = 0;
    _rxControlBytes = 0;
    _rxDroppedBytes = 0;
    _rxRepeatedFrames = 0;
    _txControlBytes = 0;
    _txFrames = 0;
    _txBytes = 0;
    _txAcknowledges = 0;
    _txAcknowledgesSuppressed = 0;
    _txConfirmTimeouts = 0;
    _rxInterfaceOverflows = 0;
    _rxQueueOverflows = 0;
    _txControlQueueOverflows = 0;
    _txQueueOverflows = 0;
    _connectionLosses = 0;
    _rxResyncs = 0;
    _chipSlaveCollisions = 0;
    _chipReceiveErrors = 0;
    _chipTransmitErrors = 0;
    _chipProtocolErrors = 0;
    _chipTemperatureWarnings = 0;
    _ticks = 0;
    _tickDeferrals = 0;
    _tickLastDeferredUs = 0;
    _tickDurationMaxUs = 0;
    _checkAcknowledgeMaxUs = 0;
    _rxInterfacePeakBytes = 0;
    _rxQueuePeakBytes = 0;
    _txControlQueuePeakBytes = 0;
    _txQueuePeakBytes = 0;

    for (uint8_t i = 0; i < BUS_LOAD_WINDOW; i++)
        _busLoadSamples[i] = {};

    _busLoadIndex = 0;
    _busLoadCount = 0;
}

// --- Empfang -------------------------------------------------------------------------------------------

void Statistics::incrementRxBytes(uint32_t increment)
{
    _rxBytes += increment;
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

void Statistics::incrementRxRepeatedFrames(uint32_t increment)
{
    _rxRepeatedFrames += increment;
}

uint32_t Statistics::getRxBytes() const
{
    return _rxBytes;
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

uint32_t Statistics::getRxRepeatedFrames() const
{
    return _rxRepeatedFrames;
}

// --- Versand -------------------------------------------------------------------------------------------

void Statistics::incrementTxBytes(uint32_t increment)
{
    _txBytes += increment;
}

void Statistics::incrementTxControlBytes(uint32_t increment)
{
    _txControlBytes += increment;
}

void Statistics::incrementTxFrames(uint32_t increment)
{
    _txFrames += increment;
}

void Statistics::incrementTxAcknowledges(uint32_t increment)
{
    _txAcknowledges += increment;
}

void Statistics::incrementTxAcknowledgesSuppressed(uint32_t increment)
{
    _txAcknowledgesSuppressed += increment;
}

void Statistics::incrementTxConfirmTimeouts(uint32_t increment)
{
    _txConfirmTimeouts += increment;
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

uint32_t Statistics::getTxAcknowledges() const
{
    return _txAcknowledges;
}

uint32_t Statistics::getTxAcknowledgesSuppressed() const
{
    return _txAcknowledgesSuppressed;
}

uint32_t Statistics::getTxConfirmTimeouts() const
{
    return _txConfirmTimeouts;
}

// --- Verluste und Verbindung ---------------------------------------------------------------------------

void Statistics::incrementRxInterfaceOverflows(uint32_t increment)
{
    _rxInterfaceOverflows += increment;
}

void Statistics::incrementRxQueueOverflows(uint32_t increment)
{
    _rxQueueOverflows += increment;
}

void Statistics::incrementTxControlQueueOverflows(uint32_t increment)
{
    _txControlQueueOverflows += increment;
}

void Statistics::incrementTxQueueOverflows(uint32_t increment)
{
    _txQueueOverflows += increment;
}

void Statistics::incrementConnectionLosses(uint32_t increment)
{
    _connectionLosses += increment;
}

uint32_t Statistics::getRxInterfaceOverflows() const
{
    return _rxInterfaceOverflows;
}

uint32_t Statistics::getRxQueueOverflows() const
{
    return _rxQueueOverflows;
}

uint32_t Statistics::getTxControlQueueOverflows() const
{
    return _txControlQueueOverflows;
}

uint32_t Statistics::getTxQueueOverflows() const
{
    return _txQueueOverflows;
}

uint32_t Statistics::getConnectionLosses() const
{
    return _connectionLosses;
}

void Statistics::incrementRxResyncs(uint32_t increment)
{
    _rxResyncs += increment;
}

void Statistics::incrementChipSlaveCollisions(uint32_t increment)
{
    _chipSlaveCollisions += increment;
}

void Statistics::incrementChipReceiveErrors(uint32_t increment)
{
    _chipReceiveErrors += increment;
}

void Statistics::incrementChipTransmitErrors(uint32_t increment)
{
    _chipTransmitErrors += increment;
}

void Statistics::incrementChipProtocolErrors(uint32_t increment)
{
    _chipProtocolErrors += increment;
}

void Statistics::incrementChipTemperatureWarnings(uint32_t increment)
{
    _chipTemperatureWarnings += increment;
}

// --- Takt ----------------------------------------------------------------------------------------------

void Statistics::recordTick()
{
    // Nicht _ticks++, sondern die Form der übrigen Zähler: ++ auf einem volatile ist seit C++20 als
    // deprecated markiert und der ESP32-Build übersetzt mit dieser Norm (-Wvolatile).
    _ticks += 1;
}

void Statistics::recordTickDeferred(uint32_t deferredUs)
{
    _tickDeferrals += 1;
    _tickLastDeferredUs = deferredUs;
}

uint32_t Statistics::getTickDeferrals() const
{
    return _tickDeferrals;
}

uint32_t Statistics::getTickLastDeferredUs() const
{
    return _tickLastDeferredUs;
}

void Statistics::updateTickDurationMaxUs(uint32_t durationUs)
{
    if (durationUs > _tickDurationMaxUs) _tickDurationMaxUs = durationUs;
}

uint32_t Statistics::getTickDurationMaxUs() const
{
    return _tickDurationMaxUs;
}

void Statistics::updateCheckAcknowledgeMaxUs(uint32_t durationUs)
{
    if (durationUs > _checkAcknowledgeMaxUs) _checkAcknowledgeMaxUs = durationUs;
}

uint32_t Statistics::getCheckAcknowledgeMaxUs() const
{
    return _checkAcknowledgeMaxUs;
}

void Statistics::updateRxInterfacePeakBytes(uint32_t pending)
{
    if (pending > _rxInterfacePeakBytes) _rxInterfacePeakBytes = pending;
}

uint32_t Statistics::getTicks() const
{
    return _ticks;
}

uint32_t Statistics::getRxInterfacePeakBytes() const
{
    return _rxInterfacePeakBytes;
}

void Statistics::updateRxQueuePeakBytes(uint32_t used)
{
    if (used > _rxQueuePeakBytes) _rxQueuePeakBytes = used;
}

void Statistics::updateTxControlQueuePeakBytes(uint32_t used)
{
    if (used > _txControlQueuePeakBytes) _txControlQueuePeakBytes = used;
}

void Statistics::updateTxQueuePeakBytes(uint32_t used)
{
    if (used > _txQueuePeakBytes) _txQueuePeakBytes = used;
}

uint32_t Statistics::getRxResyncs() const
{
    return _rxResyncs;
}

uint32_t Statistics::getChipSlaveCollisions() const
{
    return _chipSlaveCollisions;
}

uint32_t Statistics::getChipReceiveErrors() const
{
    return _chipReceiveErrors;
}

uint32_t Statistics::getChipTransmitErrors() const
{
    return _chipTransmitErrors;
}

uint32_t Statistics::getChipProtocolErrors() const
{
    return _chipProtocolErrors;
}

uint32_t Statistics::getChipTemperatureWarnings() const
{
    return _chipTemperatureWarnings;
}

uint32_t Statistics::getRxQueuePeakBytes() const
{
    return _rxQueuePeakBytes;
}

uint32_t Statistics::getTxControlQueuePeakBytes() const
{
    return _txControlQueuePeakBytes;
}

uint32_t Statistics::getTxQueuePeakBytes() const
{
    return _txQueuePeakBytes;
}

// --- Buslast -------------------------------------------------------------------------------------------

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
    _busLoadSamples[_busLoadIndex] = {getRxFrameBytes(), now};

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
    uint32_t delta = getRxFrameBytes() - _busLoadSamples[oldest]._bytes;
    return (delta * 1000 + elapsed / 2) / elapsed;
}

// --- KOMPAT, siehe Header --------------------------------------------------------------------------------

uint32_t Statistics::getRxReceivedBytes() const
{
    return getRxBytes();
}

uint32_t Statistics::getRxRepetitions() const
{
    return getRxRepeatedFrames();
}

uint32_t Statistics::getRxDiscardedBytes() const
{
    return getRxDroppedBytes();
}

uint32_t Statistics::getRxBusBytes() const
{
    return getRxFrameBytes();
}

uint32_t Statistics::getRxUartOverflow() const
{
    return getRxInterfaceOverflows();
}

uint32_t Statistics::getRxFrameBufferOverflow() const
{
    return getRxQueueOverflows();
}

uint32_t Statistics::getTxOverflowFrameBuffer() const
{
    return getTxQueueOverflows();
}

uint32_t Statistics::getRxSearchBufferOverflow() const
{
    return 0;
}

} // namespace TPUart
