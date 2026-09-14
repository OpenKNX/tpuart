#include "TPUart/Transmitter.h"

#include <stdlib.h>
#include <string.h>

#include <Arduino.h>

#include "TPUart/DataLinkLayer.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Statistics.h"

namespace TPUart
{

// Ein maximales Telegramm plus Reserve muss hineinpassen.
static_assert(TPUART_TX_BUFFER_SIZE >= TPUART_BUFFER_SIZE + TPUART_TX_PRIORITY_RESERVE,
              "TPUART_TX_BUFFER_SIZE muss ein maximales Telegramm zusätzlich zur Reserve fassen");

Transmitter::Transmitter(DataLinkLayer &dll) : _dll(dll) {}

Transmitter::~Transmitter() {}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft aus dem Tick
// ---------------------------------------------------------------------------------------------------

// Einziger Schreibzugriff auf das Interface - zählt für getTxBytes().
bool Transmitter::writeByte(uint8_t value)
{
    if (!_dll._interface->write((char)value)) return false;

    _dll._statistics.incrementTxBytes();
    return true;
}

// Höchstens eine Steuersequenz oder ein Telegramm-Oktett je Tick; Steuercodes haben Vorrang.
void Transmitter::process()
{
    // Ein wartender Steuercode belegt den Tick, auch wenn er noch nicht passt.
    if (processCtrlQueue()) return;

    // Im Busmonitor und beim Registerlesen ruht der Sendepfad samt Wachhund - vor dem Await-Zweig, sonst
    // schickte der Wachhund einen Reset. Die Steuercode-Warteschlange oben bleibt aktiv.
    if (_dll.isBusMonitor() || _dll.registerReadActive()) return;

    // Warten auf L_Data.con; hier nur der Wachhund.
    if (_state == TxState::Await)
    {
        if ((uint32_t)(millis() - _awaitSince) < TPUART_TX_CONFIRM_TIMEOUT_MS) return;

        // Keine Bestätigung: Reset. Die U_Reset.ind startet das Telegramm neu; bleibt sie aus, kommt der
        // nächste Reset nach derselben Frist.
        if (_dll._interface->availableForWrite() < 1) return;
        if (!writeByte(U_RESET_REQ)) return;

        _dll._statistics.incrementTxControlBytes();
        _dll._statistics.incrementTxConfirmTimeouts();
        _awaitSince = millis();
        _confirmTimeout = true; // gemeldet wird das aus loop(), hier darf nichts nach außen

        _dll.controlByteSent(U_RESET_REQ);
        return;
    }

    if (_state == TxState::Idle && !startNextTransmission()) return;

    bool last = (_bufferPos == _frameSize - 1); // das letzte Byte ist die Prüfsumme
    uint8_t offset = (uint8_t)(_bufferPos >> 6);

    // Das Offset-Byte vor der Platzprüfung bestimmen: verlangt werden 2 oder 3 Bytes, nicht pauschal 3.
    // Nur NCN512x - der TPUART2 kennt U_L_DataOffset nicht und sendet ohnehin höchstens 64 Oktette.
    bool needsOffset = _dll.bcuType() == BcuType::Ncn5120 && (!_chipOffsetValid || offset != _chipOffset);
    size_t needed = needsOffset ? 3 : 2;

    if (_dll._interface->availableForWrite() < needed) return;

    if (needsOffset)
    {
        writeByte((uint8_t)(U_L_DATA_OFFSET_REQ | offset));
        _chipOffset = offset;
        _chipOffsetValid = true;
    }

    writeByte((uint8_t)((last ? U_L_DATA_END_REQ : U_L_DATA_START_REQ) | (_bufferPos & U_L_DATA_POSITION_MASK)));
    writeByte(_buffer[_bufferPos]);

    _bufferPos++;

    if (!last) return;

    // Mit dem U_L_DataEnd.req beginnt der Chip die Übertragung auf den Bus.
    _dll._statistics.incrementTxFrames();
    _awaitSince = millis();
    _state = TxState::Await;
}

// Aus dem Tick: kopiert die Vorlage in den Sendepuffer. Freigegeben wird im Hauptkontext.
bool Transmitter::startNextTransmission()
{
    // Im Busmonitor ist die Vorlage überholt - verwerfen, damit der Hauptkontext räumen kann.
    if (_dll.isBusMonitor())
    {
        if (_stagedSeq != _takenSeq) _takenSeq = _takenSeq + 1;
        return false;
    }

    if (_stagedSeq == _takenSeq) return false;

    // Absicherung gegen Schreiben über den Sendepuffer hinaus.
    if (_stagedBuffer == nullptr || _stagedFrameSize == 0 || _stagedFrameSize > TPUART_BUFFER_SIZE)
    {
        _takenSeq = _takenSeq + 1;
        return false;
    }

    _frameSize = _stagedFrameSize;
    memcpy(_buffer, _stagedBuffer, _frameSize);

    // Zuletzt: ab hier darf der Hauptkontext den Platz freigeben.
    _takenSeq = _takenSeq + 1;

    beginTransmission();
    return true;
}

// Anfang einer Übertragung - neues Telegramm oder Neubeginn nach Reset. Der Offset im Chip gilt als unbekannt.
void Transmitter::beginTransmission()
{
    _bufferPos = 0;
    _chipOffsetValid = false;
    _state = TxState::Transmit;
}

// Setzt höchstens eine Steuersequenz ungeteilt ab. true auch, wenn sie noch nicht passt - der Telegrammpfad
// muss dann Platz machen, sonst käme eine 4-Byte-Gruppe während eines Telegramms nie durch.
bool Transmitter::processCtrlQueue()
{
    if (_ctrlQueueTail == _ctrlQueueHead) return false;

    uint32_t tail = _ctrlQueueTail;
    size_t length = _ctrlQueue[tail % TPUART_CTRL_QUEUE_SIZE];

    // Absicherung gegen einen korrupten Eintrag.
    if (length == 0 || length > TPUART_CTRL_MAX_GROUP)
    {
        _ctrlQueueTail = _ctrlQueueHead;
        _dll.reportControlOverflow();
        return false;
    }

    if (_dll._interface->availableForWrite() < length) return true;

    tail++;
    uint8_t code = _ctrlQueue[tail % TPUART_CTRL_QUEUE_SIZE];

    for (size_t i = 0; i < length; i++)
        writeByte(_ctrlQueue[tail++ % TPUART_CTRL_QUEUE_SIZE]);

    _ctrlQueueTail = tail;
    _dll._statistics.incrementTxControlBytes((uint32_t)length);

    // Erst jetzt gilt der Chip als umgeschaltet - abgeleitet aus dem gesendeten Code.
    _dll.controlByteSent(code);

    return true;
}

// Aus dem Tick (Receiver). Der TxState bleibt unberührt.
bool Transmitter::sendAcknowledge(AckType acknowledge)
{
    if (_dll._interface->availableForWrite() < 1) return false; // kein Platz - lieber nicht acken als blockieren

    if (!writeByte((uint8_t)(U_ACKN_REQ | (uint8_t)acknowledge))) return false;

    _dll._statistics.incrementTxAcknowledges();
    return true;
}

void Transmitter::echoReceived()
{
    if (_state != TxState::Await) return;

    _awaitSince = millis();
}

void Transmitter::confirmed()
{
    if (_state != TxState::Await) return;

    _state = TxState::Idle;
}

void Transmitter::restart()
{
    if (_state == TxState::Idle) return; // nichts unterwegs - dann gibt es auch nichts zu wiederholen

    beginTransmission();
}

// Bricht die laufende Übertragung ab (Wechsel in den Busmonitor) - nicht von vorn wie restart(). Läuft im
// Tick, damit _state einen Schreiber behält; die Warteschlange räumt stageNextTelegram().
void Transmitter::abort()
{
    if (_stagedSeq != _takenSeq) _takenSeq = _takenSeq + 1;

    if (_state == TxState::Idle) return;

    _bufferPos = 0;
    _frameSize = 0;
    _chipOffsetValid = false;
    _state = TxState::Idle;
}

// Vollständiges Telegramm. Das Wiederholungsbit (0x20) wird ausgenommen und die Prüfsumme nicht verglichen -
// die BCU löscht das Bit beim Wiederholen.
bool Transmitter::isEcho(const uint8_t *data, size_t length) const
{
    if (_state == TxState::Idle) return false;
    if (length != _frameSize) return false;
    if (((data[0] ^ _buffer[0]) & (uint8_t)~0x20) != 0) return false;

    return memcmp(data + 1, _buffer + 1, length - 2) == 0;
}

// Anfang eines laufenden Telegramms, für die Quittungsentscheidung - nicht entfernen, sonst quittiert das
// Gerät sein eigenes Echo.
bool Transmitter::isEchoPrefix(const uint8_t *data, size_t length) const
{
    if (_state == TxState::Idle) return false;
    // Strikt kürzer: bei voller Länge käme die Prüfsumme in den Vergleich, dafür ist isEcho() da.
    if (length == 0 || length >= _frameSize) return false;
    if (((data[0] ^ _buffer[0]) & (uint8_t)~0x20) != 0) return false;

    return memcmp(data + 1, _buffer + 1, length - 1) == 0;
}

// ---------------------------------------------------------------------------------------------------
// Gemütliche Seite - läuft aus dem Hauptloop
// ---------------------------------------------------------------------------------------------------

bool Transmitter::pushTransmitQueue(const Frame &frame)
{
    size_t length = frame.length();

    // Jede Ablehnung nennt ihren Grund über den Message-Callback.
    if (!_dll.isConnected())
    {
        _dll.printError("Send rejected: no connection");
        return false;
    }

    if (_dll.isBusMonitor()) // dort ist der Chip transparent und sendet nichts
    {
        _dll.printError("Send rejected: bus monitor active");
        return false;
    }

    // Ein vollständiges Standard-Telegramm hat mindestens 8 Oktetts einschließlich Prüfsumme.
    if (length < 8 || length > TPUART_BUFFER_SIZE)
    {
        _dll.printError("Send rejected: length %u out of range (8..%u)", (unsigned)length, (unsigned)TPUART_BUFFER_SIZE);
        return false;
    }

    // TPUART2: höchstens 63 Datenoktette plus Prüfsumme (Servicetabelle, Siemens S. 21).
    if (_dll.bcuType() == BcuType::Tpuart2 && length > 64)
    {
        _dll.printError("Send rejected: TPUART2 takes at most 63 byte plus checksum");
        return false;
    }

    // Vor jeder Änderung am Puffer prüfen: die Warteschlange leitet die Eintragsgrenzen aus dem Telegramm ab.
    // Die Prüfsumme wird geprüft, nicht neu gerechnet.
    if (!frame.isValid())
    {
        _dll.printError("Send rejected: not a well-formed telegram");
        return false;
    }

    if (!_queue.push(frame))
    {
        _dll._statistics.incrementTxQueueOverflows();
        _dll.printError("Send rejected: queue full (%u byte)", (unsigned)TPUART_TX_BUFFER_SIZE);
        return false;
    }

    _dll._statistics.updateTxQueuePeakBytes(_queue.used());

    // Gleich vorlegen, wenn der Sendeweg frei ist.
    stageNextTelegram();
    return true;
}

// Aus dem Hauptkontext: abgeholten Platz freigeben, im Busmonitor räumen, nächstes Telegramm vorlegen.
// Geräumt wird vor dem Vorlegen; der gepinnte Eintrag fällt erst, wenn _takenSeq nachgezogen hat.
void Transmitter::stageNextTelegram()
{
    if (_stagedSeq != _takenSeq) return; // die Vorlage liegt noch, der Tick hat sie nicht abgeholt

    if (_queue.pinned()) _queue.pop();

    _stagedBuffer = nullptr;
    _stagedFrameSize = 0;

    if (_dll.isBusMonitor()) _queue.clear();

    size_t length = 0;
    const uint8_t *data = _queue.front(length);

    // Nach front(), dort wird das Flag gesetzt.
    if (_queue.corrupted())
        _dll.printError("Transmit queue corrupt - dropped");

    if (data == nullptr) return;

    _stagedBuffer = data;
    _stagedFrameSize = length;
    _queue.pin(); // ab jetzt bewegt den Eintrag nichts mehr, auch keine Aufnahme

    _stagedSeq = _stagedSeq + 1; // ZULETZT - erst damit wird die Vorlage für den Tick sichtbar
}

bool Transmitter::queueControl(uint8_t code)
{
    return queueControl(&code, 1);
}

// Reiht eine Steuersequenz ein - auch während eines laufenden Telegramms.
bool Transmitter::queueControl(const uint8_t *codes, size_t length)
{
    if (!_dll.isConnected()) return false; // vorher regelt die Verbindungsaufnahme den Chip
    if (length == 0 || length > TPUART_CTRL_MAX_GROUP) return false;

    uint32_t needed = (uint32_t)(1 + length);
    uint32_t used = _ctrlQueueHead - _ctrlQueueTail;

    if (TPUART_CTRL_QUEUE_SIZE - used < needed)
    {
        _dll.reportControlOverflow();
        return false;
    }

    _dll._statistics.updateTxControlQueuePeakBytes(used + needed);

    // Kopf zuletzt - der Tick sieht nie eine halbe Sequenz.
    uint32_t head = _ctrlQueueHead;
    _ctrlQueue[head++ % TPUART_CTRL_QUEUE_SIZE] = (uint8_t)length;

    for (size_t i = 0; i < length; i++)
        _ctrlQueue[head++ % TPUART_CTRL_QUEUE_SIZE] = codes[i];

    _ctrlQueueHead = head;
    return true;
}

TxState Transmitter::state() const
{
    return _state;
}

bool Transmitter::isTransmitting() const
{
    return _state != TxState::Idle;
}

// In Bytes, nicht in Telegrammen.
uint32_t Transmitter::queueUsed() const
{
    return (uint32_t)_queue.used();
}

uint32_t Transmitter::queueSize() const
{
    return (uint32_t)TPUART_TX_BUFFER_SIZE;
}

bool Transmitter::confirmTimeout()
{
    if (!_confirmTimeout) return false;

    _confirmTimeout = false;
    return true;
}

} // namespace TPUart
