#include "TPUart/Receiver.h"

#include <string.h>

#include <Arduino.h>

#include "TPUart/DataLinkLayer.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Statistics.h"
#include "TPUart/Transmitter.h"

namespace TPUart
{

// Jedes gültige Frame muss in den Puffer passen (Extended: 9 + 254). LG 255 wird zur Laufzeit abgelehnt.
static_assert(TPUART_BUFFER_SIZE >= 9 + 254, "TPUART_BUFFER_SIZE muss das größtmögliche gültige Extended-Frame fassen");

static_assert(TPUART_RX_QUEUE_SIZE > TPUART_BUFFER_SIZE + TPUART_RX_QUEUE_HEADER_SIZE, "TPUART_RX_QUEUE_SIZE muss mindestens ein größtmögliches Telegramm fassen");

Receiver::Receiver(DataLinkLayer &dll) : _dll(dll) {}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft aus dem Tick
// ---------------------------------------------------------------------------------------------------

void Receiver::process()
{
    size_t pending = _dll._interface->available();
    if (!pending)
    {
        checkPause();
        return;
    }

    // Rückstand im Interface; gesund sind 0-1.
    _dll._statistics.updateRxInterfacePeakBytes((uint32_t)pending);

    int value = _dll._interface->read();
    if (value < 0)
    {
        checkPause();
        return;
    }

    // Nur hier, wenn Daten fließen - overflow() löscht beim RP2040 das Hardware-Flag.
    if (_dll._interface->overflow()) _dll.reportInterfaceOverflow();

    _dll._statistics.incrementRxBytes();

    // Jedes Byte ist ein Lebenszeichen der BCU.
    _dll._lastReceivedAt = millis();

    _emptyStarted = false;

    processByte((uint8_t)value);
}

// Eine Pause gilt erst, wenn das Interface TPUART_FRAME_WAIT_US lang ununterbrochen leer war.
void Receiver::checkPause()
{
    // In Idle hat eine Pause keine Wirkung - die Telegrammlänge kommt aus dem Längenbyte.
    if (_state == RxState::Idle) return;

    uint32_t now = micros();

    if (!_emptyStarted)
    {
        _emptyStarted = true;
        _emptySince = now;
        return;
    }

    uint32_t threshold = (_state == RxState::FrameAck) ? TPUART_FRAME_ACK_US : TPUART_FRAME_WAIT_US;

    if ((uint32_t)(now - _emptySince) < threshold) return;

    handleVerifiedPause();
}

// Eine verifizierte Pause ist eine Frame-Grenze. Jeder Zweig endet in Idle - darauf verlässt sich checkPause().
void Receiver::handleVerifiedPause()
{
    switch (_state)
    {
        // Keine Antwort gekommen: ohne Quittungs-Flags melden. Den Sendeweg gibt der Wachhund frei.
        case RxState::FrameAck:
            completeSequence(0, RxState::Idle);
            return;

        // Abgeschnitten: als kaputt melden. Kein Resync nötig, die Pause ist die Grenze.
        case RxState::Frame:
        case RxState::Control:
            completeSequence(TP_FRAME_FLAG_INVALID, RxState::Idle);
            return;

        // Nur das Steuerbyte ist der Normalfall; steht mehr im Puffer, wurde der Zyklus abgeschnitten.
        case RxState::Poll:
            completeSequence(_bufferPos > 1 ? TP_FRAME_FLAG_INVALID : 0, RxState::Idle);
            return;

        case RxState::Resync:
            resetSequence(RxState::Idle);
            return;

        default:
            return;
    }
}

void Receiver::processByte(uint8_t value)
{
    switch (_state)
    {
        // Position unbekannt - verwerfen bis zur nächsten verifizierten Pause, aber zählen.
        case RxState::Resync:
            _dll._statistics.incrementRxDroppedBytes();
            return;

        case RxState::Control:
            _buffer[1] = value;
            _bufferPos = 2;
            _dll._statistics.incrementRxControlBytes(); // das zweite Byte eines U_SystemStat.ind
            completeSequence(0, RxState::Idle);
            return;

        // Die Antwort zum fertigen Telegramm - sie geht als Flag mit, nicht als Byte.
        case RxState::FrameAck:
            // Busmonitor: die Quittung vom Bus (Figure 35).
            if (_dll.isBusMonitor() && (value & L_ACKN_MASK) == L_ACKN_IND)
            {
                // Beide Bit-Paare sind invertiert zu lesen: gesetzte Maskenbits heißen "nicht busy"/"nicht nack".
                bool nack = !(value & L_ACKN_NACK_MASK);
                bool busy = !(value & L_ACKN_BUSY_MASK);
                _flags |= acknowledgeFlags(nack ? AckType::Nack : (busy ? AckType::Busy : AckType::Addressed));

                completeSequence(0, RxState::Idle);
                return;
            }

            // Eigener Versand: L_Data.con. DATA_CON = eine Bestätigung kam, NACK = sie war negativ. BUSY gibt es
            // hier nicht - das L_Data.con trägt nur ein Bit.
            if ((value & L_DATA_CON_MASK) == L_DATA_CON)
            {
                uint8_t flags = TP_FRAME_FLAG_DATA_CON | TP_FRAME_FLAG_ACK;
                if (!(value & 0x80)) flags |= TP_FRAME_FLAG_ACK_NACK;

                // Erst melden, dann freigeben - sonst fehlte das TX-Flag (isEcho braucht den belegten Sendeweg).
                completeSequence(flags, RxState::Idle);

                _dll._transmitter.confirmed(); // Sendeweg frei, egal wie die Bestätigung ausfiel
                return;
            }

            // Im 8-Bit-UART-Modus geht dem L_Data.con ein U_FrameState.ind voraus (NCN5130 S. 42) - verwerfen.
            if ((value & U_FRAME_STATE_MASK) == U_FRAME_STATE_IND) return;

            // Etwas anderes, meist der Anfang einer Wiederholung: Telegramm melden und das Byte neu verarbeiten.
            completeSequence(0, RxState::Idle);
            processByte(value);
            return;

        case RxState::Frame:
            processFrameByte(value);
            return;

        case RxState::Poll:
            processPollByte(value);
            return;

        case RxState::Idle:
        {
            if (_awaitRegisterValue)
            {
                _awaitRegisterValue = false;
                _dll._statistics.incrementRxControlBytes();
                _dll.registerValueReceived(value);
                return;
            }

            bool isFrameStart = (value & L_DATA_MASK) == L_DATA_STANDARD_IND || (value & L_DATA_MASK) == L_DATA_EXTENDED_IND;

            if (!isFrameStart)
            {
                processControlByte(value);
                return;
            }

            resetSequence(RxState::Frame);
            processFrameByte(value);
            return;
        }

        default:
            return;
    }
}

void Receiver::processFrameByte(uint8_t value)
{
    // Die Prüfsumme fließt nicht in die CRC ein, sie wird nur verglichen.
    bool isChecksumByte = _frameSize > 0 && (_bufferPos == _frameSize - 1);

    // Absicherung gegen Schreiben hinter den Puffer.
    if (_bufferPos < TPUART_BUFFER_SIZE) _buffer[_bufferPos] = value;
    _bufferPos++;

    // Je Byte gezählt, nicht am Sequenzende - sonst landet die Buslast im falschen Messfenster.
    _dll._statistics.incrementRxFrameBytes();

    if (!isChecksumByte) _crc ^= value;

    if (_frameSize == 0)
    {
        // 0 heißt "Kopf noch nicht vollständig".
        size_t size = Frame::sizeOf(_buffer, _bufferPos);

        if (size > 0)
        {
            _frameSize = size;

            // Nur bei LG 255 (reserviert): Länge korrupt, Ende unbekannt - kaputt melden und Resync.
            if (_frameSize > TPUART_BUFFER_SIZE)
            {
                completeSequence(TP_FRAME_FLAG_INVALID, RxState::Resync);
                return;
            }

            // Läuft je Frame genau einmal - hier stehen Ziel, Adresstyp und Restlänge erstmals fest.
            sendAcknowledge();
        }
    }

    if (_frameSize > 0 && _bufferPos == _frameSize)
    {
        bool valid = (uint8_t)(~_crc) == _buffer[_frameSize - 1];

        // Bei falscher Prüfsumme ist das Frame-Ende unsicher - erst nach einer Pause wieder aufsetzen.
        if (!valid)
        {
            completeSequence(TP_FRAME_FLAG_INVALID, RxState::Resync);
            return;
        }

        bool echo = _dll._transmitter.isEcho(_buffer, _bufferPos);

        // Das Echo zeigt, dass der Chip gerade sendet - es schiebt den Wachhund weiter.
        if (echo) _dll._transmitter.echoReceived();

        // Auf eine Antwort warten: Quittung im Busmonitor bzw. L_Data.con zum eigenen Telegramm.
        if (_dll.isBusMonitor() || echo)
        {
            _state = RxState::FrameAck;
            return;
        }

        completeSequence(0, RxState::Idle);
    }
}

// Poll-Telegramm nach demselben Muster wie processFrameByte(); die Prüfsumme steht nach dem Kopf, die Slots
// dahinter deckt sie nicht ab. Hier wird nie quittiert.
void Receiver::processPollByte(uint8_t value)
{
    bool isChecksumByte = _bufferPos == L_POLL_DATA_HEADER_SIZE - 1;

    if (_bufferPos < TPUART_BUFFER_SIZE) _buffer[_bufferPos] = value;
    _bufferPos++;

    _dll._statistics.incrementRxFrameBytes();

    if (!isChecksumByte) _crc ^= value;

    if (isChecksumByte)
    {
        // Der Slot-Count steht direkt vor der Prüfsumme.
        uint8_t slots = _buffer[L_POLL_DATA_HEADER_SIZE - 2];

        // Zu viele Slots oder falsche Prüfsumme: Ende unbekannt, Resync.
        if (slots > L_POLL_DATA_MAX_SLOTS || (uint8_t)(~_crc) != value)
        {
            completeSequence(TP_FRAME_FLAG_INVALID, RxState::Resync);
            return;
        }

        _frameSize = L_POLL_DATA_HEADER_SIZE + slots;
    }

    // Alle Slots da - fertig, ohne auf eine Pause zu warten.
    if (_frameSize > 0 && _bufferPos == _frameSize) completeSequence(0, RxState::Idle);
}

// Entscheidet über das Acknowledge und lässt es sofort absetzen - noch während des Frames.
void Receiver::sendAcknowledge()
{
    // Im Busmonitor quittiert der Chip nichts, und wir auch nicht.
    if (_dll.isBusMonitor()) return;

    // Das eigene Echo wird nicht quittiert. Geprüft wird der Anfang gegen das laufende Telegramm, NICHT
    // "Sendeweg belegt" - sonst bliebe in TxState::Await jedes fremde Telegramm unquittiert.
    if (_dll._transmitter.isEchoPrefix(_buffer, _bufferPos)) return;

    // Immer fragen: ADDRESSED gilt auch, wenn die Quittung nicht mehr rechtzeitig rausgeht.
    bool extended = (_buffer[0] & L_DATA_MASK) == L_DATA_EXTENDED_IND;
    bool isGroupAddress = extended ? (_buffer[1] & 0x80) != 0 : (_buffer[5] & 0x80) != 0;
    uint16_t destination = extended ? (uint16_t)((_buffer[4] << 8) | _buffer[5]) : (uint16_t)((_buffer[3] << 8) | _buffer[4]);

    AckType acknowledge = _dll.checkAcknowledge(destination, isGroupAddress);
    if (acknowledge == AckType::None) return;

    _flags |= TP_FRAME_FLAG_ADDRESSED;

    // Busy-Modus mit aktiver Auto-Quittung: der Chip sagt BUSY ab. Ein U_Ackn.req von uns würde den Modus beenden.
    if (_dll.isBusyMode() && _dll.isAutoAcknowledge()) return;

    // Kein Schweigen bei Auto-Quittung: die deckt nur die eigene physikalische Adresse ab.

    // Liegt der Rest des Frames schon bereit, ist das Acknowledge-Fenster zu - ein U_Ackn.req träfe das
    // nächste Telegramm. >= und nicht >: bei Gleichheit ist das Frame schon komplett.
    if (_dll._interface->available() >= (_frameSize - _bufferPos))
    {
        _dll._statistics.incrementTxAcknowledgesSuppressed();
        return;
    }

    if (!_dll._transmitter.sendAcknowledge(acknowledge)) return;

    _flags |= acknowledgeFlags(acknowledge);

    // Ein U_Ackn.req beendet den Busy-Modus im Chip (S. 35).
    if (_dll.isBusyMode()) _dll.reportBusyModeCancelled();
}

// Steuerbytes sind 1 Byte lang - außer U_SystemStat.ind mit einem Folgebyte.
void Receiver::processControlByte(uint8_t value)
{
    _buffer[0] = value;
    _bufferPos = 1;

    // Anfang eines Poll-Telegramms, kein Steuerbyte.
    if (value == L_POLL_DATA_IND)
    {
        resetSequence(RxState::Poll);
        processPollByte(value);
        return;
    }

    // Nach dem Poll-Zweig: der zählt sein Byte als Telegrammbyte.
    _dll._statistics.incrementRxControlBytes();

    // Nur der NCN512x kennt U_SystemStat.ind; beim TPUART2 ist 0x4B etwas anderes.
    if (value == U_SYSTEM_STAT_IND && _dll.bcuType() == BcuType::Ncn5120)
    {
        _state = RxState::Control; // 2. Byte folgt noch
        return;
    }

    if ((value & U_CONFIGURE_MASK) == U_CONFIGURE_IND) _dll.configureIndication(value);
    if (value == U_RESET_IND) _dll.resetIndication();

    // Hier im Tick freigeben: _txState hat nur einen Schreiber.
    if ((value & L_DATA_CON_MASK) == L_DATA_CON) _dll._transmitter.confirmed();

    completeSequence(0, RxState::Idle);
}

// Einzige Stelle, an der die Sequenzfelder zurückgesetzt werden.
void Receiver::resetSequence(RxState nextState)
{
    if (nextState == RxState::Resync) _dll._statistics.incrementRxResyncs();

    _bufferPos = 0;
    _frameSize = 0;
    _crc = 0;
    _flags = 0;
    _state = nextState;
}

// Bricht eine laufende Sequenz ab. In Idle gibt es nichts abzubrechen - die folgenden Bytes (etwa die
// Reset-Antwort) sind gültig und dürfen nicht im Resync landen.
void Receiver::forceResync()
{
    if (_state == RxState::Idle || _state == RxState::Resync) return;

    _dll._statistics.incrementRxDroppedBytes((uint32_t)_bufferPos);

    resetSequence(RxState::Resync);
}

// Schiebt die fertige Sequenz in den Ringpuffer und gibt den Empfangspuffer frei.
void Receiver::completeSequence(uint8_t flags, RxState nextState)
{
    flags |= _flags;

    size_t length = _bufferPos < TPUART_BUFFER_SIZE ? _bufferPos : TPUART_BUFFER_SIZE;

    if (length > 0)
    {
        bool isFrame = (_buffer[0] & L_DATA_MASK) == L_DATA_STANDARD_IND || (_buffer[0] & L_DATA_MASK) == L_DATA_EXTENDED_IND;

        if (isFrame && _dll._transmitter.isEcho(_buffer, _bufferPos)) flags |= TP_FRAME_FLAG_TX;

        if (isFrame)
        {
            if (flags & TP_FRAME_FLAG_INVALID)
                _dll._statistics.incrementRxInvalidFrames();
            else
                _dll._statistics.incrementRxFrames();
        }

        // Kein Platz im Ring: diese Bytes hat niemand gesehen.
        if (!pushEntry(_buffer, length, flags))
            _dll._statistics.incrementRxDroppedBytes((uint32_t)length);
    }

    resetSequence(nextState);
}

// Voll: der neue Eintrag wird verworfen, nicht der älteste überschrieben.
bool Receiver::pushEntry(const uint8_t *data, size_t length, uint8_t flags)
{
    uint32_t needed = TPUART_RX_QUEUE_HEADER_SIZE + length;
    uint32_t used = _queueHead - _queueTail;

    if (TPUART_RX_QUEUE_SIZE - used < needed)
    {
        _dll.reportRxQueueOverflow();
        return false;
    }

    _dll._statistics.updateRxQueuePeakBytes(used + needed);

    uint32_t head = _queueHead;
    _queue[head++ % TPUART_RX_QUEUE_SIZE] = (uint8_t)(length & 0xFF);
    _queue[head++ % TPUART_RX_QUEUE_SIZE] = (uint8_t)(length >> 8);
    _queue[head++ % TPUART_RX_QUEUE_SIZE] = flags;

    for (size_t i = 0; i < length; i++)
        _queue[head++ % TPUART_RX_QUEUE_SIZE] = data[i];

    // Erst zuletzt sichtbar machen - sonst sähe loop() einen halben Eintrag.
    _queueHead = head;
    return true;
}

// ---------------------------------------------------------------------------------------------------
// Gemütliche Seite - läuft aus dem Hauptloop
// ---------------------------------------------------------------------------------------------------

void Receiver::processQueue()
{
    while (_queueTail != _queueHead)
    {
        uint32_t tail = _queueTail;

        size_t length = _queue[tail++ % TPUART_RX_QUEUE_SIZE];
        length |= (size_t)_queue[tail++ % TPUART_RX_QUEUE_SIZE] << 8;
        uint8_t flags = _queue[tail++ % TPUART_RX_QUEUE_SIZE];

        // Absicherung: eine korrupte Länge schriebe über das Frame auf dem Stack hinaus.
        if (length > TPUART_BUFFER_SIZE)
        {
            _dll.printError("RX queue corrupt: entry length %u - queue dropped", (unsigned)length);

            _queueTail = _queueHead;
            return;
        }

        // Kopie auf den Stack; einzeln, weil der Eintrag im Ring umbrechen kann.
        Frame frame(length, flags);
        uint8_t *data = frame.buffer();

        for (size_t i = 0; i < length; i++)
            data[i] = _queue[tail++ % TPUART_RX_QUEUE_SIZE];

        // Platz vor dem Callback freigeben - wie lange der Verbraucher braucht, geht den Tick nichts an.
        _queueTail = tail;

        if (!frame.isFrame())
        {
            _dll.handleControlEntry((const uint8_t *)frame.data(), length);
            continue;
        }

        // Wiederholungserkennung nur für gültige Telegramme - ein kaputter Inhalt verdürbe den Filtereintrag.
        if (frame.isValid())
        {
            bool seen = _dll._repetitionFilter.check(frame);

            if (seen && frame.isRepeated())
            {
                frame.setFiltered();
                _dll._statistics.incrementRxRepeatedFrames();
            }
        }

        _dll.deliverFrame(frame);
    }
}

RxState Receiver::state() const
{
    return _state;
}

// --- KOMPAT, siehe Header ------------------------------------------------------------------------------

unsigned short Receiver::getSearchBufferPosition() const
{
    return (unsigned short)_bufferPos;
}

// 0, solange die Größe noch nicht feststeht.
unsigned short Receiver::getAwaitBytes() const
{
    if (_frameSize == 0 || _bufferPos >= _frameSize) return 0;

    return (unsigned short)(_frameSize - _bufferPos);
}

} // namespace TPUart
