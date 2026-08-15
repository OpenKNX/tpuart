#include "TPUart/Transmitter.h"

#include <stdlib.h>
#include <string.h>

#include <Arduino.h>

#include "TPUart/DataLinkLayer.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Statistics.h"

namespace TPUart
{

static_assert(TPUART_TX_QUEUE_COUNT >= 1, "TPUART_TX_QUEUE_COUNT muss mindestens einen Platz haben");


Transmitter::Transmitter(DataLinkLayer &dll) : _dll(dll) {}

// Gibt her, was noch in der Warteschlange liegt. Im Regelfall läuft das nie - der DataLinkLayer ist
// üblicherweise ein globales Objekt - aber ein Heap-Halter ohne Destruktor ist eine Falle für jeden, der ihn
// doch einmal lokal anlegt.
Transmitter::~Transmitter()
{
    for (uint32_t i = _queueFree; i != _queueHead; i++)
        free(_queue[i % TPUART_TX_QUEUE_COUNT]._data);
}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft später aus einem Timer-Interrupt
// ---------------------------------------------------------------------------------------------------

// Der einzige Schreibzugriff auf die Schnittstelle in dieser Klasse - siehe Header. Zählt mit, damit
// Statistics::getTxBytes() die Auslastung der Hostleitung wirklich vollständig abbildet.
bool Transmitter::writeByte(uint8_t value)
{
    if (!_dll._interface->write((char)value)) return false;

    _dll._statistics.incrementTxBytes();
    return true;
}

// Sendet eine Steuersequenz oder ein Telegramm. Steuercodes haben Vorrang: sie sind kurz, zeitnah
// erwartet und dürfen sich laut Protokoll zwischen zwei Telegrammbytes schieben (jedes Telegrammbyte
// trägt sein eigenes Positionsbyte). Pro Tick geht höchstens eines von beidem raus.
void Transmitter::process()
{
    // Liegt ein Steuercode an, gehört ihm dieser Tick - auch wenn er gerade noch nicht abgesetzt werden
    // konnte. Der Telegrammpfad muss dann warten, sonst belegt er den Platz, auf den der Steuercode wartet.
    if (processCtrlQueue()) return;

    // Das Telegramm ist raus, es fehlt die Bestätigung der BCU. Sie kommt als L_Data.con und wird im
    // Empfangspfad ausgewertet (Receiver::processControlByte) - hier bleibt nur die Notbremse, damit ein
    // ausbleibendes L_Data.con den Sendeweg nicht für immer belegt.
    if (_state == TxState::Await)
    {
        if ((uint32_t)(millis() - _awaitSince) < TPUART_TX_CONFIRM_TIMEOUT_MS) return;

        // Keine Bestätigung. Was im Sendepuffer der BCU noch liegt, ist damit unbekannt - der einzige Weg
        // zu einem definierten Zustand ist der Reset. Der Zustand bleibt vorerst Await; wieder aufgenommen
        // wird der Versand von der U_Reset.ind aus, und die kommt hierauf genauso wie auf jeden anderen
        // Reset. Bleibt auch sie aus, läuft diese Frist erneut ab und der Reset geht noch einmal raus -
        // gegen eine stumme BCU gibt es nichts Besseres als weiter zu versuchen.
        if (_dll._interface->availableForWrite() < 1) return;
        if (!writeByte(U_RESET_REQ)) return;

        _dll._statistics.incrementTxControlBytes();
        _awaitSince = millis();
        _confirmTimeout = true; // gemeldet wird das aus loop(), hier darf nichts nach außen

        // Wie bei jedem abgesetzten Steuercode: der DataLinkLayer leitet daraus den Chip-Zustand ab (der
        // Reset beendet den Busmonitor und macht eine laufende Empfangssequenz unlesbar).
        _dll.controlByteSent(U_RESET_REQ);
        return;
    }

    // Sendeweg frei? Dann das nächste Telegramm aus der Warteschlange holen. Ist keines da, gibt es nichts
    // zu tun - das ist der Normalfall und der häufigste Pfad hier.
    if (_state == TxState::Idle && !startNextTransmission()) return;

    bool last = (_bufferPos == _frameSize - 1); // das letzte Byte ist die Prüfsumme
    uint8_t offset = (uint8_t)(_bufferPos >> 6);

    // Ob das Offset-Byte mit muss, entscheidet sich VOR der Platzprüfung - und das ist der Unterschied
    // zwischen 3 und 2 verlangten Bytes. Gebraucht wird es bei einem 263-Oktett-Telegramm fünfmal, bei
    // allen übrigen 258 Positionen nicht.
    //
    // WARUM DAS MESSBAR IST: der Ring unter uns fasst vier Bytes und leert sich mit 286µs je Byte. Wer
    // pauschal drei freie Bytes verlangt, darf nur schreiben, solange höchstens EINES belegt ist - und
    // muss einen Tick aussetzen, sobald zwei drinstehen. Bei 500µs Takt kostete das gemessene 19ms je
    // Telegramm (143ms statt 124ms); auf Kern 1 mit 84000 Ticks/s fiel es nicht auf, weil dort der
    // nächste Versuch sofort kam. Verlangt wird jetzt, was wirklich gebraucht wird.
    //
    // An der Unteilbarkeit ändert das nichts: die zwei bzw. drei Bytes gehören weiterhin zusammen und
    // werden erst geschrieben, wenn sie vollständig passen.
    bool needsOffset = _dll.bcuType() == BcuType::Ncn5120 && (!_chipOffsetValid || offset != _chipOffset);
    size_t needed = needsOffset ? 3 : 2;

    if (_dll._interface->availableForWrite() < needed) return;

    // NUR beim NCN512x, siehe needsOffset oben. Der TPUART2 kennt den Dienst nicht - seine Servicetabelle
    // (docs/Siemens_TPUART.pdf, S. 21) geht von U_L_DataContinue (Index 1...62) direkt zu U_L_DataEnd (Länge 7...63),
    // der Opcode 0x08 ist dort nicht vergeben und wäre ein unbekanntes Steuerbyte. Gebraucht wird er dort
    // auch nie: mehr als 64 Oktette kann der Chip ohnehin nicht senden, der Index bleibt also unter 64.
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

// Aus dem Tick. Kopiert das nächste Telegramm in den Sendepuffer. Der Heap-Block wird hier NICHT
// freigegeben - free() gehört nicht in einen Interrupt; das erledigt releaseSentTelegrams() aus loop().
bool Transmitter::startNextTransmission()
{
    // Im Busmonitor ist der Chip transparent und sendet nichts - dort darf nichts angefangen werden.
    // sendFrame() lehnt in diesem Modus schon ab, aber ein Telegramm, das VOR dem Umschalten eingereiht
    // wurde, läge sonst hier und ginge trotzdem raus. Es bleibt in der Warteschlange und wartet, bis der
    // Busmonitor per Reset verlassen wird.
    if (_dll.isBusMonitor()) return false;

    if (_queueTail == _queueHead) return false;

    const Entry &entry = _queue[_queueTail % TPUART_TX_QUEUE_COUNT];

    // Kann im geordneten Betrieb nicht vorkommen (sendFrame() prüft beim Einstellen). Bleibt stehen, weil
    // ein Nullzeiger oder eine überlange Länge hier sonst über den Sendepuffer hinausschreiben würde.
    if (entry._data == nullptr || entry._length == 0 || entry._length > TPUART_BUFFER_SIZE)
    {
        _queueTail = _queueTail + 1;
        return false;
    }

    _frameSize = entry._length;
    for (size_t i = 0; i < _frameSize; i++)
        _buffer[i] = entry._data[i];

    _queueTail = _queueTail + 1; // ab hier darf der Hauptkontext den Block freigeben

    beginTransmission();
    return true;
}

// Der Anfang einer Übertragung, für beide Fälle: ein frisch geholtes Telegramm und eines, das nach einem
// Reset von vorn beginnt. Identisch, weil der Sendepuffer in beiden Fällen derselbe ist - nur der Fortschritt
// darin geht auf null, und der Offset im Chip gilt als unbekannt.
void Transmitter::beginTransmission()
{
    _bufferPos = 0;
    _chipOffsetValid = false;
    _state = TxState::Transmit;
}

// Setzt höchstens eine vollständige Steuersequenz ab. Die Gruppe geht immer ungeteilt raus - deshalb wird
// vorab geprüft, ob das Interface sie komplett annehmen kann. Ein Zerteilen würde die BCU die Reste als
// eigenständigen Befehl deuten lassen.
//
// Der Rückgabewert heißt "der Sendeweg gehört in diesem Tick den Steuercodes" - true also auch dann, wenn
// die Gruppe noch nicht passt. Sonst wäre der Vorrang nur die halbe Miete: der Telegrammpfad greift sich
// drei Bytes, sobald drei frei sind, und im 4-Byte-Ring der RP2040 würden dann nie vier Bytes am Stück
// frei. Eine viergliedrige Gruppe (U_SetAddress.req, U_SetRepetition.req) käme während eines laufenden
// Telegramms überhaupt nicht mehr durch. Lässt der Telegrammpfad stattdessen aus, läuft der Ring in den
// nächsten Ticks leer und die Gruppe geht raus - Vorrang bedeutet also auch, Platz zu machen.
bool Transmitter::processCtrlQueue()
{
    if (_ctrlQueueTail == _ctrlQueueHead) return false;

    uint32_t tail = _ctrlQueueTail;
    size_t length = _ctrlQueue[tail % TPUART_CTRL_QUEUE_SIZE];

    // Kann im geordneten Betrieb nicht vorkommen (queueControl() prüft beim Einstellen). Bleibt stehen,
    // damit ein einzelner korrupter Eintrag nicht in eine Endlosschleife über Datenmüll führt.
    if (length == 0 || length > TPUART_CTRL_MAX_GROUP)
    {
        // Gemeldet wird das als Verlust von Steuercodes - was es auch ist, wenn auch aus anderem Grund als
        // ein voller Ring. Genauer geht es von hier aus nicht: der Tick darf nichts ausgeben.
        _ctrlQueueTail = _ctrlQueueHead;
        _dll.reportControlOverflow();
        return false;
    }

    // Noch kein Platz für die ganze Gruppe: nichts halb absetzen, aber den Sendeweg für diesen Tick auch
    // nicht freigeben (siehe oben).
    if (_dll._interface->availableForWrite() < length) return true;

    tail++;
    uint8_t code = _ctrlQueue[tail % TPUART_CTRL_QUEUE_SIZE];

    for (size_t i = 0; i < length; i++)
        writeByte(_ctrlQueue[tail++ % TPUART_CTRL_QUEUE_SIZE]);

    _ctrlQueueTail = tail;
    _dll._statistics.incrementTxControlBytes((uint32_t)length);

    // Erst jetzt gilt der Chip als umgeschaltet - vorher hätten wir einen Zustand angenommen, in dem er
    // noch gar nicht war. Abgeleitet aus dem tatsächlich gesendeten Code statt aus einem separaten
    // Wunsch-Feld: ein solches Feld wäre Nutzlast und müsste vor der Veröffentlichung geschrieben werden -
    // genau diese Reihenfolge war einmal falsch, mit dem Ergebnis, dass Chip-Zustand und Busmonitor-Flag
    // dauerhaft auseinanderliefen.
    _dll.controlByteSent(code);

    return true;
}

// Aus dem Empfangspfad und damit aus dem Tick. Entschieden hat der Receiver, hier geht das Byte nur noch
// raus - der TxState bleibt unberührt, ein Acknowledge ist kein Telegrammversand.
bool Transmitter::sendAcknowledge(AckType acknowledge)
{
    if (_dll._interface->availableForWrite() < 1) return false; // kein Platz - lieber nicht acken als blockieren

    if (!writeByte((uint8_t)(U_ACKN_REQ | (uint8_t)acknowledge))) return false;

    _dll._statistics.incrementAcknowledgesSent();
    return true;
}

// Nur im Wartezustand von Belang: während der Übergabe läuft die Frist ohnehin noch nicht, und wenn sie
// nicht läuft, gibt es nichts aufzuschieben.
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

// Verglichen wird das VOLLSTÄNDIGE Telegramm - eine Prefix-Fassung für den halb eingelaufenen Fall gab es
// einmal, sie wurde aber nur von der Acknowledge-Entscheidung gebraucht, und die kommt ohne aus (siehe
// Receiver::sendAcknowledge).
//
// Zwei Bytes bleiben beim Vergleich außen vor, und beide aus demselben Grund: die BCU löscht beim
// Wiederholen das Wiederholungs-Bit im Kontrollbyte. Das wird deshalb maskiert verglichen, und die
// Prüfsumme am Ende gar nicht - sie hängt an eben diesem Byte.
bool Transmitter::isEcho(const uint8_t *data, size_t length) const
{
    if (_state == TxState::Idle) return false;
    if (length != _frameSize) return false;
    if (((data[0] ^ _buffer[0]) & (uint8_t)~0x20) != 0) return false;

    return memcmp(data + 1, _buffer + 1, length - 2) == 0;
}

// ---------------------------------------------------------------------------------------------------
// Gemütliche Seite - läuft aus dem Hauptloop
// ---------------------------------------------------------------------------------------------------

// Ein Produzent, ein Konsument (tick()), Kopf zuletzt sichtbar gemacht - dieselbe Regel wie bei den anderen
// beiden Ringen, damit der Tick nie einen halben Eintrag sieht.
bool Transmitter::sendFrame(const uint8_t *data, size_t length)
{
    // Jede Ablehnung nennt ihren Grund selbst. Der Aufrufer bekommt nur ein bool und kann ihn deshalb
    // nicht kennen - eine geratene Begründung in der Anwendung ist schlimmer als gar keine.
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

    // Ein Telegramm hat mindestens 7 Nutzoktette (Standard: Control, Quelle, Ziel, Länge, TPCI), und mit
    // der angehängten Prüfsumme muss es in den Puffer passen.
    if (length < 7 || length + 1 > TPUART_BUFFER_SIZE)
    {
        _dll.printError("Send rejected: length %u out of range (7..%u)", (unsigned)length, (unsigned)(TPUART_BUFFER_SIZE - 1));
        return false;
    }

    // Der TPUART2 kann nur 63 Nutzoktette plus Prüfsumme - dieselbe Grenze wie in der alten Library, und
    // sie steckt in seiner Servicetabelle (docs/Siemens_TPUART.pdf, S. 21): U_L_DataContinue geht bis 0xBE, also
    // Index 62, und U_L_DataEnd bis 0x7F, also Länge 63. Damit sind die Datenoktette 0...62 (das sind 63)
    // und die Prüfsumme auf Index 63 - zusammen 64 Oktette. Beim NCN512x verschiebt U_L_DataOffset.req
    // dieses Fenster, dort sind es bis zu 263.
    if (_dll.bcuType() == BcuType::Tpuart2 && length + 1 > 64)
    {
        _dll.printError("Send rejected: TPUART2 takes at most 63 byte plus checksum");
        return false;
    }

    // Abgeholte, aber noch nicht freigegebene Plätze zuerst einsammeln - sonst gälte die Warteschlange als
    // voll, obwohl der Tick längst weitergekommen ist.
    releaseSentTelegrams();

    if (_queueHead - _queueFree >= TPUART_TX_QUEUE_COUNT)
    {
        _dll._statistics.incrementTxQueueOverflows();
        _dll.printError("Send rejected: queue full (%u telegrams)", (unsigned)TPUART_TX_QUEUE_COUNT);
        return false;
    }

    // malloc statt new: kein Exception-Handling nötig, der Fehlerfall ist ein sauberes nullptr. Das ist die
    // einzige Heap-Nutzung der Library, und sie liegt bewusst hier - Telegramme sind unterschiedlich groß,
    // eine Anzahl-Grenze wie in der alten Library wäre statisch nur mit vielfachem Speicher zu haben.
    uint8_t *buffer = (uint8_t *)malloc(length + 1);
    if (buffer == nullptr)
    {
        _dll._statistics.incrementTxQueueOverflows();
        _dll.printError("Send rejected: out of memory (%u byte)", (unsigned)(length + 1));
        return false;
    }

    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++)
    {
        buffer[i] = data[i];
        crc ^= data[i];
    }

    buffer[length] = (uint8_t)~crc; // dieselbe CRC-8/GSM-A wie beim Empfang

    Entry &entry = _queue[_queueHead % TPUART_TX_QUEUE_COUNT];
    entry._data = buffer;
    entry._length = (uint16_t)(length + 1);

    _queueHead = _queueHead + 1; // erst jetzt ist der Eintrag für den Tick sichtbar
    return true;
}

// Aus dem Hauptkontext. Gibt frei, was der Tick inzwischen abgeholt hat. Erst damit werden die Plätze
// wieder verfügbar - deshalb rechnet die Belegungsprüfung in sendFrame() gegen _queueFree.
void Transmitter::releaseSentTelegrams()
{
    while (_queueFree != _queueTail)
    {
        Entry &entry = _queue[_queueFree % TPUART_TX_QUEUE_COUNT];

        free(entry._data);
        entry._data = nullptr;
        entry._length = 0;

        _queueFree++;
    }
}

bool Transmitter::queueControl(uint8_t code)
{
    return queueControl(&code, 1);
}

// Stellt eine Steuersequenz in die Warteschlange. Anders als früher wird NICHT abgelehnt, solange gerade
// gesendet wird: Steuercodes sind kein Telegrammversand und müssen auch dann durchkommen. Genau deshalb
// haben sie eine eigene Warteschlange und belegen keinen TxState mehr.
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

    // Ein Produzent (dieser Kontext), ein Konsument (tick()). Der Kopf wird als ALLERLETZTES weitergesetzt,
    // damit der Tick nie eine halb geschriebene Sequenz sieht. Keine Sperre nötig, keine Besitzübergabe.
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

uint32_t Transmitter::queueUsed() const
{
    return _queueHead - _queueFree;
}

uint32_t Transmitter::queueSize() const
{
    return TPUART_TX_QUEUE_COUNT;
}

bool Transmitter::confirmTimeout()
{
    if (!_confirmTimeout) return false;

    _confirmTimeout = false;
    return true;
}

} // namespace TPUart
