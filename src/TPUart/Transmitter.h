#pragma once
#include "TPUart/TransmitQueue.h"

#include <stddef.h>
#include <stdint.h>

#include "TPUart/Types.h"

namespace TPUart
{

class DataLinkLayer;

// Schlimmster Fall eines Telegrammbytes: Offset-, Positions- und Datenbyte, unmittelbar hintereinander.
// Nur Obergrenze für TPUART_TX_INTERFACE_BUFFER - process() verlangt, was wirklich gebraucht wird.
constexpr size_t TPUART_TX_ATOMIC_BYTES = 3;

// Steuercode-Warteschlange zwischen Hauptkontext und tick(). Steuercodes gehen auch während eines
// laufenden Telegramms raus.
#ifndef TPUART_CTRL_QUEUE_EXP
#define TPUART_CTRL_QUEUE_EXP 5 // 32 Byte - reicht für mehrere Sequenzen
#endif
constexpr uint32_t TPUART_CTRL_QUEUE_SIZE = (1u << TPUART_CTRL_QUEUE_EXP);

// Längste Steuersequenz (Table 12): 4 Byte. Sie muss ununterbrochen rausgehen.
constexpr size_t TPUART_CTRL_MAX_GROUP = 4;

// Sendepuffer, den ein Interface bieten muss - nicht weniger und nicht mehr:
//   mindestens so viel, dass die längste Gruppe am Stück passt (sonst bleibt sie für immer stehen);
//   höchstens so viel, dass ein U_Ackn.req noch binnen 2,8ms beim Chip ist (4 Byte ~ 2,3ms bei 19200).
// Vier passt auch für den dichtesten Tick: 1 Byte Quittung plus 3 Bytes Offset-Gruppe.
constexpr size_t TPUART_TX_INTERFACE_BUFFER = TPUART_CTRL_MAX_GROUP > TPUART_TX_ATOMIC_BYTES ? TPUART_CTRL_MAX_GROUP : TPUART_TX_ATOMIC_BYTES;

// Wachhund, falls das L_Data.con ausbleibt: danach U_Reset.req, die U_Reset.ind startet das Telegramm neu.
// Gemessen wird fehlender Fortschritt - jedes Echo setzt die Frist neu (auch bei Wiederholungen).
#ifndef TPUART_TX_CONFIRM_TIMEOUT_MS
#define TPUART_TX_CONFIRM_TIMEOUT_MS 10000
#endif

// Die Sendehälfte - aller schreibende Zugriff auf das Interface:
//   Steuercodes  - eigene Warteschlange, Vorrang, dürfen sich zwischen Telegrammbytes schieben
//   Acknowledge  - im Receiver entschieden, hier geschrieben
//   Telegramme   - ein Oktett je Tick, einziger Zustand (TxState)
class Transmitter
{
  private:
    DataLinkLayer &_dll;

    // Die Sendewarteschlange gehört dem Hauptkontext allein; der Tick bekommt ein Telegramm vorgelegt.
    TransmitQueue _queue;

    // Vorlage für den Tick, zeigt in den Puffer der Warteschlange. Regeln:
    //   1. Der Hauptkontext schreibt die Vorlage nur, wenn _stagedSeq == _takenSeq.
    //   2. Er gibt den Platz erst frei, wenn _takenSeq nachgezogen hat.
    //   3. Zähler zuletzt veröffentlichen - Nutzlast vor Zähler.
    const uint8_t *_stagedBuffer = nullptr;
    size_t _stagedFrameSize = 0;
    volatile uint32_t _stagedSeq = 0; // schreibt nur der Hauptkontext
    volatile uint32_t _takenSeq = 0;  // schreibt nur tick()

    // Das gerade übertragene Telegramm, nur Tick. Bleibt bis zum Echo-Vergleich unangetastet.
    uint8_t _buffer[TPUART_BUFFER_SIZE];
    size_t _frameSize = 0; // inklusive Prüfsumme, genau wie sie im Puffer steht
    size_t _bufferPos = 0; // nächstes zu sendendes Oktett des laufenden Telegramms

    // Der zuletzt an den Chip gesendete Offset - er geht nur bei Änderung raus.
    uint8_t _chipOffset = 0;
    bool _chipOffsetValid = false; // nach einem Neubeginn ist unbekannt, was im Chip steht
    uint32_t _awaitSince = 0; // seit wann auf L_Data.con gewartet wird

    volatile TxState _state = TxState::Idle;

    // Steuercode-Warteschlange, Eintrag [len][bytes...]. SPSC: Kopf erst nach vollständigem Schreiben weiter.
    uint8_t _ctrlQueue[TPUART_CTRL_QUEUE_SIZE];
    volatile uint32_t _ctrlQueueHead = 0; // schreibt nur der Hauptkontext
    volatile uint32_t _ctrlQueueTail = 0; // schreibt nur tick()

    // Siehe confirmTimeout(). Der Tick setzt, der Hauptkontext liest und löscht.
    volatile bool _confirmTimeout = false;


    // Holt das vorgelegte Telegramm in den Sendepuffer. true, wenn eines zum Senden bereitsteht.
    bool startNextTransmission();

    // Setzt den Fortschritt im Sendepuffer auf Anfang (neues Telegramm oder Neubeginn nach Reset).
    void beginTransmission();

    // Setzt höchstens eine Steuersequenz ab. true: der Sendeweg gehört in diesem Tick den Steuercodes.
    bool processCtrlQueue();

    // Einziger Schreibzugriff auf das Interface - sonst stimmt getTxBytes() nicht.
    bool writeByte(uint8_t value);

    // Angetrieben vom DataLinkLayer, der Receiver braucht Quittung und Echo-Vergleich. queueControl() bleibt
    // privat: beliebige Steuerbytes verstellten die abgeleiteten Zustände.
    friend class DataLinkLayer;
    friend class Receiver;

    // Aus tick(): höchstens eine Steuersequenz ODER ein Telegramm-Oktett.
    void process();

    // Aus dem Hauptkontext: reiht ein vollständiges Telegramm samt Prüfsumme ein (kopiert).
    bool pushTransmitQueue(const Frame &frame);

    // Aus dem Hauptkontext: reiht eine Steuersequenz ein, die ununterbrochen abgesetzt wird.
    bool queueControl(uint8_t code);
    bool queueControl(const uint8_t *codes, size_t length);

    // Aus dem Tick (Receiver): schreibt das U_Ackn.req. true, wenn geschrieben.
    bool sendAcknowledge(AckType acknowledge);

    // Aus dem Tick: ein vollständiges Echo ist da - setzt die Frist des Wachhunds neu.
    void echoReceived();

    // L_Data.con ist da - der Sendeweg ist frei, egal wie es ausfiel.
    void confirmed();

    // Ein Reset hat den Sendepuffer der BCU geräumt: das laufende Telegramm beginnt von vorn.
    void restart();

    // Bricht die laufende Übertragung ab (Wechsel in den Busmonitor). Nur aus dem Tick.
    void abort();

    // Ist das vollständige Telegramm das Echo unseres eigenen?
    bool isEcho(const uint8_t *data, size_t length) const;

    // Dasselbe für den Anfang eines Telegramms - gebraucht bei der Quittungsentscheidung.
    bool isEchoPrefix(const uint8_t *data, size_t length) const;

    // Gibt den abgeholten Platz frei, räumt im Busmonitor und legt das nächste Telegramm vor. Nur Hauptkontext.
    void stageNextTelegram();

    // Der Wachhund hat zugeschlagen - gesetzt im Tick, gemeldet aus loop().
    bool confirmTimeout();

  public:
    explicit Transmitter(DataLinkLayer &dll);
    ~Transmitter();

    TxState state() const;
    bool isTransmitting() const;

    // Belegte Bytes der Sendewarteschlange, einschließlich eines vorgelegten Telegramms.
    uint32_t queueUsed() const;
    uint32_t queueSize() const;
};

} // namespace TPUart
