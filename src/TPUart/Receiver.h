#pragma once
#include <stddef.h>
#include <stdint.h>

#include "TPUart/Frame.h"
#include "TPUart/Types.h"

namespace TPUart
{

class DataLinkLayer;

// Stille, ab der ein Telegramm als beendet gilt. 2600µs: ab ">= 2.6 ms silence" garantiert NCN5130 das
// Frame-Ende, Siemens nennt 2-2,5ms. Nach oben begrenzt das früheste nächste Telegramm (~6,36ms am Host).
// Wirksam wird die Pause erst zwischen Frist und Frist plus zwei Tickintervallen.
#ifndef TPUART_FRAME_WAIT_US
#define TPUART_FRAME_WAIT_US 2600
#endif

// Frist für die Antwort nach einem vollständigen Telegramm (Quittung im Busmonitor bzw. L_Data.con). Die
// Antwort trifft erst ~2,7ms nach dem letzten Byte ein - also nach TPUART_FRAME_WAIT_US, daher eine eigene
// Frist unterhalb des nächsten möglichen Telegramms.
#ifndef TPUART_FRAME_ACK_US
#define TPUART_FRAME_ACK_US 4000
#endif

// RX-Warteschlange zwischen tick() und loop(): 1024 Byte, gut 80 Standard-Telegramme.
#ifndef TPUART_RX_QUEUE_EXP
#define TPUART_RX_QUEUE_EXP 10
#endif
constexpr uint32_t TPUART_RX_QUEUE_SIZE = (1u << TPUART_RX_QUEUE_EXP);

// Kopf eines Eintrags im Ringpuffer: Länge (2 Byte) + Flags (1 Byte), danach die Rohdaten.
constexpr uint32_t TPUART_RX_QUEUE_HEADER_SIZE = 3;

// Die Empfangshälfte. Zwei Kontexte, getrennt durch den Ringpuffer:
//   process()      - aus tick(): höchstens ein Byte, blockiert nie, alloziert nie, gibt nichts nach außen
//   processQueue() - aus loop(): leert den Ringpuffer, Wiederholungsfilter, Callbacks
//
// Drei Puffer mit verschiedenen Besitzern: _buffer (Tick, laufende Sequenz), _queue (Tick -> Loop) und das
// Frame auf dem Stack von processQueue(). Die Acknowledge-Entscheidung fällt hier, das Byte schreibt der
// Transmitter; die Callbacks liegen im DataLinkLayer.
class Receiver
{
  private:
    DataLinkLayer &_dll;

    // Die laufende Sequenz. _bufferPos: was eingelaufen ist; _frameSize: was laut Kopf kommen müsste.
    uint8_t _buffer[TPUART_BUFFER_SIZE];
    size_t _bufferPos = 0;

    volatile RxState _state = RxState::Idle;

    // 0 = Kopf noch nicht ausgewertet - daran hängt, dass der Acknowledge-Zweig je Frame genau einmal läuft.
    size_t _frameSize = 0;
    uint8_t _crc = 0;

    // Ein U_IntRegRd.req ist raus: das nächste Byte ist der Registerwert, ohne eigene Kennung.
    bool _awaitRegisterValue = false;

    // Flags der laufenden Sequenz, in der Form des Ringeintrags. ADDRESSED und ACK sind zwei Aussagen:
    // ADDRESSED entscheidet der Callback (auch wenn die Quittung unterdrückt wurde), ACK heißt "Quittung liegt vor".
    uint8_t _flags = 0;

    // Seit wann das Interface nichts hergibt. Gemessen ab der ersten Beobachtung "nichts da", damit ein
    // verzögerter Tick nicht als Pause gilt. _emptySince gilt nur, wenn _emptyStarted gesetzt ist.
    uint32_t _emptySince = 0;
    bool _emptyStarted = false;

    // SPSC-Ringpuffer: _queueHead schreibt nur tick() (erst nach dem ganzen Eintrag), _queueTail nur loop().
    uint8_t _queue[TPUART_RX_QUEUE_SIZE];
    volatile uint32_t _queueHead = 0;
    volatile uint32_t _queueTail = 0;


    void processByte(uint8_t value);
    void processFrameByte(uint8_t value);
    void processPollByte(uint8_t value);
    void processControlByte(uint8_t value);

    // Entscheidet über die Quittung zum laufenden Frame und lässt sie vom Transmitter absetzen.
    void sendAcknowledge();

    void checkPause();
    void handleVerifiedPause();

    // Setzt die Empfangsmaschine auf den Anfang einer neuen Sequenz und geht in nextState über.
    void resetSequence(RxState nextState);

    // Stellt die aktuell im _buffer liegende Sequenz in den Ringpuffer und geht in nextState über.
    void completeSequence(uint8_t flags, RxState nextState);
    bool pushEntry(const uint8_t *data, size_t length, uint8_t flags);

    // Angetrieben nur vom DataLinkLayer - jede dieser Methoden gehört in genau einen Kontext.
    friend class DataLinkLayer;

    // Aus tick(): verarbeitet höchstens EIN Byte.
    void process();

    // Aus loop(): leert den Ringpuffer.
    void processQueue();

    // Bricht eine laufende Sequenz ab (Busmonitor an, Reset) und geht in den Resync.
    void forceResync();

  public:
    explicit Receiver(DataLinkLayer &dll);

    RxState state() const;

    // KOMPAT: Diagnosewerte der alten Library, sinngemäß auf den heutigen Empfangspfad abgebildet.
    unsigned short getSearchBufferPosition() const; // Bytes der laufenden Sequenz im Puffer
    unsigned short getAwaitBytes() const;           // noch ausstehende Bytes des laufenden Telegramms
};

} // namespace TPUart
