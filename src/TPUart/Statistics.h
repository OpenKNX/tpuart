#pragma once
#include <stdint.h>

namespace TPUart
{

// Zähler für Diagnose. Namensschema: Richtung immer als Präfix (getRx…/getTx…), Einheit im Namen.
//
// Kaputte Telegramme (gemeldet, mit INVALID) und verworfene Bytes (nie gemeldet) sind verschiedene Dinge.
//
// ACHTUNG: die Byte-Kategorien sind keine Zerlegung von getRxBytes() - sie überschneiden sich (ein
// verworfenes Telegramm zählt auch als Telegrammbyte) und lassen Bytes aus (L_Data.con in FrameAck).
// Gesendete Telegrammbytes: getTxBytes() - getTxControlBytes() - getTxAcknowledges().
//
// Geschrieben aus dem Tick, gelesen aus dem Hauptkontext: volatile, je ein Schreiber. Ausnahmen sind an
// der Deklaration vermerkt.
class Statistics
{
  private:
    volatile uint32_t _rxBytes = 0;
    volatile uint32_t _rxFrames = 0;
    volatile uint32_t _rxInvalidFrames = 0;
    volatile uint32_t _rxFrameBytes = 0;
    volatile uint32_t _rxControlBytes = 0;
    volatile uint32_t _rxDroppedBytes = 0;
    volatile uint32_t _rxRepeatedFrames = 0;

    volatile uint32_t _txControlBytes = 0;
    volatile uint32_t _txFrames = 0;

    // Jedes an die Schnittstelle übergebene Byte (Transmitter::writeByte()).
    volatile uint32_t _txBytes = 0;
    volatile uint32_t _txAcknowledges = 0;
    volatile uint32_t _txAcknowledgesSuppressed = 0;

    volatile uint32_t _rxInterfaceOverflows = 0;
    volatile uint32_t _rxQueueOverflows = 0;
    volatile uint32_t _txControlQueueOverflows = 0;
    volatile uint32_t _txQueueOverflows = 0;

    // Wachhund: gar kein L_Data.con, BCU zurückgesetzt. Zeigt auf Chip oder Verkabelung, nicht auf den Bus.
    volatile uint32_t _txConfirmTimeouts = 0;

    // Aus dem HAUPTKONTEXT (processConnectionState()), nicht aus dem Tick - ein Schreiber bleibt es trotzdem.
    volatile uint32_t _connectionLosses = 0;

    // Wie oft die Position im Bytestrom verloren ging - eine andere Frage als getRxDroppedBytes().
    volatile uint32_t _rxResyncs = 0;

    // Fehlerbits aus U_State.ind, einzeln gezählt. Kein Rx/Tx-Präfix: Zustände des Chips.
    volatile uint32_t _chipSlaveCollisions = 0;
    volatile uint32_t _chipReceiveErrors = 0;
    volatile uint32_t _chipTransmitErrors = 0;
    volatile uint32_t _chipProtocolErrors = 0;
    volatile uint32_t _chipTemperatureWarnings = 0;

    // Anzahl der Ticks; gegen die Laufzeit gerechnet die mittlere Taktrate.
    volatile uint32_t _ticks = 0;

    // Wie oft ein Tickabstand die Busgrenze gerissen hat. Klassifiziert im DataLinkLayer.
    volatile uint32_t _tickDeferrals = 0;

    // Die zuletzt gemessene Verzögerung - der Höchstwert sättigte nach einem Flash-Schreibvorgang.
    volatile uint32_t _tickLastDeferredUs = 0;

    // Längste Laufzeit eines Ticks, einschließlich Quittungs-Callback - Grundlage der IRQ-Priorität.
    volatile uint32_t _tickDurationMaxUs = 0;

    // Anteil des Quittungs-Callbacks daran.
    volatile uint32_t _checkAcknowledgeMaxUs = 0;

    // Höchster Rückstand im Interface in Bytes; gesund sind 0-1.
    volatile uint32_t _rxInterfacePeakBytes = 0;

    // Höchststände der drei Warteschlangen, in Bytes.
    volatile uint32_t _rxQueuePeakBytes = 0;
    volatile uint32_t _txControlQueuePeakBytes = 0;
    volatile uint32_t _txQueuePeakBytes = 0;

    // Buslast: je Sekunde gemessen, das Ergebnis steht bis zur nächsten Messung (kein gleitender Mittelwert).
    static constexpr uint32_t BUS_LOAD_INTERVAL_MS = 1000;

    // Bezugspunkt der laufenden Messung: Zählerstände und Zeitstempel. Die Telegrammzahl gehört dazu, weil
    // Pause und Quittungsslot je Telegramm Busbelegung sind.
    uint32_t _busLoadRefBytes = 0;
    uint32_t _busLoadRefFrames = 0;
    uint32_t _busLoadRefAt = 0;
    bool _busLoadRefValid = false;

    // Ergebnis der zuletzt abgeschlossenen Sekunde, 0 bis zur ersten.
    uint32_t _busLoadBytesPerSecond = 0;
    uint16_t _busLoadPercent = 0;

  public:
    void reset();

    // --- Empfang ---------------------------------------------------------------------------------------

    void incrementRxBytes(uint32_t increment = 1);
    void incrementRxFrames(uint32_t increment = 1);
    void incrementRxInvalidFrames(uint32_t increment = 1);
    void incrementRxFrameBytes(uint32_t increment = 1);
    void incrementRxControlBytes(uint32_t increment = 1);
    void incrementRxDroppedBytes(uint32_t increment = 1);

    // Aus dem Hauptkontext (loop()), nicht aus dem Tick - der Wiederholungsfilter läuft dort.
    void incrementRxRepeatedFrames(uint32_t increment = 1);

    uint32_t getRxBytes() const;         // jedes vom Interface gelesene Byte
    uint32_t getRxFrames() const;        // gemeldet und in Ordnung
    uint32_t getRxInvalidFrames() const; // gemeldet, aber kaputt

    // Telegrammbytes vom Bus, Poll eingeschlossen - Grundlage der Buslast. Steuerbytes zählen nicht.
    uint32_t getRxFrameBytes() const;

    uint32_t getRxControlBytes() const;
    uint32_t getRxDroppedBytes() const; // nie gemeldet: Resync, Moduswechsel, voller RX-Ring

    // Als Wiederholung markiert (TP_FRAME_FLAG_FILTERED).
    uint32_t getRxRepeatedFrames() const;

    // --- Versand ---------------------------------------------------------------------------------------

    void incrementTxBytes(uint32_t increment = 1);
    void incrementTxControlBytes(uint32_t increment = 1);
    void incrementTxFrames(uint32_t increment = 1);
    void incrementTxAcknowledges(uint32_t increment = 1);
    void incrementTxAcknowledgesSuppressed(uint32_t increment = 1);
    void incrementTxConfirmTimeouts(uint32_t increment = 1);

    uint32_t getTxFrames() const;
    uint32_t getTxControlBytes() const;

    // Alles, was je an die Schnittstelle ging.
    uint32_t getTxBytes() const;
    uint32_t getTxAcknowledges() const;

    // Nicht quittiert, weil das Telegramm schon durch war. Steigt der Wert, kommt der Tick zu spät dran.
    uint32_t getTxAcknowledgesSuppressed() const;

    // Kein L_Data.con, der Wachhund hat die BCU zurückgesetzt.
    uint32_t getTxConfirmTimeouts() const;

    // --- Verluste und Verbindung -----------------------------------------------------------------------

    void incrementRxInterfaceOverflows(uint32_t increment = 1);
    void incrementRxQueueOverflows(uint32_t increment = 1);
    void incrementTxControlQueueOverflows(uint32_t increment = 1);

    // Aus dem Hauptkontext (pushTransmitQueue()).
    void incrementTxQueueOverflows(uint32_t increment = 1);

    // Aus dem Hauptkontext (processConnectionState()).
    void incrementConnectionLosses(uint32_t increment = 1);

    void incrementRxResyncs(uint32_t increment = 1);

    // Aus dem Hauptkontext (handleControlEntry()).
    void incrementChipSlaveCollisions(uint32_t increment = 1);
    void incrementChipReceiveErrors(uint32_t increment = 1);
    void incrementChipTransmitErrors(uint32_t increment = 1);
    void incrementChipProtocolErrors(uint32_t increment = 1);
    void incrementChipTemperatureWarnings(uint32_t increment = 1);

    // --- Takt ------------------------------------------------------------------------------------------

    // Zählt einen Tick.
    void recordTick();

    // Ein Tickabstand hat die Busgrenze gerissen.
    void recordTickDeferred(uint32_t deferredUs);

    void updateRxInterfacePeakBytes(uint32_t pending);

    // Wie oft tick() lief. Nur als Differenz zweier Stände benutzen - läuft nach ~25 Tagen um.
    uint32_t getTicks() const;



    // Wie oft der Tick aufgehalten wurde.
    uint32_t getTickDeferrals() const;

    // Dauer der zuletzt gemessenen Verzögerung, 0 wenn nie eine auftrat.
    uint32_t getTickLastDeferredUs() const;

    void updateTickDurationMaxUs(uint32_t durationUs);

    // Längste Laufzeit eines vollen Tick-Durchlaufs, einschließlich Quittungs-Callback.
    uint32_t getTickDurationMaxUs() const;

    // Längste Laufzeit des Quittungs-Callbacks - gegen getTickDurationMaxUs() zu lesen.
    void updateCheckAcknowledgeMaxUs(uint32_t durationUs);
    uint32_t getCheckAcknowledgeMaxUs() const;


    // Größter beobachteter Rückstand im Interface, in Bytes.
    uint32_t getRxInterfacePeakBytes() const;

    // Höchststände - der Wert wird nur größer.
    void updateRxQueuePeakBytes(uint32_t used);
    void updateTxControlQueuePeakBytes(uint32_t used);
    void updateTxQueuePeakBytes(uint32_t used);

    uint32_t getRxInterfaceOverflows() const;
    uint32_t getRxQueueOverflows() const;
    uint32_t getTxControlQueueOverflows() const;
    uint32_t getTxQueueOverflows() const;

    // Wie oft die Verbindung zur BCU als verloren galt.
    uint32_t getConnectionLosses() const;

    uint32_t getRxResyncs() const;

    // Fehlermeldungen des Chips (SC, RE, TE, PE, TW wie in showStateErrors()).
    uint32_t getChipSlaveCollisions() const;
    uint32_t getChipReceiveErrors() const;
    uint32_t getChipTransmitErrors() const;
    uint32_t getChipProtocolErrors() const;
    uint32_t getChipTemperatureWarnings() const;

    // Zu lesen gegen TPUART_RX_QUEUE_SIZE, TPUART_CTRL_QUEUE_SIZE und TPUART_TX_BUFFER_SIZE.
    uint32_t getRxQueuePeakBytes() const;
    uint32_t getTxControlQueuePeakBytes() const;
    uint32_t getTxQueuePeakBytes() const;

    // --- Buslast ---------------------------------------------------------------------------------------

    // Aus dem Hauptloop bei jedem Durchlauf; begrenzt sich selbst auf eine Messung je Sekunde.
    void sampleBusLoad();

    // Bytes pro Sekunde der zuletzt abgeschlossenen Sekunde, 0 bis zur ersten.
    uint32_t getBusLoad() const;

    // TP1: 9600 Baud, 13 Bitzeiten je Zeichen = 1354µs je Oktett.
    static constexpr uint32_t BUS_OCTET_TIME_US = 1354;
    static constexpr uint32_t BUS_MAX_BYTES_PER_SECOND = 1000000UL / BUS_OCTET_TIME_US;

    // Pause vor jedem Telegramm: 50 Bitzeiten.
    static constexpr uint32_t BUS_FRAME_GAP_US = 5208;

    // Quittungsslot: 15 Bitzeiten Abstand plus Quittungsoktett - immer reserviert, ob jemand quittiert oder nicht.
    static constexpr uint32_t BUS_ACK_SLOT_US = 2708;

    // Belegte Buszeit in Prozent: Oktetts * BUS_OCTET_TIME_US + Telegramme * (BUS_FRAME_GAP_US + BUS_ACK_SLOT_US).
    // 100% heißt "kein Telegramm passt mehr". Nicht bei 100 gedeckelt - darüber ist ein Befund, daher 16 Bit.
    uint16_t getBusLoadPercent() const;

    // --- KOMPAT: Namen der alten Library, die OGM-Common noch ruft --------------------------------------
    uint32_t getRxReceivedBytes() const;        // -> getRxBytes()
    uint32_t getRxRepetitions() const;          // -> getRxRepeatedFrames()
    uint32_t getRxDiscardedBytes() const;       // -> getRxDroppedBytes()
    uint32_t getRxBusBytes() const;             // -> getRxFrameBytes()
    uint32_t getRxUartOverflow() const;         // -> getRxInterfaceOverflows()
    uint32_t getRxFrameBufferOverflow() const;  // -> getRxQueueOverflows()
    uint32_t getTxOverflowFrameBuffer() const;  // -> getTxQueueOverflows()

    // Platzhalter: den SearchBuffer gibt es nicht mehr, bleibt 0.
    uint32_t getRxSearchBufferOverflow() const;
};

} // namespace TPUart
