#pragma once
#include <stdint.h>

namespace TPUart
{

// Zähler für Diagnose. Übernommen aus der alten Library, aber ohne deren Doppelungen: dort gab es
// getRxUartOverflow() neben getRxOverflowInterface() und getRxFrameBufferOverflow() neben
// getRxOverflowFrameBuffer() - jeweils dasselbe Ereignis unter zwei Namen. Weggefallen sind außerdem die
// Zähler für den SearchBuffer, den es hier nicht mehr gibt.
//
// Neu und der eigentliche Punkt: **fehlerhafte Frames und verworfene Bytes sind zwei verschiedene Dinge**
// und werden getrennt gezählt.
//   - Ein fehlerhaftes Frame ist ein Telegramm, das gemeldet WURDE, aber kaputt ist (CRC falsch, von einer
//     Pause abgeschnitten, Längenbyte korrupt). Der Aufrufer hat es gesehen, mit INVALID-Flag.
//   - Verworfene Bytes hat nie jemand gesehen: alles, was im Resync anfällt, weil nach einem Fehler die
//     Position im Bytestrom unbekannt ist, plus die Reste einer Sequenz, die ein Moduswechsel abbricht.
// Die alte Library kannte nur "discarded" und vermischte damit beides.
//
// Geschrieben wird ausschließlich aus tick() (also je nach Betrieb aus einem Interrupt), gelesen aus dem
// Hauptkontext. Deshalb volatile und nur einfache Inkremente - ein Zähler hat genau einen Schreiber.
// reset() aus dem Hauptkontext kann theoretisch mit einem Inkrement kollidieren; bei reiner Statistik ist
// das hinnehmbar und nicht wert, dafür zu sperren.
class Statistics
{
  private:
    volatile uint32_t _rxReceivedBytes = 0;
    volatile uint32_t _rxFrames = 0;
    volatile uint32_t _rxInvalidFrames = 0;
    volatile uint32_t _rxFrameBytes = 0;
    volatile uint32_t _rxControlBytes = 0;
    volatile uint32_t _rxDroppedBytes = 0;
    volatile uint32_t _rxRepetitions = 0;

    volatile uint32_t _txControlBytes = 0;
    volatile uint32_t _txFrames = 0;

    // JEDES an die Schnittstelle übergebene Byte, unabhängig wofür - das Gegenstück zu _rxReceivedBytes.
    // Der einzige direkte Maßstab für die Auslastung der Hostleitung: bei 38400 Baud und 8E1 (11 Bit je
    // Zeichen) passen dort 3490 Byte/s durch. Gezählt wird in Transmitter::writeByte(), dem einzigen
    // Schreibzugriff dieser Klasse auf das Interface.
    volatile uint32_t _txBytes = 0;
    volatile uint32_t _acknowledgesSent = 0;
    volatile uint32_t _acknowledgesSuppressed = 0;

    volatile uint32_t _interfaceOverflows = 0;
    volatile uint32_t _rxQueueOverflows = 0;
    volatile uint32_t _ctrlQueueOverflows = 0;
    volatile uint32_t _txQueueOverflows = 0;

    // BUSLAST: gemessen wird im Takt, nicht beim Ablesen. Vorher rechnete getBusLoad() über die Zeit seit
    // dem letzten AUFRUF - und die ist unbegrenzt, weil der von der Konsole kommt. Beim ersten Abruf nach
    // Stunden Laufzeit war das Ergebnis erstens ein Mittel über die gesamte Uptime statt einer aktuellen
    // Last, und zweitens lief die Zwischengröße (Bytes mal 1000) in 32 Bit über. Mit dem festen Fenster
    // sind beide Probleme weg.
    //
    // Es ist ein GLEITENDER MITTELWERT über BUS_LOAD_WINDOW Sekunden: im Takt von BUS_LOAD_SLICE_MS
    // wandert der Zählerstand mit seinem Zeitstempel in den Ring. Der Ring liefert damit den ANFANGSPUNKT
    // der Messung; das ENDE ist der Zählerstand im Moment der Abfrage. Die Last ist die Differenz der
    // beiden, geteilt durch die Zeit dazwischen.
    //
    // Deshalb hat der Ring genau BUS_LOAD_WINDOW Plätze und nicht einen mehr: der jüngste Messpunkt muss
    // nicht gespeichert werden, er entsteht beim Lesen. Das hält den Wert zugleich aktuell - er hinkt nicht
    // bis zu einer Sekunde hinterher, wie es ein gespeicherter Endpunkt täte.
    //
    // Die Spanne ist damit nicht fix, sondern liegt zwischen BUS_LOAD_WINDOW - 1 und BUS_LOAD_WINDOW
    // Sekunden - je nachdem, wie lange die letzte Momentaufnahme her ist. Das macht nichts: geteilt wird
    // durch die GEMESSENE Zeit zwischen den beiden Punkten, nicht durch eine angenommene. Exakt 3000ms
    // träfe man ohnehin nie.
    //
    // Beide Zahlen sind fest und nicht überschreibbar - sie beschreiben eine Anzeige, keine Betriebsart.
    // Der Takt bestimmt, wie fein das Fenster nachrückt; die Breite, wie ruhig der Wert ist. Eine
    // einzelne Sekunde ist als Auskunft zu unruhig: ein maximales Telegramm belegt den Bus 356ms, drei
    // davon füllen eine Sekunde fast allein aus - der Wert spränge zwischen Spitze und Leerlauf, ohne
    // dass sich am Bus etwas geändert hätte.
    static constexpr uint32_t BUS_LOAD_SLICE_MS = 1000;
    static constexpr uint8_t BUS_LOAD_WINDOW = 3;

    // Eine Momentaufnahme des Bus-Zählers. Gespeichert wird der ZÄHLERSTAND, nicht die Differenz - damit
    // ist die Last zwischen zwei beliebigen Punkten ausrechenbar, ohne dass beim Messen schon feststehen
    // müsste, welche das sind. Und der Zeitstempel gehört zum Eintrag, nicht zum Takt: kommt der Hauptloop
    // einmal später dran, steht darin die echte Zeit und die Rechnung stimmt trotzdem.
    struct BusLoadSample
    {
        uint32_t _bytes;
        uint32_t _at;
    };

    BusLoadSample _busLoadSamples[BUS_LOAD_WINDOW] = {};
    uint8_t _busLoadIndex = 0; // nächster Schreibplatz - und zugleich der älteste Eintrag
    uint8_t _busLoadCount = 0; // wie viele Plätze belegt sind, bis der Ring einmal voll gelaufen ist

  public:
    void reset();

    void incrementRxReceivedBytes(uint32_t increment = 1);
    void incrementRxFrames(uint32_t increment = 1);
    void incrementRxInvalidFrames(uint32_t increment = 1);
    void incrementRxFrameBytes(uint32_t increment = 1);
    void incrementRxControlBytes(uint32_t increment = 1);
    void incrementRxDroppedBytes(uint32_t increment = 1);

    // Aus dem Hauptkontext (loop()), nicht aus tick() - der Wiederholungsfilter läuft dort.
    void incrementRxRepetitions(uint32_t increment = 1);

    void incrementTxControlBytes(uint32_t increment = 1);
    void incrementTxFrames(uint32_t increment = 1);
    void incrementAcknowledgesSent(uint32_t increment = 1);
    void incrementAcknowledgesSuppressed(uint32_t increment = 1);

    void incrementInterfaceOverflows(uint32_t increment = 1);
    void incrementRxQueueOverflows(uint32_t increment = 1);
    void incrementCtrlQueueOverflows(uint32_t increment = 1);

    // Anders als die übrigen Überläufe wird dieser aus dem HAUPTKONTEXT gezählt (sendFrame()), nicht aus
    // tick() - der Zähler hat damit trotzdem genau einen Schreiber.
    void incrementTxQueueOverflows(uint32_t increment = 1);

    uint32_t getRxReceivedBytes() const;
    uint32_t getRxFrames() const;        // gemeldet und in Ordnung
    uint32_t getRxInvalidFrames() const; // gemeldet, aber kaputt
    // Telegramm-Bytes, und die eines Poll-Telegramms zählen mit - beides ist Verkehr auf dem Bus. Ein
    // eigener Poll-Zähler wäre eine Zeile in jeder Statistikausgabe, die praktisch immer null zeigt.
    uint32_t getRxFrameBytes() const;

    uint32_t getRxControlBytes() const;
    uint32_t getRxDroppedBytes() const; // nie gemeldet, im Resync weggeworfen

    // Gemeldet, aber als Wiederholung markiert (TP_FRAME_FLAG_FILTERED) - der Aufrufer soll sie sehen und
    // nicht noch einmal verarbeiten.
    uint32_t getRxRepetitions() const;

    void incrementTxBytes(uint32_t increment = 1);

    uint32_t getTxControlBytes() const;
    uint32_t getTxFrames() const;

    // Alles, was je an die Schnittstelle ging - siehe _txBytes.
    uint32_t getTxBytes() const;
    uint32_t getAcknowledgesSent() const;

    // Nicht quittiert, weil das Telegramm auf dem Bus schon durch war (Verarbeitung hing hinterher).
    // Kein Fehler, sondern die gewollte Schutzmaßnahme - ein steigender Wert heißt aber: der Tick kommt
    // nicht schnell genug dran.
    uint32_t getAcknowledgesSuppressed() const;

    uint32_t getInterfaceOverflows() const;
    uint32_t getRxQueueOverflows() const;
    uint32_t getCtrlQueueOverflows() const;
    uint32_t getTxQueueOverflows() const;

    // Grundlage der Buslast: die TELEGRAMM-BYTES, Poll eingeschlossen. Bewusst nicht mehr als das -
    // Steuerbytes kommen von der BCU und nicht vom Bus, sie gehören in die Auslastung der Hostleitung
    // (getTxBytes/getRxReceivedBytes), nicht in die des Busses. Und die im Resync verworfenen Bytes lassen
    // sich niemandem zuordnen; sie fehlen damit in der Last, aber sie sind der Ausnahmefall, und ein
    // geschätzter Anteil wäre schlechter als ein sauber definierter Wert.
    uint32_t getRxBusBytes() const;

    // AUS DEM HAUPTLOOP, bei jedem Durchlauf. Begrenzt sich selbst auf eine Momentaufnahme je Takt -
    // der Aufrufer muss also nicht takten. Bewusst nicht aus tick(): der
    // Leerlaufpfad dort soll frei von millis() bleiben, und für einen Sekundentakt ist der Hauptloop
    // schnell genug. Bleibt er einmal länger weg, verzerrt das nichts: gerechnet wird über die Zeit
    // zwischen den Einträgen, und die stimmt dann eben nicht mehr auf die Sekunde.
    void sampleBusLoad();

    // --- KOMPAT: Namen und Zähler der alten Library ---------------------------------------------------
    //
    // Die ersten vier heißen hier nur anders und geben denselben Wert zurück. Der letzte ist ein
    // PLATZHALTER: den SearchBuffer gibt es nicht mehr, also kann er nie etwas zählen und liefert 0.
    // Alle fünf können weg, sobald knx/OGM-Common umgestellt sind.
    uint32_t getRxDiscardedBytes() const;      // -> getRxDroppedBytes()
    uint32_t getRxUartOverflow() const;        // -> getInterfaceOverflows()
    uint32_t getRxFrameBufferOverflow() const; // -> getRxQueueOverflows()
    uint32_t getTxOverflowFrameBuffer() const; // -> getTxQueueOverflows()
    uint32_t getRxSearchBufferOverflow() const; // DUMMY, immer 0

    // Bytes pro Sekunde über die letzten BUS_LOAD_WINDOW Sekunden, sekündlich nachgeführt.
    // Anders als früher verändert das Ablesen nichts mehr - beliebig oft abrufbar, ohne das Ergebnis des
    // nächsten Aufrufs zu verändern. Gelesen wird aus demselben Kontext, der auch misst (Hauptloop): ein
    // Eintrag sind zwei Felder, und die werden nicht zusammen geschrieben. Solange der Ring erst einen
    // Eintrag hat, gibt es noch keine Spanne - dann 0.
    uint32_t getBusLoad() const;
};

} // namespace TPUart
