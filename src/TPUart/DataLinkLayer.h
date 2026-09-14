#pragma once
#include <stddef.h>
#include <stdint.h>

#include <functional>
#include <vector>

#include "TPUart/Frame.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Receiver.h"
#include "TPUart/RepetitionFilter.h"
#include "TPUart/Statistics.h"
#include "TPUart/SystemState.h"
#include "TPUart/Timer.h"
#include "TPUart/Transmitter.h"
#include "TPUart/Types.h"

namespace TPUart
{

// Statusabfrage je Intervall; ohne empfangenes Byte gilt die Verbindung nach dem Timeout als verloren.
#ifndef TPUART_STATE_INTERVAL_MS
#define TPUART_STATE_INTERVAL_MS 1000
#endif

#ifndef TPUART_CONNECTION_TIMEOUT_MS
#define TPUART_CONNECTION_TIMEOUT_MS 5000
#endif

// Wartezeit auf die Antwort eines Baudraten-Kandidaten.
constexpr uint32_t TPUART_DETECT_RESPONSE_TIMEOUT_MS = 50;

// Pause, bevor die Kandidatenliste erneut probiert wird - die BCU antwortet erst mit Busspannung.
#ifndef TPUART_DETECT_RETRY_INTERVAL_MS
#define TPUART_DETECT_RETRY_INTERVAL_MS 1000
#endif

// Busy-Modus endet nach dieser Zeit von selbst. Der TPUART2 tut das in Hardware, der NCN512x nicht.
#ifndef TPUART_BUSY_MODE_MS
#define TPUART_BUSY_MODE_MS 700
#endif

// Der Datalink Layer: hält Interface, Statistik, Verbindungs- und Chip-Zustand; Receiver und Transmitter
// greifen als friend darauf zu.
//
// Zwei Kontexte, getrennt durch den Ringpuffer im Receiver:
//   tick()  - zeitkritisch, aus dem Timer. Höchstens ein Telegrammbyte je Richtung, blockiert nie,
//             gibt nichts nach außen (kein Serial, kein Heap).
//   loop()  - aus dem Hauptloop. Leert den Ringpuffer, ruft Callbacks, verarbeitet Steuerbytes.
//
// Der Leerlaufpfad von tick() ist der häufigste Code: erst reine Vergleiche, dann erst Interface-Zugriffe.
class DataLinkLayer
{
    friend class Receiver;
    friend class Transmitter;

  public:
    // Das Frame besitzt seine Daten und lebt auf dem Stack des Aufrufs. Nicht const: setFiltered().
    using FrameCallback = std::function<void(Frame &frame)>;

    // Acknowledge-Entscheidung für ein einlaufendes Frame. Nicht gesetzt: es wird nicht quittiert.
    using AcknowledgeCallback = std::function<AckType(uint16_t destination, bool isGroupAddress)>;

    // Meldungen, nur aus loop(). Der Text gilt nur während des Aufrufs.
    using MessageCallback = std::function<void(const char *message, bool error)>;

  private:
    // Zeiger statt Referenz: KOMPAT-begin() liefert das Interface nach. Ohne Interface tut nichts etwas.
    Interface::Abstract *_interface = nullptr;
    Statistics _statistics;
    SystemState _systemState;
    RepetitionFilter _repetitionFilter;

    // Nur Tick: Zeitstempel des vorigen tick() für die Taktmessung, 0 = kein Vorgänger.
    uint32_t _tickLastUs = 0;

    // Nur Hauptkontext: Überwachung der erreichten Taktrate (checkTickRate). 0 = kein Bezugspunkt.
    uint32_t _tickRateCheckedAt = 0;
    uint32_t _tickRateLastTicks = 0;
    bool _tickRateReported = false;

    // Nur Hauptkontext: Start des Busy-Modus, 0 = aus.
    uint32_t _busyModeSince = 0;

    // Vom Tick gesetzt, wenn ein U_Ackn.req den Busy-Modus beendet hat; loop() zieht _busyModeSince nach.
    volatile bool _busyModeCancelled = false;

    // Einmal-Merker für Verluste: vom Tick gesetzt, vom Hauptkontext gelesen und gelöscht.
    volatile bool _interfaceOverflow = false;
    volatile bool _rxQueueOverflow = false;
    volatile bool _ctrlQueueOverflow = false;

    // Fehlerbits aus U_State.ind, verodert bis zur Ausgabe - der Chip meldet jedes Ereignis nur einmal.
    uint8_t _stateErrors = 0;

    // Verbindungsaufnahme (siehe begin() und searchBaudRate()). Die Hälften laufen erst ab Identifying.
    //
    // Zwei Schreiber: der Tick schreibt die Übergänge aus Connected, BusMonitor und Disconnected, der
    // Hauptkontext alle übrigen. begin() und end() tragen den Tick vorher aus.
    BcuType _bcuType = BcuType::Ncn5120;
    volatile BcuState _bcuState = BcuState::Uninitialized;
    uint32_t _connectedBaudRate = 0;

    // _lastReceivedAt schreibt der Tick (jedes empfangene Byte), gelesen im Hauptkontext.
    volatile uint32_t _lastReceivedAt = 0;
    uint32_t _lastStateRequestAt = 0;

    // Verlust: der Hauptkontext meldet die Frist (_connectionTimeouts), der Tick den Übergang (_connectionLosses).
    volatile uint8_t _connectionTimeouts = 0;
    volatile uint8_t _connectionTimeoutsSeen = 0;
    volatile uint8_t _connectionLosses = 0;
    uint8_t _connectionLossesSeen = 0;

    bool _connectReported = false; // die Verbindungsmeldung kommt aus loop()

    // Konfiguration, die die BCU bei jedem Reset vergisst - wird danach erneut abgesetzt.
    uint16_t _ownAddress = 0;
    uint8_t _repetitionsNack = 3;
    uint8_t _repetitionsBusy = 3;

    // Die Spannungsregler des NCN (ACR0). Standardmäßig an; abgesetzt von applyConfiguration().
    bool _powerControl = true;

    // Auto-Quittung des Chips. Zwei Schreiber (Hauptkontext beim Absetzen der Adresse, Tick bei Reset und
    // U_Configure.ind); die Konfigurationsepoche löst jede Uneinigkeit auf.
    volatile bool _autoAcknowledge = false;

    // Konfigurationsepoche: der Tick zählt bei jedem Reset hoch, der Hauptkontext zieht nach.
    volatile uint32_t _configEpoch = 0;
    uint32_t _configAppliedEpoch = 0;

    // Nur Hauptkontext: der Chip war zuletzt nicht in NORMAL (checkChipRestart).
    bool _chipOutOfNormal = false;

    // Schritt beim Bestimmen des Chips (BcuState::Identifying). Nur Hauptkontext.
    enum class RegisterRead : uint8_t
    {
        Start,
        AwaitStop,  // U_StopMode.ind
        AwaitValue, // Wert zu _readRegister
    };

    bool _detectAwaitingResponse = false;
    uint8_t _detectCandidateIndex = 0;
    uint32_t _detectRequestSentAt = 0;
    uint32_t _detectNextAttemptAt = 0;

    RegisterRead _registerRead = RegisterRead::Start;
    uint32_t _registerReadSentAt = 0;
    uint8_t _readRegister = 0;
    bool _stopModeReached = false;
    BcuChip _bcuChip = BcuChip::Unknown;
    uint8_t _ncnRevision = 0;

    // Vom Tick geschrieben: der Wert zum angeforderten Register, der Zähler zuletzt.
    volatile uint8_t _registerValue = 0;
    volatile uint8_t _registerValueCount = 0;
    uint8_t _registerValueSeen = 0;

    // Alle Callbacks liegen hier. Mehrere Frame-Callbacks: knx und OGM-Common registrieren beide.
    std::vector<FrameCallback> _frameCallbacks;
    AcknowledgeCallback _acknowledgeCallback;
    MessageCallback _messageCallback;

    // Die beiden Hälften - nach den geteilten Feldern deklariert.
    Transmitter _transmitter;
    Receiver _receiver;

    // --- Verbindungsaufnahme -----------------------------------------------------------------------
    //
    // Wer das Interface anfasst, hängt an _bcuState:
    //   Searching      - der Hauptkontext (searchBaudRate() konfiguriert es je Kandidat neu, nicht im IRQ)
    //   ab Identifying - der Tick, bis zum nächsten begin(); reconnect() nutzt das offene Interface
    enum class DetectResult : uint8_t
    {
        Pending,   // noch keine Entscheidung
        Connected, // die BCU hat mit U_Reset.ind geantwortet
        Failed,    // falsches Byte oder Frist abgelaufen
    };

    // Nur aus loop(), solange Searching.
    void searchBaudRate();

    // Nur aus tick(), in Disconnected.
    void reconnect();

    // Gemeinsamer Teil beider Wege: höchstens ein Byte ansehen, sonst die Frist prüfen.
    DetectResult pollDetectResponse();

    void connectDetected();

    // Bestimmt den Chip, solange Identifying.
    void processRegisterRead();
    bool readRegister(uint8_t reg);
    bool readNextRegister(uint8_t value);
    void endRegisterRead();
    bool registerReadActive() const;

    // Aus dem Tick: der Wert zum angeforderten Register ist da.
    void registerValueReceived(uint8_t value);

    // Nächster Baudraten-Kandidat (nur aus der Suche).
    void advanceDetectCandidate();

    // Aus loop(): Statusabfrage und Erkennung des Verbindungsverlusts.
    void processConnectionState();

    // Aus dem Tick: der Hauptkontext hat den Verlust festgestellt.
    void connectionLost();

    // Setzt die Konfiguration ab. false, wenn etwas nicht in die Steuer-Warteschlange passte - die Epoche
    // bleibt dann offen.
    bool applyConfiguration();

    // Lässt applyConfiguration() beim nächsten loop() erneut laufen.
    void markConfigurationPending();

    // Erkennt am gemeldeten Chip-Zustand einen unbemerkten Reset. Aus loop(), nach jedem U_SystemStat.ind.
    void checkChipRestart();

    // --- Rückmeldungen der beiden Hälften ----------------------------------------------------------

    // Aus dem Tick (Transmitter): ein Steuercode ist abgesetzt - erst jetzt gilt der Chip als umgeschaltet.
    void controlByteSent(uint8_t code);

    // Aus dem Tick (Receiver): U_Reset.ind, von wem auch immer ausgelöst.
    void resetIndication();

    // Aus dem Tick (Receiver): U_Configure.ind.
    void configureIndication(uint8_t value);

    // Aus dem Tick: setzt den Einmal-Merker und zählt.
    void reportInterfaceOverflow();
    void reportRxQueueOverflow();
    void reportControlOverflow();

    // Aus loop() (Receiver): ein fertiges Telegramm an die Frame-Callbacks.
    void deliverFrame(Frame &frame);

    // Aus dem TICK (Receiver): Acknowledge-Entscheidung zum einlaufenden Telegramm.
    AckType checkAcknowledge(uint16_t destination, bool isGroupAddress);

    // Aus loop() (Receiver): eine Steuerbyte-Sequenz. Geht nicht nach außen, nur als Meldung.
    void handleControlEntry(const uint8_t *data, size_t length);

    // Nur aus loop(): formatieren auf den Stack und rufen fremden Code.
    void printMessage(const char *format, ...) __attribute__((format(printf, 2, 3)));
    void printError(const char *format, ...) __attribute__((format(printf, 2, 3)));

    // Nimmt den Busy-Modus nach TPUART_BUSY_MODE_MS zurück.
    void checkBusyMode();

    // Meldet eine Taktrate unterhalb dessen, was der Bus braucht. Aus loop().
    void checkTickRate();

    // Aus dem Tick: ein U_Ackn.req ist rausgegangen (siehe _busyModeCancelled).
    void reportBusyModeCancelled();

    // Aus loop(): Änderungen am System-Status und aufgelaufene Fehlerbits ausgeben.
    void showSystemState();
    void showStateErrors();

  public:
    explicit DataLinkLayer(Interface::Abstract &interface);

    // KOMPAT: ohne Interface angelegt, es kommt in begin().
    DataLinkLayer();

    // Trägt sich beim Timer aus - der hält einen Zeiger auf diese Instanz.
    ~DataLinkLayer();

    // Startet die Verbindungsaufnahme, nicht blockierend: gesucht wird aus loop(), das also laufen muss.
    // Die Baudrate wird je begin() einmal gesucht; nach einem Verlust wird nur mit ihr neu verbunden.
    void begin(BcuType bcuType);

    // KOMPAT: setzt das Interface und ruft begin(bcuType).
    void begin(BcuType bcuType, Interface::Abstract *interface);

    // Wird diese Instanz vom zentralen Timer getickt? false: keine Timer-Plattform, Intervall 0 oder kein
    // Platz frei. Nur Auskunft - einen fehlenden Antrieb meldet checkTickRate() selbst.
    bool usesTimer() const;

    // KOMPAT: beendet den Betrieb und schließt das Interface, gibt es aber nicht frei.
    void end();

    // Zeitkritische Seite. Von außen nur rufen, wenn der Timer diese Instanz nicht treibt.
    void tick();

    // Leert den RX-Ringpuffer. Nur aus dem Hauptloop.
    void loop();

    // KOMPAT: ruft nur loop() - hier wird nie getickt.
    void process();

    // Identifying, Connected oder BusMonitor.
    bool isConnected() const;
    uint32_t connectedBaudRate() const;
    BcuType bcuType() const;

    // Verbauter Chip, bestimmt je begin(). Unknown, solange nicht bestimmt.
    BcuChip bcuChip() const;

    // Silizium-Revision eines NCN5121/5130. 0, wenn nicht gelesen.
    uint8_t ncnRevision() const;

    BcuState bcuState() const;
    const char *bcuStateName() const;

    // Aus loop() gerufen. Mehrfach registrierbar, jeder Callback bekommt jedes Telegramm.
    void registerFrameCallback(FrameCallback callback);

    // KOMPAT: alter Name von registerFrameCallback().
    void registerReceivedFrame(FrameCallback callback);

    // ACHTUNG, aus tick() gerufen: kurz, nicht blockierend, ohne Heap - und vor dem Start des Tick-Antriebs
    // setzen, die Zuweisung ist nicht atomar.
    void registerCheckAcknowledge(AcknowledgeCallback callback);

    // Ausgabekanal für Meldungen. Ohne ihn entfallen sie.
    void registerMessage(MessageCallback callback);

    // Einmal-Meldungen, setzen sich beim Abfragen zurück. Aus dem Hauptkontext.
    bool queueOverflow();     // ein Telegramm wurde verworfen, weil der Ringpuffer voll war
    bool interfaceOverflow(); // das Interface hat Daten verloren
    bool controlOverflow();   // ein Steuercode wurde verworfen, weil seine Warteschlange voll war

    // Die beiden Hälften, für Diagnose.
    Receiver &getReceiver();
    Transmitter &getTransmitter();

    Statistics &getStatistics();

    // Für clear() und Diagnose; der Filter arbeitet in loop() von selbst.
    RepetitionFilter &getRepetitionFilter();

    // Letzter gemeldeter System-Status - nur NCN512x, gefüllt durch requestState().
    SystemState &getSystemState();

    // --- Steuerbefehle an die BCU -------------------------------------------------------------------
    //
    // Rückgabe heißt "eingereiht", nicht "auf dem Bus". false: keine Verbindung, falscher Chiptyp oder
    // Warteschlange voll (controlOverflow()).

    // Busmonitor einschalten. Verlassen nur per reset(). Schon aktiv: true, ohne zu senden.
    bool startMonitoring();

    // Setzt den Chip zurück; beendet den Busmonitor und stellt den Default-CRC-Modus her.
    bool reset();

    // U_State.req, beim NCN512x zusätzlich U_SystemState.req.
    bool requestState();

    // Stop-Modus betreten/verlassen. Nur NCN512x.
    bool stopMode(bool state);

    // Busy-Modus: adressierte Telegramme werden mit BUSY quittiert. Endet nach TPUART_BUSY_MODE_MS von selbst.
    bool busyMode(bool state);

    // Schaltet die Spannungsregler VCC2/20V des Chips (Schreibzugriff auf ACR0). Nur NCN512x.
    bool powerControl(bool state);

    // Schreibt ein internes Register (NCN_REG_*). Nur NCN512x. Der Wert überlebt keinen Reset.
    bool writeRegister(uint8_t reg, uint8_t value);

    // Physikalische Adresse. Aktiviert die Auto-Quittung des Chips als Rückfall - unser eigenes Acknowledge
    // bleibt. 0 = keine Adresse; abschalten lässt sich die Auto-Quittung nur per reset(). Wird nach jedem
    // Reset erneut abgesetzt.
    bool setOwnAddress(uint16_t address);
    uint16_t ownAddress() const;

    // Wiederholungen nach NACK und BUSY, je 0...7. false bei zu großem Wert oder voller Warteschlange.
    bool setRepetitions(uint8_t nack, uint8_t busy);

    // Auto-Quittung im Chip aktiv? Reine Auskunft, ändert am eigenen Acknowledge nichts.
    bool isAutoAcknowledge() const;

    // Läuft unser Busy-Zeitgeber?
    bool isBusyMode() const;

    // Der tatsächliche Zustand des Chips, nicht der gewünschte.
    bool isBusMonitor() const;

    // KOMPAT: alte Namen.
    bool isMonitoring() const;      // -> isBusMonitor()
    const char *getBcuStateInfo() const; // -> bcuStateName()

    // --- Telegrammversand ---------------------------------------------------------------------------
    //
    // Reiht ein vollständiges Telegramm samt Prüfsumme ein; die Prüfsumme wird geprüft, nicht neu gerechnet.
    // Abgearbeitet nach Priorität. false: keine Verbindung, Busmonitor, ungültiges Telegramm oder
    // Warteschlange voll - der Grund geht an den Message-Callback. Die Daten werden kopiert.
    bool pushTransmitQueue(const Frame &frame);
    bool pushTransmitQueue(const uint8_t *data, size_t length);

    // KOMPAT: übernimmt den Besitz - bei Erfolg wird das Frame gelöscht. Der knx-Stack verlässt sich darauf.
    bool pushTransmitQueue(Frame *frame);
};

} // namespace TPUart
