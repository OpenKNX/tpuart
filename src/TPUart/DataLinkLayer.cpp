#include "TPUart/DataLinkLayer.h"

// Lebenszeichen beim Bau: zeigt im Fremdprojekt, dass diese Version gelinkt wird. In der .cpp, damit die
// Zeile einmal je Bau kommt und nicht einmal je Übersetzungseinheit.
#pragma message "TPUART v2"
    
#include <stdarg.h>
#include <stdio.h>

#include <Arduino.h>

#include <string>

// Hier gibt es bewusst KEINE Sperre und keinen plattformabhängigen Code. Alles, was sich Tick-Antrieb und
// Hauptkontext teilen, ist über Besitz und Reihenfolge geregelt statt über Locks:
//   - RX-, TX- und Steuercode-Warteschlange: je ein Produzent, ein Konsument, Kopf zuletzt sichtbar
//     gemacht - der Konsument sieht damit nie einen halb geschriebenen Eintrag
//   - Callbacks: werden vor dem Start des Tick-Antriebs gesetzt, also ohne gleichzeitigen Leser
// Ein blockierendes Lock wäre ohnehin nicht möglich - auf dem RP2040 läuft tick() im Interrupt.

namespace
{

constexpr uint32_t TPUART2_BAUD_RATES[] = {19200};
constexpr uint32_t NCN5120_BAUD_RATES[] = {19200, 38400};

// Kandidaten für die Baudratenerkennung, abhängig vom Chip: der TPUART2 kennt nur 19200 fest, die
// NCN512x-Reihe zusätzlich 38400 (siehe alte Library, DataLinkLayer.cpp::tryInitialize()).
void candidateBaudRates(TPUart::BcuType type, const uint32_t *&rates, size_t &count)
{
    if (type == TPUart::BcuType::Tpuart2)
    {
        rates = TPUART2_BAUD_RATES;
        count = 1;
        return;
    }
    rates = NCN5120_BAUD_RATES;
    count = 2;
}

} // namespace

namespace TPUart
{

DataLinkLayer::DataLinkLayer(Interface::Abstract &interface) : _interface(&interface), _transmitter(*this), _receiver(*this) {}

// KOMPAT: ohne Interface - es kommt dann über begin(bcuType, interface).
DataLinkLayer::DataLinkLayer() : _transmitter(*this), _receiver(*this) {}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft später aus einem Timer-Interrupt
// ---------------------------------------------------------------------------------------------------

// Hält sich nirgends auf und blockiert nie.
void DataLinkLayer::tick()
{
    if (_interface == nullptr) return; // ohne Interface gibt es nichts zu tun (siehe KOMPAT-Konstruktor)
    if (!_initialized) return;         // begin() wurde noch nicht aufgerufen

    // Die Baudraten-SUCHE gehört dem Hauptkontext, weil sie das Interface neu konfiguriert - hier ist
    // deshalb Schluss, bis die Verbindung zum ersten Mal gestanden hat (siehe searchBaudRate).
    if (!_everConnected) return;

    if (!_connected)
    {
        reconnect();
        return;
    }

    _receiver.process();
    _transmitter.process();
}

void DataLinkLayer::begin(BcuType bcuType)
{
    _bcuType = bcuType;
    _initialized = true;
    _connected = false;
    _detectAwaitingResponse = false;
    _detectCandidateIndex = 0;
    _detectNextAttemptAt = millis(); // sofort beim nächsten loop() fällig

    // Der Antrieb darf sofort loslaufen, obwohl noch keine Verbindung steht: bis _everConnected gesetzt
    // ist, kehrt tick() gleich wieder um und fasst das Interface nicht an - die Baudratensuche gehört dem
    // Hauptkontext (siehe searchBaudRate). Schlägt der Start fehl oder kennt die Plattform keinen Timer,
    // bleibt es beim Antrieb aus process(); gemeldet wird das nicht, weil hier noch kein
    // Message-Callback gesetzt sein muss - hasTickDriver() gibt jederzeit Auskunft.
    if (_tickIntervalUs > 0 && !_externalTick) _tickDriver.start(*this, _tickIntervalUs);
}

// KOMPAT: die alte Library bekam das Interface hier statt im Konstruktor.
void DataLinkLayer::begin(BcuType bcuType, Interface::Abstract *interface)
{
    _interface = interface;
    begin(bcuType);
}

void DataLinkLayer::setTickInterval(uint32_t intervalUs)
{
    _tickIntervalUs = intervalUs;

    // Ein laufender Antrieb wird angehalten, aber nicht neu gestartet - sonst hinge an einer bloßen
    // Zahlenänderung ein Wechsel des Ausführungskontextes mitten im Betrieb.
    if (intervalUs == 0) _tickDriver.stop();
}

// Ein laufender Timer wird sofort angehalten - ab jetzt tickt der Aufrufer, und zwei Kontexte auf derselben
// Schnittstelle sind ein Defekt, kein Nachteil. Dass _tickIntervalUs unangetastet bleibt, ist Absicht: der
// Wert beschreibt die gewünschte Taktrate, _externalTick den Antrieb. Nur begin() fragt beide zusammen.
void DataLinkLayer::setExternalTick(bool enabled)
{
    _externalTick = enabled;

    if (enabled) _tickDriver.stop();
}

bool DataLinkLayer::hasTickDriver() const
{
    return _tickDriver.running();
}

// KOMPAT: Gegenstück zu begin(). Das Interface wird geschlossen, aber NICHT freigegeben - es gehört dem
// Aufrufer.
void DataLinkLayer::end()
{
    // ZUERST den Antrieb anhalten: danach ist garantiert kein tick() mehr unterwegs, der gleich in ein
    // geschlossenes Interface greifen würde.
    _tickDriver.stop();

    _initialized = false;
    _connected = false;

    if (_interface != nullptr) _interface->end();
}

// KOMPAT: ein Einstieg für beide Hälften, wie in der alten Library. Der Tick kommt nur dann von hier, wenn
// ihn sonst niemand antreibt - weder der eigene Timer noch ein externer Antrieb (setExternalTick). Beides
// wäre ein zweiter Kontext in derselben State-Machine, und der zerlegt Sende- wie Empfangsstrom.
void DataLinkLayer::process()
{
    if (!_tickDriver.running() && !_externalTick) tick();
    loop();
}

// Nicht-blockierendes Gegenstück zur alten Library: dort wartete tryInitialize(baudrate) bis zu 50ms in
// einer Busy-Loop auf die Antwort. Hier wird pro Durchlauf höchstens ein Byte betrachtet, der Ablauf ist
// über _detectAwaitingResponse/_detectRequestSentAt auf mehrere Aufrufe verteilt.
//
// LÄUFT IM HAUPTKONTEXT (aus loop()), und das ist der Grund für die Aufteilung in searchBaudRate() und
// reconnect(): pro Kandidat wird das Interface neu konfiguriert, und das gehört in keinen Interrupt. Der
// Tick kehrt deshalb um, solange !_everConnected - siehe den Block im Header.
//
// GESUCHT wird die Baudrate nur EINMAL. Sie ist eine Hardware-Eigenschaft der BCU (beim TP-UART 2+ etwa der
// BDS-Pin) und kann sich im Betrieb nicht ändern. Nach einem Verbindungsverlust übernimmt darum
// reconnect() mit der bekannten Rate; eine andere zu finden verlangt einen Neustart. Diese Funktion läuft
// danach nie wieder.
void DataLinkLayer::searchBaudRate()
{
    if (_detectAwaitingResponse)
    {
        switch (pollDetectResponse())
        {
            case DetectResult::Pending:
                return;
            case DetectResult::Connected:
                connectDetected();
                return;
            case DetectResult::Failed:
                advanceDetectCandidate();
                return;
        }
    }

    if ((int32_t)(millis() - _detectNextAttemptAt) < 0) return; // noch nicht fällig

    const uint32_t *rates;
    size_t count;
    candidateBaudRates(_bcuType, rates, count);
    uint32_t baud = rates[_detectCandidateIndex % count];

    // Baudratenwechsel erfordert einen sauberen Neustart des Interfaces - ein bereits laufendes würde bei
    // reinem begin() ohne vorheriges end() undefiniert reagieren (RP2040 würde z.B. einen zweiten
    // DMA-Channel beanspruchen wollen).
    _interface->end();
    _interface->begin(baud);

    // Geht der Request nicht raus, wird auch nicht auf Antwort gewartet: 50ms auf etwas zu warten, das
    // nie gefragt wurde, würde diesen Kandidaten grundlos verbrennen. Nach einem frischen begin() ist der
    // Sendepuffer leer, das kann also praktisch nicht vorkommen - kostet aber auch nichts.
    if (!_interface->write((char)U_RESET_REQ)) return;

    _detectAwaitingResponse = true;
    _detectRequestSentAt = millis();
}

// Aus dem Tick, nach einem Verbindungsverlust. Derselbe Ablauf wie bei der Suche, mit einem Unterschied:
// das Interface wird NICHT angefasst. Die Baudrate steht fest, das Interface ist offen und richtig
// konfiguriert - es geht nur noch darum, das U_Reset.req zu wiederholen, bis die BCU wieder antwortet.
//
// Geduldig, weil die BCU erst antwortet, sobald am Bus die Busspannung anliegt: nach jedem Fehlversuch
// TPUART_DETECT_RETRY_INTERVAL_MS Pause, statt eng weiterzuprobieren.
void DataLinkLayer::reconnect()
{
    if (_detectAwaitingResponse)
    {
        switch (pollDetectResponse())
        {
            case DetectResult::Pending:
                return;
            case DetectResult::Connected:
                connectDetected();
                return;
            case DetectResult::Failed:
                _detectAwaitingResponse = false;
                _detectNextAttemptAt = millis() + TPUART_DETECT_RETRY_INTERVAL_MS;
                return;
        }
    }

    if ((int32_t)(millis() - _detectNextAttemptAt) < 0) return; // noch nicht fällig

    if (_interface->availableForWrite() < 1) return;
    if (!_interface->write((char)U_RESET_REQ)) return;

    _detectAwaitingResponse = true;
    _detectRequestSentAt = millis();
}

// Wichtig für die Korrektheit: die Antwort muss UNMITTELBAR auf unseren eigenen Request folgen. Bei noch
// unbestätigter Baudrate ist gelesener Datenmüll nämlich beliebig - er könnte zufällig wie ein U_Reset.ind
// aussehen. Deshalb wird nur ausgewertet, während _detectAwaitingResponse gesetzt ist, und das erste Byte,
// das dabei ankommt und nicht die (laut altem Kommentar "TPUart sendet Nullen am Anfang") ignorierbare 0
// ist, entscheidet sofort über Erfolg oder Misserfolg - es wird nicht weiter auf ein verspätetes 0x03
// gewartet.
DataLinkLayer::DetectResult DataLinkLayer::pollDetectResponse()
{
    if (_interface->available())
    {
        int value = _interface->read();
        if (value >= 0)
        {
            if (value == 0) return DetectResult::Pending; // führende Null - noch keine Entscheidung
            if (value == U_RESET_IND) return DetectResult::Connected;

            return DetectResult::Failed; // irgendein anderes Byte statt der erwarteten Antwort
        }
    }

    if ((uint32_t)(millis() - _detectRequestSentAt) >= TPUART_DETECT_RESPONSE_TIMEOUT_MS) return DetectResult::Failed;

    return DetectResult::Pending;
}

// Die BCU hat geantwortet. Reihenfolge beachten: _everConnected veröffentlicht den Besitzwechsel am
// Interface, _connected ganz zuletzt gibt die beiden Hälften frei - erst wenn alles davor steht.
void DataLinkLayer::connectDetected()
{
    const uint32_t *rates;
    size_t count;
    candidateBaudRates(_bcuType, rates, count);

    // Dieselbe Behandlung wie bei jedem anderen Reset: der U_Reset.ind ist zugleich die Bestätigung, dass
    // der Sendepuffer der BCU leer ist und sie Adresse und Zähler vergessen hat.
    resetIndication();

    _connectedBaudRate = rates[_detectCandidateIndex % count];
    _detectAwaitingResponse = false;
    _lastReceivedAt = millis(); // sonst gälte die Verbindung sofort als still
    _everConnected = true;
    _connected = true;
}

// Ein Kandidat ist gescheitert (falsches Byte oder Frist abgelaufen) - weiter zum nächsten. Ist die Liste
// am Ende angekommen, wird nicht sofort neu begonnen, sondern erst nach TPUART_DETECT_RETRY_INTERVAL_MS:
// die BCU antwortet erst, sobald am Bus die Busspannung anliegt, und das kann nach dem eigenen Boot noch
// beliebig lange dauern - ein enges Dauerprobieren brächte bis dahin nichts außer unnötigem Bus-Traffic.
void DataLinkLayer::advanceDetectCandidate()
{
    _detectAwaitingResponse = false;

    const uint32_t *rates;
    size_t count;
    candidateBaudRates(_bcuType, rates, count);

    _detectCandidateIndex++;
    if (_detectCandidateIndex >= count)
    {
        _detectCandidateIndex = 0;
        _detectNextAttemptAt = millis() + TPUART_DETECT_RETRY_INTERVAL_MS;
    }
}

// ---------------------------------------------------------------------------------------------------
// Rückmeldungen der beiden Hälften - aus dem Tick
// ---------------------------------------------------------------------------------------------------

// Ein Steuercode hat den Chip erreicht (Transmitter). Nur diese beiden ändern etwas am Zustand, den die
// Library über den Chip führt.
void DataLinkLayer::controlByteSent(uint8_t code)
{
    if (code == U_BUSMON_REQ) _busMonitor = true;
    if (code == U_RESET_REQ) _busMonitor = false;

    // Nur diese beiden Codes ändern die Bedeutung des Bytestroms (im Busmonitor kommen die Quittungen vom
    // Bus dazu, nach einem Reset fallen sie weg). Ab wann genau der Chip umschaltet, ist von hier aus nicht
    // erkennbar, eine laufende Sequenz also nicht mehr verlässlich zu deuten. Andere Steuercodes lassen das
    // Format unberührt - dort wäre ein Resync nur ein unnötig verworfenes Telegramm.
    if (code == U_BUSMON_REQ || code == U_RESET_REQ) _receiver.forceResync();
}

// Die BCU hat sich zurückgesetzt - von wem auch immer veranlasst: vom Wachhund des Transmitters, von
// reset() aus der Anwendung, oder von ihr selbst. In jedem Fall ist ihr Sendepuffer leer und ihr Zustand
// definiert. Unterschieden wird deshalb NICHT, wer den Reset ausgelöst hat: liegt noch ein Telegramm im
// Sendepuffer, beginnt es einfach von vorn; liegt keines, holt der Tick das nächste aus der Warteschlange.
void DataLinkLayer::resetIndication()
{
    _busMonitor = false;      // der Reset beendet ihn, auch wenn wir ihn nicht selbst geschickt haben
    _autoAcknowledge = false; // "After reset the address evaluation is deactivated again"

    // Adresse und Wiederholungszähler sind damit im Chip weg - der Hauptkontext setzt sie neu ab.
    _configEpoch = _configEpoch + 1;

    _transmitter.restart();
}

// Der Chip meldet seine Betriebsarten. Für die Auto-Quittung ist das eine Bestätigung, keine Nachricht:
// dass sie mit der Adresse aktiv wird, weiß applyConfiguration() schon beim Absetzen - und muss es auch
// wissen, weil dieser Dienst nur beim NCN existiert. Gilt aber trotzdem der Chip: was er hier meldet, ist
// der wahre Zustand, auch wenn er dem Flag widerspricht.
void DataLinkLayer::configureIndication(uint8_t value)
{
    _autoAcknowledge = (value & U_CONFIGURE_AUTO_ACKNOWLEDGE) != 0;
}

// ---------------------------------------------------------------------------------------------------
// Gemütliche Seite - läuft aus dem Hauptloop
// ---------------------------------------------------------------------------------------------------

void DataLinkLayer::loop()
{
    if (_interface == nullptr) return;

    _receiver.processQueue();

    // Der Tick darf keinen Heap anfassen, also wird hier aufgeräumt, was er abgeholt hat.
    _transmitter.releaseSentTelegrams();

    // Die Baudratensuche gehört hierher und nicht in den Tick: sie konfiguriert das Interface pro Kandidat
    // neu. Sie läuft genau einmal, bis die Verbindung zum ersten Mal steht - danach übernimmt der Tick
    // (siehe den Block bei searchBaudRate).
    if (!_everConnected)
        searchBaudRate();
    else
        processConnectionState();

    // Begrenzt sich selbst auf einen Messwert je Sekunde, steht deshalb hier und nicht in
    // processConnectionState(): auch ohne Verbindung soll die gemeldete Buslast auf null fallen statt
    // beim letzten Wert stehenzubleiben.
    _statistics.sampleBusLoad();

    checkBusyMode();

    // Der eine Reset, der einen Fehler anzeigt: der Wachhund des Transmitters hat zugeschlagen, weil zum
    // gesendeten Telegramm keine Bestätigung kam. Er kann nicht selbst melden - er läuft im Tick.
    if (_transmitter.confirmTimeout())
        printError("No L_Data.con for %u ms - BCU reset, telegram will be sent again", (unsigned)TPUART_TX_CONFIRM_TIMEOUT_MS);

    // Erst nachdem der Ringpuffer leer ist: die Meldungen entstehen zum Teil genau dort oben, und in
    // derselben Runde ausgegeben zu werden ist die Erwartung des Aufrufers.
    showSystemState();
    showStateErrors();
}

// Aus loop(), also aus dem Hauptkontext - hier darf queueControl() benutzt und gemeldet werden.
//
// Zwei Aufgaben in einer Frist: die regelmäßige Statusabfrage und der Verbindungsverlust. Ohne die Abfrage
// ließe sich ein Ausfall nicht von einem ruhigen Bus unterscheiden - dort kommt regulär minutenlang nichts.
// Als Lebenszeichen zählt jedes empfangene Byte (siehe Receiver::process), die Abfrage sorgt nur dafür, dass
// es auch bei völliger Stille eines gibt.
void DataLinkLayer::processConnectionState()
{
    if (!_connected) return; // die Erkennung läuft, die überwacht sich selbst

    // Herstellen kann die Verbindung beides (searchBaudRate hier, reconnect im Tick), melden nur der Hauptkontext - also hier,
    // beim ersten loop() danach.
    if (!_connectReported)
    {
        _connectReported = true;
        printMessage("BCU connected (%u baud)", (unsigned)_connectedBaudRate);
    }

    // REIHENFOLGE UND VORZEICHEN SIND HIER BEIDES PFLICHT, und beides hat gefehlt.
    //
    // _lastReceivedAt schreibt der Tick, also ein Interrupt, und zwar mitten zwischen zwei beliebige
    // Instruktionen von hier. Stand `now = millis()` VOR dem Lesen von _lastReceivedAt, dann konnte
    // dazwischen ein Byte eintreffen und einen Zeitstempel setzen, der GRÖSSER ist als das schon
    // eingefrorene now. Die Differenz wurde negativ, lief als uint32_t auf ~4,29 Mrd. über und war damit
    // sicher >= der Frist: sofortiger Verbindungsabbruch, obwohl gerade eben ein Byte kam.
    //
    // Das Fenster ist nur ein paar Instruktionen breit - bei 38400 Baud trifft aber alle ~286µs ein Byte
    // hinein. Der Fehler trat deshalb ausgerechnet unter Last auf (ETS-Programmierung über den Router)
    // und war sonst kaum zu provozieren.
    //
    //   1. ZUERST den Zeitstempel greifen, DANACH die Uhr lesen. Dann liegt now nie vor last.
    //   2. Trotzdem VORZEICHENBEHAFTET vergleichen: der Tick kann auch nach Schritt 1 noch schreiben, und
    //      ein minimal vorauseilender Zeitstempel bedeutet "gerade eben empfangen", nicht "ewig her".
    uint32_t last = _lastReceivedAt;
    uint32_t now = millis();

    if ((int32_t)(now - last) >= (int32_t)TPUART_CONNECTION_TIMEOUT_MS)
    {
        connectionLost();
        return;
    }

    // Nach jedem Reset der BCU steht hier eine neue Epoche - dann muss die Konfiguration erneut raus.
    // Die Epoche wird erst nachgezogen, wenn alles in die Steuer-Warteschlange gepasst hat; war sie voll,
    // bleibt es beim nächsten loop() nochmal offen. Sonst wäre die Konfiguration weg, ohne dass es
    // irgendwo auffiele - und mit ihr die Quittung für alles, was an unsere Adresse geht.
    if (_configAppliedEpoch != _configEpoch)
    {
        uint32_t epoch = _configEpoch; // erst lesen, dann anwenden: der Tick kann sie dazwischen erhöhen
        if (applyConfiguration()) _configAppliedEpoch = epoch;
    }

    if ((uint32_t)(now - _lastStateRequestAt) < TPUART_STATE_INTERVAL_MS) return;

    _lastStateRequestAt = now;
    requestState();
}

// Die BCU antwortet nicht mehr. Weiter geht es mit reconnect() im Tick - mit der bereits festgestellten
// Baudrate, nicht als neue Suche. _detectCandidateIndex bleibt deshalb unangetastet: er zeigt noch auf den
// Kandidaten, der beim ersten Mal geantwortet hat, und connectDetected() liest ihn wieder.
//
// Der Ablauf ist damit derselbe wie beim Start: U_Reset.req an die BCU, und die U_Reset.ind gilt als
// Bestätigung. Sie ist zugleich der definierte Zustand, den wir nach einer Störung ohnehin haben wollen.
//
// Reihenfolge: erst die Felder der Erkennung, _connected ALS LETZTES. Der Tick liest sie nur, wenn
// _connected false ist - er sieht sie damit entweder gar nicht oder vollständig vorbereitet. Das Interface
// bleibt dabei durchgehend offen und in der Hand des Ticks; der Hauptkontext fasst es nach der ersten
// Verbindung nie wieder an (siehe searchBaudRate).
void DataLinkLayer::connectionLost()
{
    printError("BCU disconnected - no byte received for %u ms", (unsigned)TPUART_CONNECTION_TIMEOUT_MS);

    _connectReported = false;
    _detectAwaitingResponse = false;
    _detectNextAttemptAt = millis();
    _connected = false;
}

// Steuerbytes gehen bewusst NICHT nach außen - ihre Deutung ist Sache des Datalink Layers selbst, der
// Aufrufer bekommt davon nur die Meldungen zu sehen.
void DataLinkLayer::handleControlEntry(const uint8_t *data, size_t length)
{
    uint8_t value = data[0];

    // ZUERST DIE BEIDEN, DIE INHALT TRAGEN. Ausgegeben werden sie nicht (siehe unten), ausgewertet schon.

    // Der einzige mehrbytige Steuerdienst - das Statusbyte steht dahinter. Die Längenprüfung ist reine
    // Absicherung: eine zweibytige Sequenz entsteht ausschließlich für genau diesen Code. Trifft sie nicht
    // zu, fällt der Dienst absichtlich durch bis zur Ausgabe unten - eine halbe Sequenz will man sehen.
    if (value == U_SYSTEM_STAT_IND && length == 2)
    {
        _systemState.update(data[1]);
        checkChipRestart();
        return;
    }

    // XOR statt Maskierung: die drei Kennbits sind gesetzt und werden damit gelöscht, übrig bleiben genau
    // die Fehlerbits. Verodert, weil der Chip jedes Ereignis nur einmal meldet (siehe _stateErrors).
    if ((value & U_STATE_MASK) == U_STATE_IND)
    {
        _stateErrors |= (uint8_t)(value ^ U_STATE_MASK);
        return;
    }

    // AUF DIE KONSOLE KOMMT NUR, WAS ÜBERRASCHT. Alles hier drunter ist die Antwort auf etwas, das wir
    // selbst veranlasst haben - es zu melden hieße, sich das eigene Echo vorlesen zu lassen:
    //
    //   U_State.ind / U_SystemStat.ind - Antwort auf die Statusabfrage, die JEDE SEKUNDE läuft. Ihr Inhalt
    //     wird gemeldet, und nur wenn er etwas sagt: Fehlerbits über showStateErrors(), der System-Status
    //     über showSystemState() bei Änderung.
    //   U_Configure.ind - Bestätigung einer Einstellung von uns (die Adresse aktiviert die Auto-Quittung).
    //     Der Zustand steht jederzeit über isAutoAcknowledge() bereit.
    //   U_Reset.ind - Antwort auf unser U_Reset.req, sei es aus reset(), aus der Verbindungsaufnahme oder
    //     vom Wachhund des Transmitters. Der EINE Reset, der einen Fehler anzeigt, meldet sich selbst und
    //     mit Begründung: "No L_Data.con ... BCU reset" (siehe loop()). Und kommt ein Reset von der BCU
    //     selbst, wird das ohnehin sichtbar - entweder über den Verbindungsverlust oder, beim NCN, an der
    //     Statuszeile, die dann POWER-UP/SYNC durchläuft (siehe checkChipRestart).
    //   U_StopMode.ind - Antwort auf stopMode(), also ebenfalls unser eigenes Werk.
    //   L_Data.con - die Bestätigung zu unserem eigenen Telegramm. Sie steht hier nur deshalb, weil sie
    //     NEBEN dem Frame ankam statt dahinter: erkennt der Empfänger das Echo, wartet er im Zustand
    //     FrameAck darauf und hängt sie als Flag an das Telegramm (Receiver::processByte). Ohne erkanntes
    //     Echo - Frame verstümmelt, im Resync verloren, Wiederholung dazwischen - landet sie hier. Das ist
    //     eine Aussage über den Empfangsweg, keine über die Bestätigung, und ablesbar ist sie am fehlenden
    //     'A' im gemeldeten Telegramm sowie an getRxInvalidFrames(). Als Konsolenzeile war sie nur Lärm.
    //
    // An der AUSWERTUNG ändert das nichts: die geschieht für Reset, Configure und L_Data.con schon im Tick
    // (resetIndication/configureIndication/Transmitter::confirmed), lange bevor dieser Eintrag hier ankommt.
    //
    // Übrig bleiben damit genau die Fälle, die etwas bedeuten: ein L_Ackn.ind außerhalb des Busmonitors,
    // ein U_FrameEnd.ind (Marker-Modus, den wir nie einschalten), ein U_FrameState.ind außerhalb eines
    // Versands - und unbekannte Bytes.
    if (value == U_RESET_IND) return;
    if (value == U_STOP_MODE_IND) return;
    if ((value & U_CONFIGURE_MASK) == U_CONFIGURE_IND) return;
    if ((value & L_DATA_CON_MASK) == L_DATA_CON) return;

    const char *name = controlServiceName(value);

    // Ein Steuerbyte außerhalb von Table 13 kann die BCU gar nicht senden - kommt trotzdem eines an, wird
    // der Bytestrom an dieser Stelle fehlgedeutet (verpasstes Folgebyte, Telegrammbyte als Sequenzanfang
    // gelesen, falsche Baudrate). Deshalb als Fehler und nicht als Meldung.
    if (!name)
    {
        printError("Unknown control byte %02X", value);
        return;
    }

    // Der Rohwert steht immer dabei: die übrigen dieser Dienste tragen ihre Aussage in den Bits (etwa
    // positiv/negativ im L_Data.con).
    if (length == 2)
        printMessage("%s %02X %02X", name, value, data[1]);
    else
        printMessage("%s %02X", name, value);
}

// Die Konfiguration hängt daran, dass wir die U_Reset.ind SEHEN - und die kann uns entgehen: verworfen im
// Resync, weg bei einem Überlauf des Ringpuffers, oder mitten in einer Sequenz aufgeschlagen. Dann stünde
// die BCU ohne Adresse da und niemand wüsste es.
//
// Der gemeldete Chip-Zustand schließt diese Lücke, ohne dass dafür etwas Zusätzliches gesendet werden muss:
// POWER-UP und SYNC durchläuft der NCN ausschließlich nach einem Reset ("Entered after Reset State",
// S. 30), und abgefragt wird der Status ohnehin jede Sekunde. Sobald er danach wieder NORMAL meldet, wird
// die Konfiguration erneut abgesetzt.
//
// GENAU DESHALB wird sie nicht stur wiederholt: Erkennen ist billiger als Nachplappern. Ein Nachsetzen im
// Sekundentakt wäre Dauerverkehr auf der Host-Strecke für einen Fall, der fast nie eintritt.
//
// STOP zählt mit, obwohl der Zustand vom Aufrufer selbst kommt (stopMode) und die Konfiguration dort nicht
// verlorengeht. Das kostet ein überflüssiges Nachsetzen nach jedem Stop-Zyklus - dieselben Werte noch
// einmal, also folgenlos. Den Fall zu unterscheiden wäre mehr Zustand als der Fehler wert ist.
void DataLinkLayer::checkChipRestart()
{
    if (!_systemState.isValid()) return;

    if (!_systemState.normalMode())
    {
        _chipOutOfNormal = true;
        return;
    }

    if (!_chipOutOfNormal) return;

    _chipOutOfNormal = false;
    markConfigurationPending();
}

// Der System-Status ist NCN-spezifisch; ohne Antwort des Chips ist er nie dirty, die Typprüfung ist also
// nur Vorsorge. Ausgegeben wird ausschließlich bei Änderung - sonst käme die Zeile bei jeder Abfrage.
void DataLinkLayer::showSystemState()
{
    if (_bcuType != BcuType::Ncn5120) return;
    if (!_systemState.dirty()) return;

    printMessage("%s", _systemState.print().c_str());
}

// Die Bits stehen für ein Ereignis SEIT DER LETZTEN ABFRAGE, nicht für einen Dauerzustand: der Chip löscht
// sie mit dem Melden. Genau deshalb werden sie hier gesammelt ausgegeben und dabei zurückgesetzt.
void DataLinkLayer::showStateErrors()
{
    if (!_stateErrors) return;

    std::string message = "TP Error:";
    if (_stateErrors & U_STATE_SLAVE_COLLISION) message += " SC";
    if (_stateErrors & U_STATE_RECEIVE_ERROR) message += " RE";
    if (_stateErrors & U_STATE_TRANSMIT_ERROR) message += " TE";
    if (_stateErrors & U_STATE_PROTOCOL_ERROR) message += " PE";
    if (_stateErrors & U_STATE_TEMPERATURE_WARNING) message += " TW";

    printError("%s", message.c_str());
    _stateErrors = 0;
}

// ---------------------------------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------------------------------

// Aus dem Hauptkontext. Gesetzt wird sofort, wenn die Verbindung steht - sonst holt es die
// Konfigurations-Epoche nach, sobald die BCU antwortet.
//
// Zur Adresse 0: sie steht für "keine" und wird deshalb nicht abgesetzt - der Chip behält damit die
// zuletzt gesetzte Adresse und quittiert weiter. Abschaltbar ist die Adressauswertung nämlich überhaupt
// nur durch einen Reset ("After reset the address evaluation is deactivated again"), es gibt keinen Dienst
// dafür. _autoAcknowledge bleibt darum absichtlich stehen: er beschreibt den Chip, nicht unseren Wunsch.
bool DataLinkLayer::setOwnAddress(uint16_t address)
{
    _ownAddress = address;

    if (!_connected) return true;         // gemerkt, wird beim Verbinden abgesetzt
    if (applyConfiguration()) return true;

    markConfigurationPending();
    return false;
}

bool DataLinkLayer::setRepetitions(uint8_t nack, uint8_t busy)
{
    if (nack > U_REPETITION_COUNTER_MASK || busy > U_REPETITION_COUNTER_MASK) return false;

    _repetitionsNack = nack;
    _repetitionsBusy = busy;

    if (!_connected) return true;
    if (applyConfiguration()) return true;

    markConfigurationPending();
    return false;
}

// Die Konfiguration ist nicht (vollständig) rausgegangen. Statt eines zweiten Merkers wird die vorhandene
// Epoche benutzt: der Hauptkontext setzt seinen Stand einen Schritt zurück, damit processConnectionState()
// beim nächsten loop() erneut anwendet. _configAppliedEpoch hat nur diesen einen Schreiber, und der Wert
// kann nie mit _configEpoch zusammenfallen - egal ob der Tick sie zwischenzeitlich erhöht.
void DataLinkLayer::markConfigurationPending()
{
    _configAppliedEpoch = _configEpoch - 1;
}

// Setzt beides ab, was die BCU nach einem Reset vergessen hat. Nur aus dem Hauptkontext - queueControl()
// hat dort seinen Produzenten.
//
// Die Zähler werden nur bei Abweichung vom Reset-Zustand geschickt (beide 3, in beiden Datenblättern), die
// Adresse nur wenn eine gesetzt ist. Damit bleibt der Regelfall ohne jeden Zusatzverkehr.
bool DataLinkLayer::applyConfiguration()
{
    if (!_connected) return false;

    bool complete = true;

    if (_ownAddress != 0)
    {
        uint8_t high = (uint8_t)(_ownAddress >> 8);
        uint8_t low = (uint8_t)(_ownAddress & 0xFF);
        bool queued;

        if (_bcuType == BcuType::Ncn5120)
        {
            // Das vierte Byte ist ein Dummy, das der NCN verlangt (Table 12: "X (don't care)").
            const uint8_t sequence[] = {U_NCN5120_SET_ADDRESS_REQ, high, low, 0xFF};
            queued = _transmitter.queueControl(sequence, sizeof(sequence));
        }
        else
        {
            const uint8_t sequence[] = {U_TPUART2_SET_ADDRESS_REQ, high, low};
            queued = _transmitter.queueControl(sequence, sizeof(sequence));
        }

        // Mit der Adresse übernimmt der Chip die Quittung - und zwar SOFORT, ohne dass er darauf noch
        // hinweisen müsste: "Sets the physical address of the device and activates the auto-acknowledge
        // function" (NCN5130 p.36) bzw. "If the address is set a complete address evaluation in the
        // TP-UART is activated" (Siemens p.23). Deshalb wird der Zustand HIER gesetzt und nicht erst aus
        // dem U_Configure.ind heraus: das gibt es nur beim NCN. Der TPUART2 hat den Dienst gar nicht
        // (Fig. 25 kennt host-seitig nur Reset-, ProductID-, State- und L_Data.confirm-Indication), dort
        // wäre also nie eine Bestätigung gekommen und wir hätten neben dem Chip weiterquittiert.
        //
        // Gesetzt wird beim Einreihen, nicht beim Absenden. Es bleibt damit ein Fenster, in dem weder der
        // Chip quittiert (er hat die Adresse noch nicht, und aktiv wird sie ohnehin erst "after the KNX
        // bus becomes idle", NCN5130 p.38) noch wir (wir halten uns schon zurück). Ein Telegramm, das
        // genau dann für uns hereinkommt, bleibt unquittiert und wird vom Absender wiederholt - dann
        // greift der Chip. Die Umkehrung wäre schlechter: quittieren wir neben dem Chip, wird auf dem NCN
        // nebenbei ein aktiver BUSY-Modus gelöscht ("BUSY mode is deactivated immediately if the host
        // controller confirms a frame by sending U_Ackn.req", p.35) - und ohne Bestätigung vom Chip
        // (TPUART2) hätte diese Überlappung kein Ende.
        if (!queued) complete = false;
        else _autoAcknowledge = true;
    }

    if (_repetitionsNack != 3 || _repetitionsBusy != 3)
    {
        if (_bcuType == BcuType::Ncn5120)
        {
            uint8_t counters = (uint8_t)((_repetitionsBusy << U_NCN5120_REPETITION_BUSY_SHIFT) | _repetitionsNack);
            const uint8_t sequence[] = {U_NCN5120_SET_REPETITION_REQ, counters, 0x00, 0x00};
            if (!_transmitter.queueControl(sequence, sizeof(sequence))) complete = false;
        }
        else
        {
            uint8_t counters = (uint8_t)((_repetitionsBusy << U_TPUART2_REPETITION_BUSY_SHIFT) | _repetitionsNack);
            const uint8_t sequence[] = {U_TPUART2_SET_REPETITION_REQ, counters};
            if (!_transmitter.queueControl(sequence, sizeof(sequence))) complete = false;
        }
    }

    // Die Spannungsregler - siehe queuePowerControl(). Nur wenn der Aufrufer sie je angefasst hat.
    if (!queuePowerControl()) complete = false;

    return complete;
}

// ---------------------------------------------------------------------------------------------------
// Steuerbefehle an die BCU
// ---------------------------------------------------------------------------------------------------

// Der Chip-Zustand gilt erst als umgeschaltet, wenn das Steuerbyte den Chip auch erreicht hat - bis dahin
// würden wir sonst schon Quittungen vom Bus erwarten, die noch gar nicht kommen können (bzw. umgekehrt
// welche als Steuerbytes fehldeuten). Übernommen wird der neue Zustand deshalb in controlByteSent(),
// abgeleitet aus dem vom Transmitter tatsächlich abgesetzten Steuercode.
//
// Schon im Busmonitor? Dann kein Byte senden und true melden - genau das Verhalten der alten Library.
bool DataLinkLayer::startMonitoring()
{
    if (!_connected) return false;
    if (_busMonitor) return true;

    return _transmitter.queueControl(U_BUSMON_REQ);
}

// Der Reset ist zugleich der einzige Weg aus dem Busmonitor heraus (Datenblatt S. 36, Figure 35) und
// stellt nebenbei den Default-CRC-Modus wieder her.
bool DataLinkLayer::reset()
{
    // Nach einem Reset ist der Bytestrom neu aufgesetzt; was vorher lief, ist kein Bezugspunkt mehr für
    // eine Wiederholung. Das Leeren gleich hier ist unkritisch - beide Kontexte sind derselbe.
    _repetitionFilter.clear();

    return _transmitter.queueControl(U_RESET_REQ);
}

// Der System-Status ist NCN-spezifisch. Und er ist die Vorbedingung dafür, dass überhaupt ein
// U_SystemStat.ind eintreffen kann - der einzige mehrbytige Steuerdienst, den der Receiver kennt.
bool DataLinkLayer::requestState()
{
    if (!_transmitter.queueControl(U_STATE_REQ)) return false;

    if (_bcuType == BcuType::Ncn5120) _transmitter.queueControl(U_SYSTEM_STATE_REQ);
    return true;
}

bool DataLinkLayer::stopMode(bool state)
{
    if (_bcuType != BcuType::Ncn5120) return false; // Dienst gibt es beim TPUART2 nicht

    if (!_transmitter.queueControl(state ? U_STOP_MODE_REQ : U_EXIT_STOP_MODE_REQ)) return false;

    requestState(); // die Auswirkung wird über den Status sichtbar - wie bei powerControl()
    return true;
}

bool DataLinkLayer::busyMode(bool state)
{
    bool queued = _bcuType == BcuType::Ncn5120
                      ? _transmitter.queueControl(state ? U_NCN5120_SET_BUSY_REQ : U_NCN5120_QUIT_BUSY_REQ)
                      : _transmitter.queueControl(state ? U_TPUART2_SET_BUSY_REQ : U_TPUART2_QUIT_BUSY_REQ);

    if (!queued) return false;

    // Der Ablauf der Frist wird HIER aufgezogen bzw. gelöscht, checkBusyMode() setzt ihn nur um.
    _busyModeSince = state ? millis() : 0;
    return true;
}

// Der Busy-Modus endet nach TPUART_BUSY_MODE_MS von selbst - übernommen aus der alten Library, und dort
// steht auch die Begründung: der TPUART2 verlässt ihn NACH 700ms IN HARDWARE, der NCN512x nicht. Ohne
// dieses Nachziehen verhielten sich die beiden Chips also grundverschieden, und auf dem NCN bliebe ein
// einmal gesetzter Busy-Modus für immer stehen, wenn der Aufrufer ihn nicht selbst zurücknimmt.
//
// Läuft aus loop(), also demselben Kontext, aus dem busyMode() gerufen wird - _busyModeSince hat damit
// genau einen Schreiber und braucht weder volatile noch die Vorzeichenregel aus processConnectionState().
// Aus demselben Grund räumt resetIndication() das Feld NICHT mit auf, obwohl ein Reset den Busy-Modus im
// Chip beendet: die Funktion läuft im Tick, und ein zweiter Schreiber wäre der teurere Fehler. Die Folge
// ist ein einzelnes überflüssiges U_QuitBusy.req nach der Frist, das einen bereits gelöschten Zustand
// noch einmal löscht.
//
// DIE ABSAGE SENDET DER CHIP, NICHT WIR: "During this time and when autoacknowledge is active, NCN5130
// rejects the frames whose destination address corresponds to the stored physical address by sending the
// BUSY acknowledge" (S. 35). Solange der Modus läuft, hält sich Receiver::sendAcknowledge() deshalb heraus -
// jedes U_Ackn.req von uns würde den Modus beenden statt ihn zu ergänzen. Die Begründung steht dort.
void DataLinkLayer::reportBusyModeCancelled()
{
    _busyModeCancelled = true;
}

void DataLinkLayer::checkBusyMode()
{
    // Der Tick hat quittiert - damit ist der Modus im Chip weg, ganz gleich wie lange die Frist noch
    // gelaufen wäre. Unser Merker zieht nach, statt einen Zustand zu behaupten, den der Chip nicht mehr
    // hat: sonst schwiege sendAcknowledge() weiter, sobald die Auto-Quittung wieder anläuft (sie kommt
    // nach jedem Reset neu), und aus der Absage würde vollständige Stille.
    if (_busyModeCancelled)
    {
        _busyModeCancelled = false;
        _busyModeSince = 0;
    }

    if (_busyModeSince == 0) return;
    if ((uint32_t)(millis() - _busyModeSince) < TPUART_BUSY_MODE_MS) return;

    busyMode(false); // löscht _busyModeSince - und versucht es beim nächsten loop() erneut, falls die
                     // Steuer-Warteschlange gerade voll war
}

// Schreibzugriff auf ACR0 - eine 2-Byte-Sequenz, die ununterbrochen rausgehen muss, deshalb als Gruppe.
// Flag-Werte wie in der alten Library: im ausgeschalteten Zustand bleiben Taktausgang und Strombegrenzung
// aktiv, nur die beiden Regler VCC2 und 20V werden abgeschaltet.
bool DataLinkLayer::powerControl(bool state)
{
    if (_bcuType != BcuType::Ncn5120) return false; // interne Register hat nur die NCN512x-Reihe

    _powerControl = state ? PowerControl::On : PowerControl::Off;

    if (!queuePowerControl()) return false;

    requestState(); // die Auswirkung wird über den Status sichtbar
    return true;
}

// Auch das ist Konfiguration, die ein Reset kassiert - deshalb steht sie hier und wird von
// applyConfiguration() mitgesetzt. ACR0 trägt laut Datenblatt (Table 16) den Reset-Wert 0111 0100, also
// beide Regler AN, und die RESET-Zustandsbeschreibung nennt ausdrücklich beide Auslöser: "Entered after
// Power On Reset (POR) or in response to a U_Reset.req service issued by the host controller. In this state
// NCN5130 gets initialized" (S. 30). Ein powerControl(false) wäre nach jedem Reset also still wieder
// aufgehoben - und Resets schickt diese Library selbst (Wachhund des Transmitters, reset() aus der
// Anwendung).
//
// Sollte ACR0 einen U_Reset.req wider Erwarten doch überleben, kostet das Wiederholen nichts: es schreibt
// denselben Wert. Die Annahme ist damit in beide Richtungen unschädlich - anders als das Weglassen.
bool DataLinkLayer::queuePowerControl()
{
    if (_powerControl == PowerControl::Unset) return true; // nie gesetzt - dann bleibt der Chip, wie er ist

    uint8_t value = ACR0_FLAG_XCLKEN | ACR0_FLAG_V20VCLIMIT;
    if (_powerControl == PowerControl::On) value |= ACR0_FLAG_DC2EN | ACR0_FLAG_V20VEN;

    const uint8_t sequence[] = {U_INT_REG_WR_REQ_ACR0, value};
    return _transmitter.queueControl(sequence, sizeof(sequence));
}

// ---------------------------------------------------------------------------------------------------
// Meldungen
// ---------------------------------------------------------------------------------------------------

// 128 Byte statt der 1024 der alten Library: der längste Text hier ist die System-Status-Zeile mit gut 60
// Zeichen. Der Puffer liegt auf dem Stack, und der ist auf dem RP2040/ESP32 knapper als auf dem PC.
void DataLinkLayer::printMessage(const char *format, ...)
{
    if (!_messageCallback) return;

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    _messageCallback(buffer, false);
}

void DataLinkLayer::printError(const char *format, ...)
{
    if (!_messageCallback) return;

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    _messageCallback(buffer, true);
}

void DataLinkLayer::registerMessage(MessageCallback callback)
{
    _messageCallback = callback;
}

// ---------------------------------------------------------------------------------------------------
// Zustand und Durchreichen an die beiden Hälften
// ---------------------------------------------------------------------------------------------------

bool DataLinkLayer::isConnected() const
{
    return _connected;
}

uint32_t DataLinkLayer::connectedBaudRate() const
{
    return _connectedBaudRate;
}

BcuType DataLinkLayer::bcuType() const
{
    return _bcuType;
}

BcuState DataLinkLayer::bcuState() const
{
    if (!_initialized) return BcuState::Uninitialized;
    if (_connected) return BcuState::Connected;

    return _everConnected ? BcuState::Disconnected : BcuState::Uninitialized;
}

const char *DataLinkLayer::bcuStateName() const
{
    switch (bcuState())
    {
        case BcuState::Connected:
            return "connected";
        case BcuState::Disconnected:
            return "disconnected";
        default:
            return "uninitialized";
    }
}

bool DataLinkLayer::isAutoAcknowledge() const
{
    return _autoAcknowledge;
}

bool DataLinkLayer::isBusyMode() const
{
    return _busyModeSince != 0;
}

bool DataLinkLayer::isBusMonitor() const
{
    return _busMonitor;
}

// --- KOMPAT: alte Namen -------------------------------------------------------------------------------

bool DataLinkLayer::isMonitoring() const
{
    return isBusMonitor();
}

const char *DataLinkLayer::getBcuStateInfo() const
{
    return bcuStateName();
}

uint16_t DataLinkLayer::ownAddress() const
{
    return _ownAddress;
}

// Unkritisch: gesetzt wird aus dem Hauptkontext, gerufen wird ausschließlich aus loop() - also derselbe
// Kontext, keine Nebenläufigkeit. Genau dafür ist der Ringpuffer da.
void DataLinkLayer::registerFrameCallback(FrameCallback callback)
{
    _frameCallbacks.push_back(callback);
}

// KOMPAT: alter Name.
void DataLinkLayer::registerReceivedFrame(FrameCallback callback)
{
    registerFrameCallback(callback);
}

// Jeder registrierte Callback bekommt dasselbe Frame, in der Reihenfolge der Registrierung - und ALLE
// sehen die Änderungen der vorherigen (setFiltered() etwa). Das ist Absicht und war in der alten Library
// genauso: der Empfangspfad des Stacks und eine Diagnoseausgabe daneben sind zwei Zuhörer, kein Ersatz
// füreinander.
void DataLinkLayer::deliverFrame(Frame &frame)
{
    for (FrameCallback &callback : _frameCallbacks)
        if (callback) callback(frame);
}

void DataLinkLayer::registerCheckAcknowledge(AcknowledgeCallback callback)
{
    _acknowledgeCallback = callback;
}

// Aus dem TICK, mitten im laufenden Frame. Ohne registrierte Entscheidung wird NICHT quittiert - genauso
// wie die alte Library, deren checkAcknowledge() ohne Callback ACK_None liefert. Alles zu quittieren wäre
// am echten Bus falsch: das Gerät gäbe damit vor, unter jeder Zieladresse erreichbar zu sein.
AckType DataLinkLayer::checkAcknowledge(uint16_t destination, bool isGroupAddress)
{
    if (!_acknowledgeCallback) return AckType::None;

    return _acknowledgeCallback(destination, isGroupAddress);
}

// Aus dem Tick. Beides zusammen, weil beides dasselbe Ereignis beschreibt: der Merker beantwortet "ist
// gerade etwas passiert", der Zähler "wie oft insgesamt".
void DataLinkLayer::reportInterfaceOverflow()
{
    _interfaceOverflow = true;
    _statistics.incrementInterfaceOverflows();
}

void DataLinkLayer::reportRxQueueOverflow()
{
    _rxQueueOverflow = true;
    _statistics.incrementRxQueueOverflows();
}

void DataLinkLayer::reportControlOverflow()
{
    _ctrlQueueOverflow = true;
    _statistics.incrementCtrlQueueOverflows();
}

bool DataLinkLayer::queueOverflow()
{
    if (!_rxQueueOverflow) return false;

    _rxQueueOverflow = false;
    return true;
}

bool DataLinkLayer::interfaceOverflow()
{
    if (!_interfaceOverflow) return false;

    _interfaceOverflow = false;
    return true;
}

bool DataLinkLayer::controlOverflow()
{
    if (!_ctrlQueueOverflow) return false;

    _ctrlQueueOverflow = false;
    return true;
}

Receiver &DataLinkLayer::getReceiver()
{
    return _receiver;
}

Transmitter &DataLinkLayer::getTransmitter()
{
    return _transmitter;
}

Statistics &DataLinkLayer::getStatistics()
{
    return _statistics;
}

RepetitionFilter &DataLinkLayer::getRepetitionFilter()
{
    return _repetitionFilter;
}

SystemState &DataLinkLayer::getSystemState()
{
    return _systemState;
}

bool DataLinkLayer::sendFrame(const uint8_t *data, size_t length)
{
    return _transmitter.sendFrame(data, length);
}

// KOMPAT, siehe Header. Die Länge geht um eins gekürzt weiter: das Frame bringt die Prüfsumme als letztes
// Byte mit, sendFrame() rechnet sie selbst - dieselbe CRC-8 über dieselben Bytes, also derselbe Wert.
bool DataLinkLayer::pushTransmitQueue(Frame *frame)
{
    if (frame == nullptr) return false;
    if (frame->length() < 2) return false;

    if (!sendFrame(frame->buffer(), frame->length() - 1)) return false;

    delete frame; // übernommen - wie in der alten Library, wo die Warteschlange den Besitz bekam
    return true;
}

} // namespace TPUart
