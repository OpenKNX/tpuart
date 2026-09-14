#include "TPUart/DataLinkLayer.h"

// Zeigt beim Bau im Fremdprojekt, dass diese Version gelinkt wird.
#pragma message "TPUART v2"

#include <stdarg.h>
#include <stdio.h>

#include <Arduino.h>

#include <string>

// Keine Sperren: geteilter Zustand ist über Besitz und Reihenfolge geregelt (SPSC-Warteschlangen mit Kopf
// zuletzt, Callbacks vor dem Start des Ticks gesetzt). Auf dem RP2040 läuft tick() im Interrupt.

namespace
{

constexpr uint32_t TPUART2_BAUD_RATES[] = {19200};
constexpr uint32_t NCN5120_BAUD_RATES[] = {19200, 38400};

// Baudraten-Kandidaten je Chip.
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

DataLinkLayer::~DataLinkLayer()
{
    Timer::instance().remove(*this);
}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft aus dem Timer
// ---------------------------------------------------------------------------------------------------

void DataLinkLayer::tick()
{
    if (_interface == nullptr) return; // ohne Interface gibt es nichts zu tun (siehe KOMPAT-Konstruktor)
    if (_bcuState == BcuState::Uninitialized) return; // begin() wurde noch nicht aufgerufen

    // Taktmessung vor allen weiteren Abbrüchen - gemessen wird der Antrieb, nicht die Arbeit.
    uint32_t tickNow = micros();
    uint32_t sinceLast = _tickLastUs == 0 ? 0 : tickNow - _tickLastUs;
    _tickLastUs = tickNow;

    _statistics.recordTick();

    if (sinceLast > TPUART_TICK_DEFERRED_US) _statistics.recordTickDeferred(sinceLast);

    // Während der Suche gehört das Interface dem Hauptkontext.
    BcuState state = _bcuState;
    if (state == BcuState::Searching) return;

    if (_connectionTimeouts != _connectionTimeoutsSeen)
    {
        _connectionTimeoutsSeen = _connectionTimeouts;
        if (state == BcuState::Connected)
        {
            connectionLost();
            return;
        }
    }

    if (state == BcuState::Disconnected)
    {
        reconnect();
        return;
    }

    _receiver.process();
    _transmitter.process();

    // Nur der volle Durchlauf, einschließlich Quittungs-Callback.
    _statistics.updateTickDurationMaxUs(micros() - tickNow);
}

void DataLinkLayer::begin(BcuType bcuType)
{
    // Zuerst austragen - ein laufender reconnect() schriebe sonst noch Connected.
    Timer::instance().remove(*this);

    _bcuType = bcuType;
    _connectReported = false;
    _connectionTimeoutsSeen = _connectionTimeouts;
    _connectionLossesSeen = _connectionLosses;
    _detectAwaitingResponse = false;
    _detectCandidateIndex = 0;
    _detectNextAttemptAt = millis(); // sofort beim nächsten loop() fällig

    _bcuChip = BcuChip::Unknown;
    _ncnRevision = 0;

    // Auch nach end(): die Suche öffnet das geschlossene Interface wieder.
    _bcuState = BcuState::Searching;

    // Die Zeit seit end() ist keine Lücke im Antrieb.
    _tickLastUs = 0;

    // Der Timer wird immer benutzt. Schlägt das Eintragen fehl, meldet checkTickRate() den fehlenden Antrieb.
    Timer::instance().add(*this);
}

// KOMPAT: die alte Library bekam das Interface hier statt im Konstruktor.
void DataLinkLayer::begin(BcuType bcuType, Interface::Abstract *interface)
{
    _interface = interface;
    begin(bcuType);
}

// Timer muss laufen UND diese Instanz eingetragen sein.
bool DataLinkLayer::usesTimer() const
{
    return Timer::instance().running() && Timer::instance().contains(*this);
}

// KOMPAT: schließt das Interface, gibt es aber nicht frei.
void DataLinkLayer::end()
{
    // Zuerst austragen: danach läuft kein tick() mehr, der ins geschlossene Interface griffe.
    Timer::instance().remove(*this);

    _bcuState = BcuState::Uninitialized;

    if (_interface != nullptr) _interface->end();
}

// KOMPAT: hier wird NIE getickt - der Hauptloop ist kein Antrieb für tick() (Festlegung des Anwenders).
// Der Antrieb ist der Timer oder Timer::trigger().
void DataLinkLayer::process()
{
    loop();
}

// Aus loop(), nicht blockierend, verteilt über mehrere Aufrufe. Konfiguriert das Interface je Kandidat neu -
// deshalb nicht im Tick.
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

    // Baudratenwechsel nur mit end()/begin() - der RP2040 beanspruchte sonst einen zweiten DMA-Kanal.
    _interface->end();
    _interface->begin(baud);

    if (!_interface->write((char)U_RESET_REQ)) return;

    _detectAwaitingResponse = true;
    _detectRequestSentAt = millis();
}

// Aus dem Tick in Disconnected: wie die Suche, aber mit bekannter Baudrate und ohne das Interface anzufassen.
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

// Nur während _detectAwaitingResponse: das erste Byte außer einer führenden 0 entscheidet sofort.
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

// Aus der Suche (Hauptkontext) oder aus reconnect() (Tick). Der Zustand kommt zuletzt.
void DataLinkLayer::connectDetected()
{
    const uint32_t *rates;
    size_t count;
    candidateBaudRates(_bcuType, rates, count);

    // Die U_Reset.ind ist ein Reset wie jeder andere.
    resetIndication();

    _connectedBaudRate = rates[_detectCandidateIndex % count];
    _detectAwaitingResponse = false;
    _lastReceivedAt = millis(); // sonst gälte die Verbindung sofort als still
    _receiver._awaitRegisterValue = false; // nach dem Reset steht keine Registerantwort mehr aus

    if (_bcuState == BcuState::Searching && _bcuType == BcuType::Ncn5120)
    {
        _registerRead = RegisterRead::Start;
        _bcuState = BcuState::Identifying;
    }
    else
    {
        _bcuState = BcuState::Connected;
    }
}

// ---------------------------------------------------------------------------------------------------
// Lesezugriff auf die internen Register
// ---------------------------------------------------------------------------------------------------

constexpr uint32_t NCN_STOP_MODE_TIMEOUT_MS = 50;
constexpr uint32_t NCN_STOP_MODE_MAX_MS = 500;
constexpr uint32_t NCN_REG_READ_TIMEOUT_MS = 20;

void DataLinkLayer::processRegisterRead()
{
    switch (_registerRead)
    {
        case RegisterRead::Start:
            if (!_transmitter.queueControl(U_STOP_MODE_REQ)) return;

            _stopModeReached = false;
            _registerRead = RegisterRead::AwaitStop;
            _registerReadSentAt = millis();
            return;

        case RegisterRead::AwaitStop:
        {
            // Frist ab Busruhe, nach oben begrenzt.
            uint32_t since = _registerReadSentAt;
            uint32_t last = _lastReceivedAt;
            if ((int32_t)(last - since) > 0) since = last;
            uint32_t now = millis();

            if (_stopModeReached)
            {
                if (!readRegister(NCN_REG_WD)) endRegisterRead();
            }
            else if ((int32_t)(now - since) >= (int32_t)NCN_STOP_MODE_TIMEOUT_MS ||
                     (uint32_t)(now - _registerReadSentAt) >= NCN_STOP_MODE_MAX_MS)
            {
                endRegisterRead();
            }
            return;
        }

        case RegisterRead::AwaitValue:
            if (_registerValueCount != _registerValueSeen)
            {
                _registerValueSeen = _registerValueCount;

                if (!readNextRegister(_registerValue)) endRegisterRead();
            }
            else if ((uint32_t)(millis() - _registerReadSentAt) >= NCN_REG_READ_TIMEOUT_MS)
            {
                endRegisterRead();
            }
            return;
    }
}

bool DataLinkLayer::readRegister(uint8_t reg)
{
    if (reg > U_INT_REG_RD_ADDRESS_MASK) return false;
    if (!_transmitter.queueControl((uint8_t)(U_INT_REG_RD_REQ | reg))) return false;

    _readRegister = reg;
    _registerValueSeen = _registerValueCount;
    _registerRead = RegisterRead::AwaitValue;
    _registerReadSentAt = millis();
    return true;
}

bool DataLinkLayer::readNextRegister(uint8_t value)
{
    switch (_readRegister)
    {
        case NCN_REG_WD:
            if (value != NCN_WD_RESET) return false; // Anker: nach dem Reset muss der Resetwert stehen
            return readRegister(NCN_REG_ACR1);

        case NCN_REG_ACR1:
            if (value == NCN_ACR1_RESET_5120)
            {
                _bcuChip = BcuChip::Ncn5120;
                return false;
            }
            if (value != NCN_ACR1_RESET_5121_5130) return false;
            return readRegister(NCN_REG_REVID); // RevID nur auf 5121/5130

        case NCN_REG_REVID:
            switch (value & NCN_REVID_PART_MASK)
            {
                case NCN_PART_5130:
                    _bcuChip = BcuChip::Ncn5130;
                    break;
                case NCN_PART_5121:
                    _bcuChip = BcuChip::Ncn5121;
                    break;
                default:
                    return false;
            }
            _ncnRevision = value >> NCN_REVID_REVISION_SHIFT;
            return false;

        default:
            return false;
    }
}

// Ausstieg per Reset: U_ExitStopMode.req wird ignoriert, solange der Stop noch nicht erreicht ist.
void DataLinkLayer::endRegisterRead()
{
    if (reset()) _bcuState = BcuState::Connected;
}

bool DataLinkLayer::registerReadActive() const
{
    return _bcuState == BcuState::Identifying;
}

void DataLinkLayer::registerValueReceived(uint8_t value)
{
    _registerValue = value;
    _registerValueCount = _registerValueCount + 1;
}

// Nächster Kandidat; nach dem letzten erst nach TPUART_DETECT_RETRY_INTERVAL_MS wieder von vorn.
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

// Ein Steuercode hat den Chip erreicht (Transmitter).
void DataLinkLayer::controlByteSent(uint8_t code)
{
    if (code == U_BUSMON_REQ)
    {
        if (_bcuState == BcuState::Connected) _bcuState = BcuState::BusMonitor;

        // Ein laufendes Telegramm wird abgebrochen, nicht wiederholt. Die Warteschlange räumt stageNextTelegram().
        _transmitter.abort();

        // Busmonitor beendet den Busy-Modus. Gemeldet statt gelöscht - _busyModeSince gehört dem Hauptkontext.
        reportBusyModeCancelled();

        // Im Busmonitor quittiert der Chip nichts.
        _autoAcknowledge = false;
    }

    if ((code & ~U_INT_REG_RD_ADDRESS_MASK) == U_INT_REG_RD_REQ) _receiver._awaitRegisterValue = true;

    if (code == U_RESET_REQ)
    {
        _receiver._awaitRegisterValue = false;

        // Beim Verlassen des Busmonitors die Überwachung neu aufziehen - _lastReceivedAt ist dort beliebig alt.
        if (_bcuState == BcuState::BusMonitor)
        {
            _lastReceivedAt = millis();
            _bcuState = BcuState::Connected;
        }
    }

    // Nur diese beiden Codes ändern die Bedeutung des Bytestroms.
    if (code == U_BUSMON_REQ || code == U_RESET_REQ) _receiver.forceResync();
}

// Die BCU hat sich zurückgesetzt, egal durch wen: Sendepuffer leer, ein offenes Telegramm beginnt von vorn.
void DataLinkLayer::resetIndication()
{
    if (_bcuState == BcuState::BusMonitor) _bcuState = BcuState::Connected; // auch ohne eigenes U_Reset.req
    _autoAcknowledge = false; // "After reset the address evaluation is deactivated again"

    // Adresse und Wiederholungszähler sind damit im Chip weg - der Hauptkontext setzt sie neu ab.
    _configEpoch = _configEpoch + 1;

    _transmitter.restart();
}

// Der Chip meldet seine Betriebsarten - maßgeblich, auch wenn es dem Flag widerspricht.
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

    // Die Sendewarteschlange gehört dem Hauptkontext: freigeben, räumen, vorlegen.
    _transmitter.stageNextTelegram();

    if (_bcuState == BcuState::Searching)
        searchBaudRate();
    else
        processConnectionState();

    // Auch ohne Verbindung, damit die Buslast auf null fällt.
    _statistics.sampleBusLoad();

    checkBusyMode();
    checkTickRate();

    // Der Wachhund im Tick kann nicht selbst melden.
    if (_transmitter.confirmTimeout())
        printError("No L_Data.con for %u ms - BCU reset, telegram will be sent again", (unsigned)TPUART_TX_CONFIRM_TIMEOUT_MS);

    showSystemState();
    showStateErrors();
}

// Meldet eine erreichte Taktrate unter der Busgrenze - einmal je Störung, gilt für jeden Antrieb.
void DataLinkLayer::checkTickRate()
{
    // Vor begin() zählt nichts.
    if (_bcuState == BcuState::Uninitialized) return;

    uint32_t now = millis();

    // Der erste Durchlauf legt nur den Bezugspunkt fest.
    if (_tickRateCheckedAt == 0)
    {
        _tickRateCheckedAt = now;
        _tickRateLastTicks = _statistics.getTicks();
        return;
    }

    uint32_t elapsed = now - _tickRateCheckedAt;
    if (elapsed < TPUART_TICK_RATE_WINDOW_MS) return;

    uint32_t ticks = _statistics.getTicks();
    uint32_t delta = ticks - _tickRateLastTicks;

    _tickRateCheckedAt = now;
    _tickRateLastTicks = ticks;

    // delta 0 ausdrücklich behandeln (Division durch null); 64 Bit, weil elapsed * 1000 sonst überläuft.
    uint32_t averageUs = delta == 0 ? 0xFFFFFFFF : (uint32_t)(((uint64_t)elapsed * 1000) / delta);

    if (averageUs <= TPUART_TICK_DEFERRED_US)
    {
        _tickRateReported = false; // erholt - der Melder wird wieder scharf
        return;
    }

    if (_tickRateReported) return;
    _tickRateReported = true;

    if (delta == 0)
        printError("Tick stopped - no tick in %u ms, bus needs one below %u us", (unsigned)elapsed, (unsigned)TPUART_TICK_DEFERRED_US);
    else
        printError("Tick too slow - %u us average (%u/s), bus needs below %u us", (unsigned)averageUs, (unsigned)(((uint64_t)delta * 1000) / elapsed), (unsigned)TPUART_TICK_DEFERRED_US);
}

// Aus loop(): Statusabfrage und Verbindungsverlust. Jedes empfangene Byte ist ein Lebenszeichen; die
// Abfrage sorgt dafür, dass auch auf einem stillen Bus eines kommt.
void DataLinkLayer::processConnectionState()
{
    // Den Verlust schreibt der Tick, gemeldet wird er hier.
    if (_connectionLosses != _connectionLossesSeen)
    {
        _connectionLossesSeen = _connectionLosses;
        _connectReported = false;
        _statistics.incrementConnectionLosses();
        printError("BCU disconnected - no byte received for %u ms", (unsigned)TPUART_CONNECTION_TIMEOUT_MS);
    }

    if (!isConnected()) return; // die Erkennung läuft, die überwacht sich selbst

    if (!_connectReported)
    {
        _connectReported = true;
        printMessage("BCU connected (%u baud)", (unsigned)_connectedBaudRate);
    }

    if (_bcuState == BcuState::Identifying)
    {
        processRegisterRead();
        return;
    }

    // Im Busmonitor ruht die Überwachung: U_State.req bleibt dort unbeantwortet, und der Reconnect-Reset
    // beendete den Monitor. Eine dort ausfallende BCU bleibt bis zum Verlassen unbemerkt.
    if (_bcuState == BcuState::BusMonitor) return;

    // ERST den Zeitstempel lesen, DANN die Uhr, und vorzeichenbehaftet vergleichen - sonst ergibt ein
    // dazwischen empfangenes Byte eine negative Differenz und einen falschen Verbindungsabbruch.
    uint32_t last = _lastReceivedAt;
    uint32_t now = millis();

    if ((int32_t)(now - last) >= (int32_t)TPUART_CONNECTION_TIMEOUT_MS)
    {
        if (_connectionTimeouts == _connectionTimeoutsSeen) _connectionTimeouts = _connectionTimeouts + 1;
        return;
    }

    // Neue Epoche nach einem Reset: Konfiguration erneut absetzen. Nachgezogen wird erst, wenn alles passte.
    if (_configAppliedEpoch != _configEpoch)
    {
        uint32_t epoch = _configEpoch; // erst lesen, dann anwenden: der Tick kann sie dazwischen erhöhen
        if (applyConfiguration()) _configAppliedEpoch = epoch;
    }

    if ((uint32_t)(now - _lastStateRequestAt) < TPUART_STATE_INTERVAL_MS) return;

    _lastStateRequestAt = now;
    requestState();
}

// LÄUFT IM TICK - aus Connected schreibt nur er (siehe _bcuState). Der Reconnect nutzt die bekannte Baudrate.
void DataLinkLayer::connectionLost()
{
    _detectAwaitingResponse = false;
    _detectNextAttemptAt = millis();
    _bcuState = BcuState::Disconnected;
    _connectionLosses = _connectionLosses + 1;
}

// Steuerbytes gehen nicht nach außen, nur als Meldung.
void DataLinkLayer::handleControlEntry(const uint8_t *data, size_t length)
{
    uint8_t value = data[0];

    // Einziger zweibytiger Steuerdienst. Eine halbe Sequenz fällt absichtlich durch zur Ausgabe.
    if (value == U_SYSTEM_STAT_IND && length == 2)
    {
        _systemState.update(data[1]);
        checkChipRestart();
        return;
    }

    // XOR löscht die drei Kennbits, übrig bleiben die Fehlerbits.
    if ((value & U_STATE_MASK) == U_STATE_IND)
    {
        uint8_t errors = (uint8_t)(value ^ U_STATE_MASK);

        if (errors & U_STATE_SLAVE_COLLISION) _statistics.incrementChipSlaveCollisions();
        if (errors & U_STATE_RECEIVE_ERROR) _statistics.incrementChipReceiveErrors();
        if (errors & U_STATE_TRANSMIT_ERROR) _statistics.incrementChipTransmitErrors();
        if (errors & U_STATE_PROTOCOL_ERROR) _statistics.incrementChipProtocolErrors();
        if (errors & U_STATE_TEMPERATURE_WARNING) _statistics.incrementChipTemperatureWarnings();

        _stateErrors |= errors;
        return;
    }

    // Antworten auf eigene Anforderungen werden nicht gemeldet. Ausgewertet sind Reset, Configure und
    // L_Data.con schon im Tick.
    if (value == U_RESET_IND) return;
    if (value == U_STOP_MODE_IND)
    {
        _stopModeReached = true;
        return;
    }
    if ((value & U_CONFIGURE_MASK) == U_CONFIGURE_IND) return;
    if ((value & L_DATA_CON_MASK) == L_DATA_CON) return;

    const char *name = controlServiceName(value);

    // Unbekanntes Steuerbyte: der Bytestrom wird fehlgedeutet - deshalb als Fehler.
    if (!name)
    {
        printError("Unknown control byte %02X", value);
        return;
    }

    if (length == 2)
        printMessage("%s %02X %02X", name, value, data[1]);
    else
        printMessage("%s %02X", name, value);
}

// Erkennt einen Reset, dessen U_Reset.ind verloren ging: der NCN durchläuft POWER-UP/SYNC nur nach einem Reset.
// Kehrt er nach NORMAL zurück, wird die Konfiguration erneut abgesetzt (STOP zählt mit, das ist folgenlos).
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

// Nur bei Änderung ausgeben.
void DataLinkLayer::showSystemState()
{
    if (_bcuType != BcuType::Ncn5120) return;
    if (!_systemState.dirty()) return;

    printMessage("%s", _systemState.print().c_str());
}

// Die Bits sind Ereignisse seit der letzten Abfrage - gesammelt ausgeben und zurücksetzen.
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

bool DataLinkLayer::setOwnAddress(uint16_t address)
{
    _ownAddress = address;

    if (!isConnected()) return true;         // gemerkt, wird beim Verbinden abgesetzt
    if (applyConfiguration()) return true;

    markConfigurationPending();
    return false;
}

bool DataLinkLayer::setRepetitions(uint8_t nack, uint8_t busy)
{
    if (nack > U_REPETITION_COUNTER_MASK || busy > U_REPETITION_COUNTER_MASK) return false;

    _repetitionsNack = nack;
    _repetitionsBusy = busy;

    if (!isConnected()) return true;
    if (applyConfiguration()) return true;

    markConfigurationPending();
    return false;
}

// Setzt die angewandte Epoche einen Schritt zurück - sie kann nie mit _configEpoch zusammenfallen.
void DataLinkLayer::markConfigurationPending()
{
    _configAppliedEpoch = _configEpoch - 1;
}

// Setzt ab, was die BCU nach einem Reset vergessen hat. Zähler nur bei Abweichung von 3, Adresse nur wenn gesetzt.
bool DataLinkLayer::applyConfiguration()
{
    if (!isConnected()) return false;

    // Im Busmonitor ignoriert der Chip die Dienste - Epoche offen lassen, der Reset beim Verlassen löscht ohnehin.
    if (isBusMonitor()) return false;
    if (registerReadActive()) return false;

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

        // Die Adresse aktiviert die Auto-Quittung sofort - hier gesetzt, weil der TPUART2 kein U_Configure.ind kennt.
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

    powerControl(_powerControl);

    return complete;
}

// ---------------------------------------------------------------------------------------------------
// Steuerbefehle an die BCU
// ---------------------------------------------------------------------------------------------------

// Der Zustand wechselt erst in controlByteSent(), wenn das Byte beim Chip ist.
bool DataLinkLayer::startMonitoring()
{
    if (!isConnected()) return false;
    if (isBusMonitor()) return true;
    if (_bcuState == BcuState::Identifying) return false; // der Reset am Ende beendete ihn gleich wieder

    return _transmitter.queueControl(U_BUSMON_REQ);
}

// Einziger Weg aus dem Busmonitor; stellt nebenbei den Default-CRC-Modus her.
bool DataLinkLayer::reset()
{
    // Nach einem Reset ist der Bytestrom neu - kein Bezugspunkt mehr für Wiederholungen.
    _repetitionFilter.clear();

    return _transmitter.queueControl(U_RESET_REQ);
}

// Im Busmonitor bliebe die Abfrage unbeantwortet.
bool DataLinkLayer::requestState()
{
    if (isBusMonitor()) return false;
    if (registerReadActive()) return false; // die Antwort landete sonst als Registerwert

    if (!_transmitter.queueControl(U_STATE_REQ)) return false;

    if (_bcuType == BcuType::Ncn5120) _transmitter.queueControl(U_SYSTEM_STATE_REQ);
    return true;
}

bool DataLinkLayer::stopMode(bool state)
{
    if (_bcuType != BcuType::Ncn5120) return false; // Dienst gibt es beim TPUART2 nicht

    if (!_transmitter.queueControl(state ? U_STOP_MODE_REQ : U_EXIT_STOP_MODE_REQ)) return false;

    requestState(); // die Auswirkung wird über den Status sichtbar
    return true;
}

// Im Busmonitor wirkungslos - _busyModeSince behauptete sonst einen Zustand, den es nicht gibt.
bool DataLinkLayer::busyMode(bool state)
{
    if (isBusMonitor()) return false;

    bool queued = _bcuType == BcuType::Ncn5120
                      ? _transmitter.queueControl(state ? U_NCN5120_SET_BUSY_REQ : U_NCN5120_QUIT_BUSY_REQ)
                      : _transmitter.queueControl(state ? U_TPUART2_SET_BUSY_REQ : U_TPUART2_QUIT_BUSY_REQ);

    if (!queued) return false;

    _busyModeSince = state ? millis() : 0;
    return true;
}

// Aus dem Tick: ein U_Ackn.req hat den Busy-Modus beendet. Über ein Flag, damit _busyModeSince einen Schreiber behält.
void DataLinkLayer::reportBusyModeCancelled()
{
    _busyModeCancelled = true;
}

// Nimmt den Busy-Modus nach TPUART_BUSY_MODE_MS zurück. Nur Hauptkontext.
void DataLinkLayer::checkBusyMode()
{
    // Quittiert: der Modus ist im Chip schon weg.
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

bool DataLinkLayer::powerControl(bool state)
{
    if (_bcuType != BcuType::Ncn5120) return false; // interne Register hat nur die NCN512x-Reihe

    _powerControl = state;

    if (!isConnected()) return true; // gemerkt, geht beim Verbinden raus

    uint8_t acr0 = NCN_ACR0_FLAG_XCLKEN;
    if (_bcuChip != BcuChip::Ncn5120) acr0 |= NCN_ACR0_FLAG_V20VCLIMIT; // auf dem 5120 reserviert
    if (state) acr0 |= NCN_ACR0_FLAG_DC2EN | NCN_ACR0_FLAG_V20VEN;

    return writeRegister(NCN_REG_ACR0, acr0);
}

bool DataLinkLayer::writeRegister(uint8_t reg, uint8_t value)
{
    if (_bcuType != BcuType::Ncn5120) return false;
    if (reg > U_INT_REG_WR_ADDRESS_MASK) return false; // liefe sonst in einen benachbarten Dienst über

    const uint8_t sequence[] = {(uint8_t)(U_INT_REG_WR_REQ | reg), value};
    return _transmitter.queueControl(sequence, sizeof(sequence));
}

// ---------------------------------------------------------------------------------------------------
// Meldungen
// ---------------------------------------------------------------------------------------------------

// 128 Byte auf dem Stack reichen für die längste Meldung.
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
    BcuState state = _bcuState;
    return state == BcuState::Identifying || state == BcuState::Connected || state == BcuState::BusMonitor;
}

uint32_t DataLinkLayer::connectedBaudRate() const
{
    return _connectedBaudRate;
}

BcuType DataLinkLayer::bcuType() const
{
    return _bcuType;
}

BcuChip DataLinkLayer::bcuChip() const
{
    BcuState state = _bcuState;
    if (state == BcuState::Uninitialized || state == BcuState::Searching) return BcuChip::Unknown;
    if (_bcuType == BcuType::Tpuart2) return BcuChip::Tpuart2;

    return _bcuChip;
}

uint8_t DataLinkLayer::ncnRevision() const
{
    return _ncnRevision;
}

BcuState DataLinkLayer::bcuState() const
{
    return _bcuState;
}

const char *DataLinkLayer::bcuStateName() const
{
    switch (bcuState())
    {
        case BcuState::Searching:
            return "Searching";
        case BcuState::Identifying:
            return "Identifying";
        case BcuState::Connected:
            return "Connected";
        case BcuState::BusMonitor:
            return "BusMonitor";
        case BcuState::Disconnected:
            return "Disconnected";
        default:
            return "Uninitialized";
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
    return _bcuState == BcuState::BusMonitor;
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

void DataLinkLayer::registerFrameCallback(FrameCallback callback)
{
    _frameCallbacks.push_back(callback);
}

// KOMPAT: alter Name.
void DataLinkLayer::registerReceivedFrame(FrameCallback callback)
{
    registerFrameCallback(callback);
}

// Jeder Callback bekommt dasselbe Frame und sieht die Änderungen der vorherigen (setFiltered()).
void DataLinkLayer::deliverFrame(Frame &frame)
{
    for (FrameCallback &callback : _frameCallbacks)
        if (callback) callback(frame);
}

void DataLinkLayer::registerCheckAcknowledge(AcknowledgeCallback callback)
{
    _acknowledgeCallback = callback;
}

// Aus dem Tick. Ohne Callback wird nicht quittiert - sonst gäbe das Gerät vor, unter jeder Adresse erreichbar zu sein.
AckType DataLinkLayer::checkAcknowledge(uint16_t destination, bool isGroupAddress)
{
    if (!_acknowledgeCallback) return AckType::None;

    // Gemessen: der Callback ist der einzige unbegrenzte Anteil des Ticks. Läuft einmal je Telegramm.
    uint32_t startedAt = micros();
    AckType result = _acknowledgeCallback(destination, isGroupAddress);
    _statistics.updateCheckAcknowledgeMaxUs(micros() - startedAt);

    return result;
}

// Aus dem Tick: Einmal-Merker und Zähler.
void DataLinkLayer::reportInterfaceOverflow()
{
    _interfaceOverflow = true;
    _statistics.incrementRxInterfaceOverflows();
}

void DataLinkLayer::reportRxQueueOverflow()
{
    _rxQueueOverflow = true;
    _statistics.incrementRxQueueOverflows();
}

void DataLinkLayer::reportControlOverflow()
{
    _ctrlQueueOverflow = true;
    _statistics.incrementTxControlQueueOverflows();
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

bool DataLinkLayer::pushTransmitQueue(const Frame &frame)
{
    return _transmitter.pushTransmitQueue(frame);
}

bool DataLinkLayer::pushTransmitQueue(const uint8_t *data, size_t length)
{
    if (data == nullptr) return false;
    if (length == 0 || length > TPUART_BUFFER_SIZE) return false;

    return pushTransmitQueue(Frame(data, length));
}

// KOMPAT: übernimmt den Besitz - der knx-Stack gibt das Frame nur im Fehlerfall selbst frei.
bool DataLinkLayer::pushTransmitQueue(Frame *frame)
{
    if (frame == nullptr) return false;

    if (!pushTransmitQueue(*frame)) return false;

    delete frame;
    return true;
}

} // namespace TPUart
