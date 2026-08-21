// TESTSAMMLUNG FÜR DIE TPUART-LIBRARY
//
// Läuft mit "pio test -e pico_test" bzw. "-e esp32_test" auf der jeweiligen Zielplattform. Nicht nativ:
// getestet werden soll, was der Compiler DIESER Plattform aus der Library macht, und die Interfaces sind
// ohnehin plattformgebunden. Der Gegenpart ist das Dummy-Interface nebenan, das den Bus nachstellt.
//
// ES WIRD IN ECHTZEIT GETESTET, es gibt keine stellbare Uhr. Ein Fall kostet damit so viel Zeit, wie die
// Sache eben dauert - die Pausenerkennung 2,8ms, der Verbindungsabbruch 5s. Das ist der Preis dafür, dass
// hier die echten Fristen geprüft werden und nicht heruntergedrehte. Der ganze Durchlauf bleibt trotzdem
// im Sekundenbereich.
//
// AUFBAU EINES FALLS: setUp() legt ein frisches Fixture an (Dummy + DataLinkLayer), der Fall stellt eine
// Situation her, pumpt sie durch und prüft das Ergebnis. Frisch je Fall, weil sich RxState und TxState von
// außen nicht zurücksetzen lassen - ein hängengebliebener Resync würde sonst in den nächsten Fall
// durchschlagen.
//
// NEUE FÄLLE: eine parameterlose Funktion schreiben und sie unten in setup() mit RUN_TEST() eintragen.
// Das ist die einzige Stelle, die von ihr weiß.

#include <Arduino.h>
#include <unity.h>

#include <vector>

#include "Dummy.h"

#include "TPUart/DataLinkLayer.h"

using namespace TPUart;

// ---------------------------------------------------------------------------------------------------
// Ausgabekanal für Unity
// ---------------------------------------------------------------------------------------------------
//
// Diese vier Funktionen erzeugt PlatformIO normalerweise selbst - sobald aber eine eigene unity_config.h
// im Testordner liegt (und die brauchen wir, siehe dort), übersetzt es seine Fassung nicht mehr mit.
//
// DER UNTERSCHIED IST DIE LETZTE: bei PlatformIO steht dort Serial.end(), und das meldet auf dem RP2040
// den USB-CDC vom Bus ab, während der Testläufer noch liest. Hier bleibt die Verbindung offen.
#ifdef ARDUINO
extern "C"
{
    void unityOutputStart(unsigned long baudrate) { Serial.begin(baudrate); }
    void unityOutputChar(unsigned int c) { Serial.write(c); }
    void unityOutputFlush(void) { Serial.flush(); }

    void unityOutputComplete(void)
    {
        // Nur sicherstellen, dass alles draußen ist. Die Schnittstelle bleibt bestehen - der Testläufer
        // liest hier noch, und der nächste Upload braucht sie.
        Serial.flush();
    }
}
#endif

// ---------------------------------------------------------------------------------------------------
// Fixture und Helfer
// ---------------------------------------------------------------------------------------------------

// Ein geliefertes Telegramm, kopiert - das Frame selbst lebt nur für die Dauer des Callbacks.
struct CapturedFrame
{
    std::vector<uint8_t> _data;
    uint8_t _flags = 0;
    bool _valid = false;
    bool _isFrame = false;
    uint16_t _source = 0;
    uint16_t _destination = 0;
};

struct Fixture
{
    Interface::Dummy _interface;
    DataLinkLayer _dll;

    std::vector<CapturedFrame> _frames;
    std::vector<std::string> _errors; // nur die Fehlermeldungen, die sind die interessanten
    AckType _ackAnswer = AckType::None;
    uint16_t _lastAckDestination = 0;

    Fixture() : _dll(_interface)
    {
        // DIE ZUSTELLUNG MUSS BILLIG SEIN, und das ist keine Kosmetik. Ohne laufenden Timer laufen Tick
        // und Loop serialisiert: was der Callback kostet, liegt zwischen dem Tick, der das letzte Byte einer
        // Sequenz verbraucht, und dem nächsten, der die Leere bemerkt und _emptySince setzt. Diese Kosten
        // verschieben also die Pausenerkennung nach hinten.
        // Ohne das Reservieren wuchs _frames während eines Falls um und kopierte dabei jeden Eintrag samt
        // seines Vektors neu - sprunghaft teuer, und genau dadurch fiel
        // test_confirmation_after_broken_echo_arrives_after_pause auf dem RP2040 in etwa der Hälfte der
        // Läufe durch: die Pause wurde erst erkannt, nachdem das L_Data.con schon eingetroffen und im
        // Resync verworfen war. Auf dem ESP32 war die Reserve groß genug, dort fiel es nie auf.
        _frames.reserve(64);
        _errors.reserve(32);

        // KEIN LAUFENDER TIMER: das Intervall ist global, 0 hält ihn an. Getickt wird dann ausschließlich
        // aus pump()/tickOnly(), jeder Fall bestimmt also selbst, wann wie oft. Nativ wäre das ohnehin so
        // (Timer::supported() ist dort falsch), auf der Hardware nicht: dort liefe sonst ein echter Timer
        // nebenher und tickte gegen die Fälle an - zwei Kontexte auf einem Interface.
        //
        // Die Instanz wird von begin() trotzdem eingetragen - das Verzeichnis ist plattformunabhängig, nur
        // der Takt nicht. Ausgetragen wird sie vom Destruktor, jeder Fall fängt also sauber an.
        Timer::instance().setInterval(0);

        _dll.registerFrameCallback([this](Frame &frame) {
            CapturedFrame captured;
            captured._data.assign((const uint8_t *)frame.data(), (const uint8_t *)frame.data() + frame.length());
            captured._flags = frame.flags();
            captured._valid = frame.isValid();
            captured._isFrame = frame.isFrame();
            captured._source = frame.source();
            captured._destination = frame.destination();
            _frames.push_back(std::move(captured)); // verschieben, nicht den Datenvektor erneut kopieren
        });

        _dll.registerCheckAcknowledge([this](uint16_t destination, bool isGroupAddress) {
            (void)isGroupAddress;
            _lastAckDestination = destination;
            return _ackAnswer;
        });

        _dll.registerMessage([this](const char *message, bool error) {
            if (error) _errors.push_back(message);
        });
    }

    ~Fixture()
    {
        _dll.end();
    }

    // Die einzige Stelle, an der Zeit vergeht. Alles andere ist reine Zustandsprüfung.
    //
    // AUF DER HARDWARE läuft die echte Uhr, und getickt wird so schnell, wie der Prozessor durch die
    // Schleife kommt. Damit misst die Sammlung dort tatsächliche Fristen - das ist ihr eigentlicher Wert.
    //
    // NATIV gibt es keine echte Zeit, also wird sie gestellt: die Uhr rückt in festen Schritten vor, und
    // zwischen zwei Schritten läuft ein tick()/loop()-Paar. Die Schrittweite IST damit die Tickrate. 50µs liegen
    // deutlich unter dem 500µs-Vorgabetakt der Library und weit unter der Bytezeit des Busses (1354µs),
    // die Pausenerkennung sieht also feiner auf als auf jeder echten Plattform - was Fälle durchgehen
    // lässt, die auf der Hardware knapp scheitern würden. Der native Lauf prüft die Protokolllogik, nicht
    // das Zeitverhalten.
    // TICK UND LOOP AUSDRÜCKLICH, nicht über process(): process() tickt NICHT (der Hauptloop ist kein
    // Antrieb, siehe DataLinkLayer::process()). Hier steht damit genau das, was im echten Gerät der Timer
    // und der Hauptloop zusammen tun - nur serialisiert, damit jeder Fall die Reihenfolge in der Hand hat.
    void pump(uint32_t ms)
    {
#ifdef ARDUINO
        uint32_t until = millis() + ms;
        while ((int32_t)(millis() - until) < 0)
        {
            _dll.tick();
            _dll.loop();
        }
#else
        constexpr uint32_t STEP_US = 50;

        for (uint32_t elapsed = 0; elapsed < ms * 1000; elapsed += STEP_US)
        {
            tpuartNativeClockUs() += STEP_US;
            _dll.tick();
            _dll.loop();
        }
#endif
    }

    // ZEIT VERGEHT, ABER ES WIRD NICHT GETICKT - das Gegenstück zu pump(). Nur so lässt sich ein
    // ausgehungerter Antrieb nachstellen, und genau darauf sitzt der Wächter in checkTickRate().
    void idle(uint32_t ms)
    {
#ifdef ARDUINO
        uint32_t until = millis() + ms;
        while ((int32_t)(millis() - until) < 0)
            _dll.loop();
#else
        constexpr uint32_t STEP_US = 50;

        for (uint32_t elapsed = 0; elapsed < ms * 1000; elapsed += STEP_US)
        {
            tpuartNativeClockUs() += STEP_US;
            _dll.loop();
        }
#endif
    }

    // NUR TICKEN, NICHT LOOPEN - damit füllt sich der RX-Ring, ohne dass ihn jemand leert. Das ist genau
    // das Bild eines steckengebliebenen Hauptloops, und anders ist ein Ringüberlauf nicht nachzustellen.
    void tickOnly(uint32_t ms)
    {
#ifdef ARDUINO
        uint32_t until = millis() + ms;
        while ((int32_t)(millis() - until) < 0)
            _dll.tick();
#else
        constexpr uint32_t STEP_US = 50;

        for (uint32_t elapsed = 0; elapsed < ms * 1000; elapsed += STEP_US)
        {
            tpuartNativeClockUs() += STEP_US;
            _dll.tick();
        }
#endif
    }

    // Verbindungsaufnahme durchspielen. Die Baudratensuche liest das Interface DIREKT (nicht über den
    // Empfänger), es genügt also, die Antwort einzuspeisen, sobald der Request draußen ist.
    void connect(BcuType type = BcuType::Ncn5120)
    {
        _dll.begin(type);

        _dll.tick(); // schickt U_Reset.req an den ersten Kandidaten
        _dll.loop();

        _interface.addByte((char)U_RESET_IND, 0);

        for (int i = 0; i < 50 && !_dll.isConnected(); i++)
        {
            _dll.tick();
            _dll.loop();
        }

        // Was die Verbindungsaufnahme selbst geschrieben hat (Reset, Konfiguration), interessiert die
        // Fälle nicht - sie prüfen gegen das, was DANACH rausgeht.
        pump(5);
        _interface.clearWrittenBytes();
        _frames.clear();
        _errors.clear();
    }

    // Hängt eine Bytefolge im Bus-Zeitraster an das laufende Skript an (ein TP1-Zeichen dauert 1354µs),
    // OHNE den Fahrplan neu aufzusetzen - damit lassen sich mehrere Blöcke hintereinander einspeisen und
    // die Abstände dazwischen gezielt setzen.
    //
    // trailingGapUs gilt nur für das LETZTE Byte des Blocks, und darauf kommt es bei den Fehlerfällen an:
    // eine Pause gehört hinter den Block, nicht zwischen seine Bytes. Bekäme jedes Byte 3000µs, zerfiele
    // schon ein zweibytiges Fragment in zwei Sequenzen.
    void append(const uint8_t *data, size_t length, uint32_t gapUs = 1354, uint32_t trailingGapUs = 0)
    {
        for (size_t i = 0; i < length; i++)
        {
            bool last = (i + 1 == length);
            _interface.addByte((char)data[i], last && trailingGapUs > 0 ? trailingGapUs : gapUs);
        }
    }

    void append(const std::vector<uint8_t> &data, uint32_t gapUs = 1354, uint32_t trailingGapUs = 0)
    {
        append(data.data(), data.size(), gapUs, trailingGapUs);
    }

    // Wie append(), setzt den Fahrplan aber vorher auf JETZT zurück - und das ist keine Kosmetik.
    // _nextAvailableAt steht nur nach begin()/clearData() auf der aktuellen Zeit und wandert danach je
    // gelesenem Byte um dessen Pause weiter. Nach der Verbindungsaufnahme liegt er deshalb einige
    // Millisekunden in der VERGANGENHEIT - dann sind die ersten eingespeisten Bytes alle auf einen Schlag
    // verfügbar statt im Raster. Die Reihenfolge stimmt zwar, aber jede Aussage über Zeit wird wertlos:
    // eine Pausenerkennung, die nach 13ms noch nicht hätte greifen dürfen, war schon durch. Genau daran
    // sind die beiden Poll-Fälle zuerst gescheitert.
    void feed(const uint8_t *data, size_t length, uint32_t gapUs = 1354, uint32_t trailingGapUs = 0)
    {
        _interface.clearData();
        append(data, length, gapUs, trailingGapUs);
    }

    void feed(const std::vector<uint8_t> &data, uint32_t gapUs = 1354, uint32_t trailingGapUs = 0)
    {
        feed(data.data(), data.size(), gapUs, trailingGapUs);
    }

    // Einspeisen und so lange takten, bis alle Bytes verarbeitet sein können, plus die Pausenfrist.
    void feedAndSettle(const std::vector<uint8_t> &data, uint32_t gapUs = 1354)
    {
        feed(data, gapUs);
        pump((uint32_t)((data.size() * gapUs) / 1000) + 6);
    }

    bool written(const std::vector<uint8_t> &expected) const
    {
        return _interface.writtenBytes() == expected;
    }
};

static Fixture *fx = nullptr;

void setUp()
{
    fx = new Fixture();
}

void tearDown()
{
    delete fx;
    fx = nullptr;
}

// KNX-Prüfsumme: das Komplement des XOR über alle vorhergehenden Bytes.
static uint8_t checksum(const std::vector<uint8_t> &data)
{
    uint8_t crc = 0;
    for (uint8_t value : data)
        crc ^= value;

    return (uint8_t)~crc;
}

// Ein Standard-Telegramm mit einem APDU-Byte: Ctrl, Quelle, Ziel, Adresstyp+Länge, TPCI, APCI, Prüfsumme.
// apduSize geht hier nur bis 15: im Standard-Format ist die Länge ein NIBBLE. Damit sind 8 + 15 = 23
// Oktette das Maximum - der Grund, warum es das Extended-Format überhaupt gibt.
static std::vector<uint8_t> standardFrame(uint16_t source = 0x1101, uint16_t destination = 0x0801, uint8_t apduSize = 1)
{
    std::vector<uint8_t> frame = {
        0xBC, // L_Data standard, Priorität low, keine Wiederholung
        (uint8_t)(source >> 8), (uint8_t)source,
        (uint8_t)(destination >> 8), (uint8_t)destination,
        (uint8_t)(0xE0 | (apduSize & 0x0F)), // Gruppenadresse, Hop-Count 6, Längen-Nibble
        0x00};                               // TPCI

    for (uint8_t i = 0; i < apduSize; i++)
        frame.push_back(i == 0 ? 0x81 : (uint8_t)i);

    frame.push_back(checksum(frame));
    return frame;
}

// Dasselbe als EXTENDED-Telegramm. Der Aufbau unterscheidet sich an jeder Stelle, die zählt: das
// Kontrollfeld erkennt man an einem anderen Bitmuster (0x3C & 0xD3 == 0x10), dahinter steht ein zweites
// Kontrollbyte mit dem Adresstyp, die Adressen rücken um eins nach hinten, und die Länge ist ein VOLLES
// Byte statt eines Nibbles - deshalb passen hier bis zu 254 APDU-Oktette hinein statt 15.
//
// apduSize ist frei wählbar, weil zwei Randfälle interessieren: die 254 für das größtmögliche Telegramm
// (263 Oktette) und die 1 für ein Extended, das KÜRZER ist als mancher Standard. Beides ist erlaubt, und
// die Formatwahl hat nichts mit der Länge zu tun - genau das prüfen die Fälle unten.
static std::vector<uint8_t> extendedFrame(uint16_t source = 0x1101, uint16_t destination = 0x0801, uint8_t apduSize = 1)
{
    std::vector<uint8_t> frame = {
        0x3C, // L_Data extended, Priorität low, keine Wiederholung
        0xE0, // Zieladresse ist eine Gruppenadresse, Hop-Count 6, Extended Frame Format 0
        (uint8_t)(source >> 8), (uint8_t)source,
        (uint8_t)(destination >> 8), (uint8_t)destination,
        apduSize,
        0x00}; // TPCI

    for (uint8_t i = 0; i < apduSize; i++)
        frame.push_back(i == 0 ? 0x81 : (uint8_t)i);

    frame.push_back(checksum(frame));
    return frame;
}

// ---------------------------------------------------------------------------------------------------
// Verbindung
// ---------------------------------------------------------------------------------------------------

static void test_connect_reports_connected()
{
    fx->connect();

    TEST_ASSERT_TRUE(fx->_dll.isConnected());
    TEST_ASSERT_EQUAL(BcuType::Ncn5120, fx->_dll.bcuType());
}

// Ohne Antwort der BCU darf die Verbindung NICHT zustande kommen - sonst würde ein totes Interface als
// funktionierend gemeldet.
static void test_connect_without_answer_stays_disconnected()
{
    fx->_dll.begin(BcuType::Ncn5120);
    fx->pump(120);

    TEST_ASSERT_FALSE(fx->_dll.isConnected());
}

// ---------------------------------------------------------------------------------------------------
// Rahmenerkennung
// ---------------------------------------------------------------------------------------------------

// Ein gültiges Telegramm kommt vollständig oben an - Format, Länge, Adressen. Beides wird geprüft, weil
// die beiden Formate ihre Adressen an verschiedenen Stellen tragen und die Länge verschieden herleiten.
static void expectFrameDelivered(const std::vector<uint8_t> &frame, uint16_t source, uint16_t destination)
{
    fx->connect();
    fx->feedAndSettle(frame);

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE(fx->_frames[0]._isFrame);
    TEST_ASSERT_TRUE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(frame.size(), fx->_frames[0]._data.size());
    TEST_ASSERT_EQUAL_HEX16(source, fx->_frames[0]._source);
    TEST_ASSERT_EQUAL_HEX16(destination, fx->_frames[0]._destination);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getRxFrames());
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getRxInvalidFrames());
}

static void test_standard_frame_is_delivered()
{
    expectFrameDelivered(standardFrame(), 0x1101, 0x0801);
}

// EIN EXTENDED, DAS KÜRZER IST ALS EIN STANDARD - und das ist erlaubt. Mit einem APDU-Oktett sind es zehn
// Oktette; das Format sagt nichts über die Länge, sondern nur darüber, wo was steht. Wer die beiden über
// die Größe auseinanderhalten wollte, läge hier falsch.
static void test_short_extended_frame_is_delivered()
{
    expectFrameDelivered(extendedFrame(0x1101, 0x0801, 1), 0x1101, 0x0801);
}

// Das größtmögliche Telegramm überhaupt: 254 APDU-Oktette, 263 insgesamt. Genau darauf ist der
// Empfangspuffer ausgelegt (static_assert in Receiver.cpp), ein Byte mehr wäre ein Überlauf.
static void test_extended_frame_maximum_length()
{
    std::vector<uint8_t> frame = extendedFrame(0x0E01, 0xF000, 254);
    TEST_ASSERT_EQUAL(263, frame.size());

    expectFrameDelivered(frame, 0x0E01, 0xF000);
}

// Falsche Prüfsumme: gemeldet WIRD es trotzdem, aber als kaputt - das ist der Unterschied zur alten
// Library, die es verworfen hätte.
static void expectBadChecksumReported(std::vector<uint8_t> frame)
{
    fx->connect();

    frame.back() ^= 0xFF;
    fx->feedAndSettle(frame);

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_FALSE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getRxInvalidFrames());
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getRxFrames());
}

// Alle vier Größen, weil der Prüfsummenvergleich an der ERWARTETEN Länge hängt: sie kommt aus dem
// Nibble bzw. dem Längenbyte, und das Prüfsummen-Oktett ist das letzte davor. Ein Fehler in dieser
// Rechnung fiele bei einem kurzen Telegramm womöglich nicht auf, bei 263 Oktetten aber sicher.
static void test_bad_checksum_is_reported_as_invalid()
{
    expectBadChecksumReported(standardFrame(0x1101, 0x0801, 1));
}

static void test_bad_checksum_standard_maximal()
{
    expectBadChecksumReported(standardFrame(0x1101, 0x0801, 15));
}

static void test_extended_bad_checksum_is_reported_as_invalid()
{
    expectBadChecksumReported(extendedFrame(0x1101, 0x0801, 1));
}

static void test_bad_checksum_extended_maximal()
{
    expectBadChecksumReported(extendedFrame(0x0E01, 0xF000, 254));
}

// Von einer Pause abgeschnitten: gemeldet wird, was angekommen ist, mit INVALID.
static void test_truncated_frame_is_reported_as_invalid()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    frame.resize(5); // mitten im Kopf abgeschnitten
    fx->feedAndSettle(frame);

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_FALSE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(5, fx->_frames[0]._data.size());
}

// Nach einem Fehler ist die Position im Bytestrom unbekannt. Alles bis zur nächsten verifizierten Pause
// wird verworfen - und danach läuft es wieder sauber.
static void test_resync_recovers_at_next_frame()
{
    fx->connect();

    std::vector<uint8_t> broken = standardFrame();
    broken.back() ^= 0xFF;
    fx->feedAndSettle(broken);
    fx->_frames.clear();

    fx->feedAndSettle(standardFrame(0x1102));

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL_HEX16(0x1102, fx->_frames[0]._source);
}

// Der Resync-ZÄHLER beantwortet eine andere Frage als die verworfenen Bytes: wie oft die Position im
// Bytestrom verloren ging, nicht wie viel es gekostet hat. Zwei kaputte Telegramme sind zwei Resyncs,
// ganz gleich wie viele Bytes dazwischen anfielen.
static void test_resyncs_are_counted()
{
    fx->connect();

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getRxResyncs());

    std::vector<uint8_t> broken = standardFrame();
    broken.back() ^= 0xFF;

    fx->feedAndSettle(broken);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getRxResyncs());

    fx->feedAndSettle(broken);
    TEST_ASSERT_EQUAL(2, fx->_dll.getStatistics().getRxResyncs());

    // Ein GÜLTIGES Telegramm loest keinen aus - sonst zaehlte der Wert den Normalbetrieb mit.
    fx->feedAndSettle(standardFrame(0x1102));
    TEST_ASSERT_EQUAL(2, fx->_dll.getStatistics().getRxResyncs());
}

// Der Höchststand zeigt die Auslegung, bevor etwas überläuft. Ein Telegramm im Ring belegt seine Länge
// plus den Kopf des Eintrags; abgeholt wird es erst im nächsten loop().
static void test_rx_queue_peak_is_tracked()
{
    fx->connect();

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getRxQueuePeakBytes());

    fx->feedAndSettle(standardFrame());

    uint32_t peak = fx->_dll.getStatistics().getRxQueuePeakBytes();

    TEST_ASSERT_TRUE(peak >= 9);                     // 9 Oktette Telegramm ...
    TEST_ASSERT_TRUE(peak <= TPUART_RX_QUEUE_SIZE);  // ... und niemals mehr, als der Ring fasst
}

// Die Fehlerbits eines U_State.ind werden nur verodert und beim Ausgeben gelöscht - erst die Zähler
// unterscheiden "einmal" von "dauernd". EINZELN, weil es fünf verschiedene Diagnosen sind: ein Telegramm
// mit zwei gesetzten Bits erhöht auch zwei Zähler.
static void test_chip_errors_are_counted_individually()
{
    fx->connect();

    const Statistics &st = fx->_dll.getStatistics();

    TEST_ASSERT_EQUAL(0, st.getChipProtocolErrors());
    TEST_ASSERT_EQUAL(0, st.getChipTemperatureWarnings());

    // U_State.ind ohne Fehlerbits: die drei Kennbits allein.
    uint8_t clean[] = {U_STATE_IND};
    fx->feedAndSettle(std::vector<uint8_t>(clean, clean + 1));
    TEST_ASSERT_EQUAL(0, st.getChipProtocolErrors());

    // Ein Bit.
    uint8_t pe[] = {(uint8_t)(U_STATE_IND | U_STATE_PROTOCOL_ERROR)};
    fx->feedAndSettle(std::vector<uint8_t>(pe, pe + 1));
    TEST_ASSERT_EQUAL(1, st.getChipProtocolErrors());
    TEST_ASSERT_EQUAL(0, st.getChipTemperatureWarnings());

    // Zwei Bits in EINER Meldung - beide Zähler rücken vor. Genau deshalb ist die Zahl der Meldungen aus
    // den fünf Zählern nicht ableitbar.
    uint8_t both[] = {(uint8_t)(U_STATE_IND | U_STATE_PROTOCOL_ERROR | U_STATE_TEMPERATURE_WARNING)};
    fx->feedAndSettle(std::vector<uint8_t>(both, both + 1));
    TEST_ASSERT_EQUAL(2, st.getChipProtocolErrors());
    TEST_ASSERT_EQUAL(1, st.getChipTemperatureWarnings());

    TEST_ASSERT_EQUAL(0, st.getChipSlaveCollisions());
    TEST_ASSERT_EQUAL(0, st.getChipReceiveErrors());
    TEST_ASSERT_EQUAL(0, st.getChipTransmitErrors());
}

// Eine markierte Wiederholung desselben Telegramms wird als solche erkannt - gemeldet trotzdem, damit ein
// Busmonitor sie sieht.
static void test_repeated_frame_is_filtered()
{
    fx->connect();

    fx->feedAndSettle(standardFrame());

    std::vector<uint8_t> repeated = standardFrame();
    repeated[0] &= (uint8_t)~0x20; // Wiederholungsbit ist invertiert: gelöscht heißt "Wiederholung"
    repeated.back() = checksum(std::vector<uint8_t>(repeated.begin(), repeated.end() - 1));
    fx->feedAndSettle(repeated);

    TEST_ASSERT_EQUAL(2, fx->_frames.size());
    TEST_ASSERT_TRUE((fx->_frames[1]._flags & TP_FRAME_FLAG_FILTERED) != 0);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getRxRepetitions());
}

// ---------------------------------------------------------------------------------------------------
// Fehlerfälle: ein angefangenes Telegramm, das nie fertig wird
// ---------------------------------------------------------------------------------------------------
//
// Grundlage beider Fälle sind zwei Bytes, 0xBC 0x01. Das erste ist ein gültiger Telegrammanfang, danach
// bricht es ab - die Länge steht zu diesem Zeitpunkt noch gar nicht fest, die käme erst aus dem sechsten
// Byte. Was daraus wird, entscheidet allein, ob eine Pause folgt.

// MIT PAUSE ist die Sache eindeutig: die Pause ist die Telegrammgrenze, das Fragment wird als kaputtes
// Telegramm gemeldet, und das nächste läuft völlig unbeeinflusst ein.
static void test_fragment_then_pause_then_valid_frame()
{
    fx->connect();

    fx->feed({0xBC, 0x01}, 1354, 3000); // 3ms Pause hinter dem Fragment
    fx->append(standardFrame());
    fx->pump(30);

    TEST_ASSERT_EQUAL(2, fx->_frames.size());

    TEST_ASSERT_FALSE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(2, fx->_frames[0]._data.size());
    TEST_ASSERT_EQUAL_HEX8(0xBC, fx->_frames[0]._data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, fx->_frames[0]._data[1]);

    TEST_ASSERT_TRUE(fx->_frames[1]._valid);
    TEST_ASSERT_EQUAL_HEX16(0x1101, fx->_frames[1]._source);
}

// OHNE PAUSE verschluckt das Fragment das nachfolgende Telegramm, und das ist kein Fehler, sondern die
// Folge des Konzepts: ohne Grenze gibt es keinen Grund, hier etwas Neues anzunehmen. Die Bytes laufen in
// dieselbe Sequenz, und aus dem sechsten (0x08, in Wahrheit ein Adressbyte des zweiten Telegramms) errechnet
// sich eine Länge von 16 - die nie erreicht wird, weil vorher die Pause kommt.
//
// Gemeldet wird deshalb EIN kaputtes Telegramm aus 11 Bytes: die zwei des Fragments plus die neun des
// verschluckten. Erst das Telegramm nach der Pause kommt wieder sauber an. Genau dafür gibt es das
// INVALID-Flag - der Verlust ist sichtbar, statt sich als stiller Bytestrom zu verlieren.
static void test_fragment_without_pause_swallows_next_frame()
{
    fx->connect();

    fx->feed({0xBC, 0x01});                       // kein Sonderabstand: es geht nahtlos weiter
    fx->append(standardFrame(), 1354, 3000);      // dieses wird verschluckt, danach 3ms Pause
    fx->append(standardFrame(0x1102));            // und dieses kommt wieder sauber
    fx->pump(40);

    TEST_ASSERT_EQUAL(2, fx->_frames.size());

    TEST_ASSERT_FALSE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(11, fx->_frames[0]._data.size());
    TEST_ASSERT_EQUAL_HEX8(0xBC, fx->_frames[0]._data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, fx->_frames[0]._data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xBC, fx->_frames[0]._data[2]); // ab hier steckt das verschluckte Telegramm drin

    TEST_ASSERT_TRUE(fx->_frames[1]._valid);
    TEST_ASSERT_EQUAL_HEX16(0x1102, fx->_frames[1]._source);
}

// DIE LÄNGE ZEIGT WEIT ÜBER DAS ENDE HINAUS - und die Pause gewinnt trotzdem.
//
// Aufbau wie oben, nur ist das verschluckte Telegramm so gewählt, dass die errechnete Länge nicht bloß
// etwas zu groß ist, sondern über das ÜBERNÄCHSTE Telegramm hinausreicht. Die Zieladresse 0x0F01 sorgt
// dafür: im zusammengelaufenen Puffer landet ihr oberes Byte auf Position 5, und dessen unteres Nibble
// ist das Längen-Nibble - 0x0F ergibt 8 + 15 = 23 Oktette.
//
// Vorhanden sind nur 11. Ohne die Pause würde der Empfänger also weitersammeln und das dritte Telegramm
// gleich mitnehmen. Genau das darf nicht passieren: die Pause ist die Grenze, nicht die Länge. Deshalb
// steht hier auch die Prüfung auf NULL verworfene Bytes - abgeschnitten wird sauber, es bleibt nichts
// liegen, das anschließend im Resync verlorenginge.
static void test_fragment_with_overlong_length_still_ends_at_pause()
{
    fx->connect();

    uint32_t droppedBefore = fx->_dll.getStatistics().getRxDroppedBytes();

    fx->feed({0xBC, 0x01});
    fx->append(standardFrame(0x1101, 0x0F01), 1354, 3000);
    fx->append(standardFrame(0x1102));
    fx->pump(40);

    TEST_ASSERT_EQUAL(2, fx->_frames.size());

    TEST_ASSERT_FALSE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(11, fx->_frames[0]._data.size());

    TEST_ASSERT_TRUE(fx->_frames[1]._valid);
    TEST_ASSERT_EQUAL_HEX16(0x1102, fx->_frames[1]._source);

    TEST_ASSERT_EQUAL(droppedBefore, fx->_dll.getStatistics().getRxDroppedBytes());
}

// DASSELBE MIT EINEM ECHTEN 254 IM LÄNGENBYTE, und dafür muss das Fragment ein EXTENDED-Anfang sein:
// im Standard-Format steckt die Länge in einem Nibble, mehr als 15 ist dort gar nicht ausdrückbar. Mit
// 0x3C beginnt die Sequenz als Extended, das Längenbyte liegt dann auf Position 6 - dort landet das
// untere Adressbyte des verschluckten Telegramms, hier 0xFE. Macht 9 + 254 = 263 Oktette, also die
// größtmögliche Sequenz überhaupt.
//
// Auch das ändert nichts: die Pause schließt nach 11 Bytes ab, verworfen wird nichts.
static void test_fragment_with_length_254_still_ends_at_pause()
{
    fx->connect();

    uint32_t droppedBefore = fx->_dll.getStatistics().getRxDroppedBytes();

    fx->feed({0x3C, 0x01});
    fx->append(standardFrame(0x1101, 0x08FE), 1354, 3000);
    fx->append(standardFrame(0x1102));
    fx->pump(40);

    TEST_ASSERT_EQUAL(2, fx->_frames.size());

    TEST_ASSERT_FALSE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(11, fx->_frames[0]._data.size());

    TEST_ASSERT_TRUE(fx->_frames[1]._valid);
    TEST_ASSERT_EQUAL(droppedBefore, fx->_dll.getStatistics().getRxDroppedBytes());
}

// DIE LÄNGE IST ZU KURZ - und jetzt bleibt tatsächlich etwas liegen.
//
// Die Zieladresse 0x0001 setzt das Längen-Nibble auf 0, die Sequenz ist damit nach 8 Oktetten "fertig".
// Sie umfasst dann das Fragment und die ersten sechs Bytes des verschluckten Telegramms; die Prüfsumme
// stimmt natürlich nicht, also INVALID und Resync.
//
// DIE RESTLICHEN DREI BYTES sind der Unterschied zu den Fällen oben: sie gehören zu keiner erkannten
// Sequenz mehr und werden im Resync verworfen - sichtbar als getRxDroppedBytes(), nicht als Telegramm.
// Genau dafür sind die beiden Zähler getrennt: "gemeldet, aber kaputt" ist etwas anderes als "hat nie
// jemand gesehen".
static void test_fragment_with_short_length_leaves_discarded_bytes()
{
    fx->connect();

    uint32_t droppedBefore = fx->_dll.getStatistics().getRxDroppedBytes();

    fx->feed({0xBC, 0x01});
    fx->append(standardFrame(0x1101, 0x0001), 1354, 3000);
    fx->append(standardFrame(0x1102));
    fx->pump(40);

    TEST_ASSERT_EQUAL(2, fx->_frames.size());

    TEST_ASSERT_FALSE(fx->_frames[0]._valid);
    TEST_ASSERT_EQUAL(8, fx->_frames[0]._data.size());

    TEST_ASSERT_TRUE(fx->_frames[1]._valid);
    TEST_ASSERT_EQUAL_HEX16(0x1102, fx->_frames[1]._source);

    TEST_ASSERT_EQUAL(droppedBefore + 3, fx->_dll.getStatistics().getRxDroppedBytes());
}

// ---------------------------------------------------------------------------------------------------
// Steuerbytes
// ---------------------------------------------------------------------------------------------------

// Ein Steuerbyte außerhalb der Diensttabelle kann die BCU gar nicht senden - kommt trotzdem eines an,
// stimmt die Deutung des Bytestroms nicht mehr, und das gehört als Fehler gemeldet.
static void test_unknown_control_byte_is_reported_as_error()
{
    fx->connect();

    // 0x54 fällt durch JEDE Maske der Diensttabelle - das ist der Punkt, und es ist enger, als es
    // aussieht: 0x55 etwa wäre schon ein U_Configure.ind (0x55 & U_CONFIGURE_MASK == U_CONFIGURE_IND)
    // und würde als bekannter Dienst still geschluckt.
    uint8_t unknown = 0x54;
    fx->feed(&unknown, 1);
    fx->pump(6);

    TEST_ASSERT_EQUAL(1, fx->_errors.size());
}

// Der einzige mehrbytige Steuerdienst, und nur beim NCN512x: das Statusbyte gehört zur Sequenz und darf
// nicht als Anfang von etwas Neuem gelesen werden.
static void test_system_state_is_a_two_byte_sequence()
{
    fx->connect(BcuType::Ncn5120);

    uint8_t sequence[] = {U_SYSTEM_STAT_IND, 0x3F};
    fx->feed(sequence, sizeof(sequence));
    fx->pump(6);

    TEST_ASSERT_TRUE(fx->_dll.getSystemState().isValid());
    TEST_ASSERT_EQUAL(0, fx->_errors.size());
}

// ---------------------------------------------------------------------------------------------------
// Poll-Telegramme
// ---------------------------------------------------------------------------------------------------

// Der Regelfall an echter Hardware: der Chip reicht nur das Steuerbyte durch. Das ist NICHT kaputt - es
// geht kein Resync los und es wird nichts verworfen.
//
// ZUR PRÜFGRÖSSE: naheliegend wäre getRxInvalidFrames() gewesen, aber der Zähler zählt nur TELEGRAMME,
// und ein Poll ist keines - er bliebe auch dann bei 0, wenn der Code das Byte als kaputt behandelte. Der
// Fall wäre also grün gewesen, ohne etwas zu prüfen. Unterscheidbar sind die beiden Ausgänge von außen am
// Zustand der Empfangsmaschine und an den verworfenen Bytes.
static void test_poll_control_byte_only_is_not_invalid()
{
    fx->connect();

    uint32_t droppedBefore = fx->_dll.getStatistics().getRxDroppedBytes();

    uint8_t poll = L_POLL_DATA_IND;
    fx->feed(&poll, 1);
    fx->pump(6);

    TEST_ASSERT_EQUAL(RxState::Idle, fx->_dll.getReceiver().state());
    TEST_ASSERT_EQUAL(droppedBefore, fx->_dll.getStatistics().getRxDroppedBytes());
    TEST_ASSERT_EQUAL(0, fx->_errors.size());
}

// Kommt der ganze Zyklus durch, wird er anhand des Slot-Counts abgezählt und OHNE Pause abgeschlossen.
static void test_poll_full_cycle_completes_without_pause()
{
    fx->connect();

    std::vector<uint8_t> poll = {L_POLL_DATA_IND, 0x11, 0x00, 0x1F, 0xF0, 3};
    poll.push_back(checksum(poll));
    poll.push_back(0xA0);
    poll.push_back(0xA1);
    poll.push_back(0xA2);

    fx->feed(poll);
    // NUR so lange takten, wie die Bytes brauchen - keine Pausenfrist obendrauf. Genau das ist der Punkt.
    fx->pump((uint32_t)((poll.size() * 1354) / 1000) + 2);

    TEST_ASSERT_EQUAL(RxState::Idle, fx->_dll.getReceiver().state());
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getRxInvalidFrames());
}

// Stimmt der Kopf nicht, ist auch das Ende des Zyklus unbekannt - dann hilft nur der Resync bis zur
// nächsten verifizierten Pause. Beides wird geprüft: dass er anspringt (Zustand direkt nach dem Kopf)
// und dass er das Nachfolgende wirklich verwirft.
static void expectPollHeaderRejected(const std::vector<uint8_t> &header)
{
    std::vector<uint8_t> stream = header;
    stream.push_back(0xA0); // drei Slot-Bytes, die im Resync verlorengehen müssen
    stream.push_back(0xA1);
    stream.push_back(0xA2);

    uint32_t droppedBefore = fx->_dll.getStatistics().getRxDroppedBytes();

    fx->feed(stream);
    fx->pump((uint32_t)((stream.size() * 1354) / 1000)); // ohne Pausenfrist obendrauf

    TEST_ASSERT_EQUAL(RxState::Resync, fx->_dll.getReceiver().state());

    fx->pump(6); // jetzt die Pause abwarten

    TEST_ASSERT_EQUAL(RxState::Idle, fx->_dll.getReceiver().state());
    TEST_ASSERT_EQUAL(droppedBefore + 3, fx->_dll.getStatistics().getRxDroppedBytes());
}

static void test_poll_bad_checksum_is_rejected()
{
    fx->connect();

    std::vector<uint8_t> poll = {L_POLL_DATA_IND, 0x11, 0x00, 0x1F, 0xF0, 3};
    poll.push_back((uint8_t)(checksum(poll) ^ 0xFF));

    expectPollHeaderRejected(poll);
}

// Mehr Slots, als es Slotnummern gibt (0...14): dann stimmt der Kopf nicht.
static void test_poll_impossible_slot_count_is_rejected()
{
    fx->connect();

    std::vector<uint8_t> poll = {L_POLL_DATA_IND, 0x11, 0x00, 0x1F, 0xF0, 99};
    poll.push_back(checksum(poll));

    expectPollHeaderRejected(poll);
}

// Ein Poll darf NIE quittiert werden - er trägt keinen Adresstyp, geantwortet wird mit dem eigenen Slot.
static void test_poll_is_never_acknowledged()
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed; // selbst wenn der Callback alles bejahen würde

    std::vector<uint8_t> poll = {L_POLL_DATA_IND, 0x11, 0x00, 0x1F, 0xF0, 1};
    poll.push_back(checksum(poll));
    poll.push_back(0xA0);

    fx->feedAndSettle(poll);

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTxAcknowledges());
}

// ---------------------------------------------------------------------------------------------------
// Quittung
// ---------------------------------------------------------------------------------------------------

// Die Quittungsentscheidung fällt, sobald Ziel, Adresstyp und Restlänge feststehen - und die liest der
// Empfänger bei den beiden Formaten an VERSCHIEDENEN Positionen aus (Ziel bei 3/4 gegen 4/5, Adresstyp im
// Byte 5 gegen Byte 1). Deshalb wird hier auch geprüft, welches Ziel im Callback ankommt: eine Verwechslung
// der Positionen fiele sonst nicht auf, weil quittiert würde ja trotzdem.
static void expectAddressedAcknowledge(const std::vector<uint8_t> &frame, uint16_t destination)
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed;

    fx->feedAndSettle(frame);

    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxAcknowledges());
    TEST_ASSERT_TRUE(fx->written({(uint8_t)(U_ACKN_REQ | U_ACKN_REQ_ADDRESSED)}));
    TEST_ASSERT_EQUAL_HEX16(destination, fx->_lastAckDestination);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ADDRESSED) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK) != 0);
}

static void test_addressed_frame_is_acknowledged()
{
    expectAddressedAcknowledge(standardFrame(0x1101, 0x0801), 0x0801);
}

static void test_extended_addressed_frame_is_acknowledged()
{
    expectAddressedAcknowledge(extendedFrame(0x1101, 0x0A02), 0x0A02);
}

static void test_unaddressed_frame_is_not_acknowledged()
{
    fx->connect();
    fx->_ackAnswer = AckType::None;

    // OHNE ZEITRASTER, also mit vollem Rückstand: das Telegramm liegt beim ersten Tick schon komplett im
    // Interface. Damit läuft dieser Fall genau in die Bedingung von test_acknowledge_is_suppressed_when_behind
    // hinein - und darf trotzdem NICHTS zählen. Der Callback wird vor der Rückstandsprüfung gefragt
    // (Receiver::sendAcknowledge), ein abgelehntes Telegramm kommt dort also gar nicht an.
    //
    // Das ist keine Feinheit, sondern das, was den Zähler überhaupt lesbar macht: zählte er mit, zeigte ein
    // Gerät ohne eigene Adressen Rückstände in Höhe des gesamten Busverkehrs an, und ein echter Rückstand
    // wäre darin nicht mehr zu finden.
    fx->feed(standardFrame(), 0);
    fx->pump(6);

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTxAcknowledges());
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTxAcknowledgesSuppressed());
    TEST_ASSERT_EQUAL(0, fx->_interface.writtenBytes().size());
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ADDRESSED) == 0);
}

// Liegt schon das ganze Frame im Interface, ist es auf dem Bus längst durch - eine Quittung käme zu spät
// und würde vom Chip dem NÄCHSTEN Telegramm zugeordnet. Dann lieber gar keine.
static void test_acknowledge_is_suppressed_when_behind()
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed;

    // Ohne Zeitraster: alle Bytes sind sofort verfügbar, die Verarbeitung hinkt also hinterher.
    fx->feed(standardFrame(), 0);
    fx->pump(6);

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTxAcknowledges());
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxAcknowledgesSuppressed());

    // Gemeldet wird das Telegramm trotzdem, und ADDRESSED bleibt: an wen es gerichtet war, ändert die
    // ausgefallene Quittung nicht.
    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ADDRESSED) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK) == 0);
}

// NACK und BUSY sind keine Sonderfälle des Sendens, sondern eigene Aussagen über das Telegramm - sie
// müssen als Bits am gemeldeten Frame ankommen. Geprüft wird beides zugleich: was auf der Leitung steht
// (der Dienst trägt die Art der Quittung in seinen unteren Bits) und was oben ankommt.
static void expectOwnAcknowledge(AckType answer, uint8_t expectedRequest, uint8_t expectedFlags)
{
    fx->connect();
    fx->_ackAnswer = answer;

    fx->feedAndSettle(standardFrame());

    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxAcknowledges());
    TEST_ASSERT_TRUE(fx->written({expectedRequest}));

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_EQUAL_HEX8(expectedFlags, fx->_frames[0]._flags);
}

static void test_nack_is_sent_and_flagged()
{
    expectOwnAcknowledge(AckType::Nack,
                         U_ACKN_REQ | U_ACKN_REQ_ADDRESSED | U_ACKN_REQ_NACK,
                         TP_FRAME_FLAG_ADDRESSED | TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_NACK);
}

static void test_busy_is_sent_and_flagged()
{
    expectOwnAcknowledge(AckType::Busy,
                         U_ACKN_REQ | U_ACKN_REQ_ADDRESSED | U_ACKN_REQ_BUSY,
                         TP_FRAME_FLAG_ADDRESSED | TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_BUSY);
}

// DIE QUITTUNG VOM BUS, und die gibt es nur im Busmonitor: dort ist der Chip transparent und reicht auch
// das Quittungsbyte durch, das ein ANDERES Gerät gesendet hat (NCN5130 Figure 35).
//
// Zwei Dinge unterscheiden diesen Fall von der eigenen Quittung. Erstens sind die beiden Bitpaare im
// Quittungsbyte invertiert zu lesen - gesetzte Maskenbits heißen "nicht nack" bzw. "nicht busy", weshalb
// 0xCC das positive ACK ist, 0x0C ein NACK und 0xC0 ein BUSY. Und zweitens darf ADDRESSED NICHT
// mitkommen: das Telegramm ging nicht an uns, wir haben nur zugesehen. Deshalb wird hier das ganze
// Flag-Byte verglichen und nicht nur einzelne Bits - so fällt ein fälschlich gesetztes ADDRESSED auf.
//
// ackByte < 0 heißt: es kommt keine Quittung. Dann muss die Frist ablaufen und das Telegramm ohne
// Quittungs-Bits gemeldet werden.
static void expectMonitoredAcknowledge(int ackByte, uint8_t expectedFlags)
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed; // im Busmonitor wirkungslos - genau das wird mitgeprüft

    TEST_ASSERT_TRUE(fx->_dll.startMonitoring());
    fx->pump(5);
    fx->_interface.clearWrittenBytes();

    std::vector<uint8_t> stream = standardFrame();
    if (ackByte >= 0) stream.push_back((uint8_t)ackByte);

    fx->feed(stream);

    // Reichlich Zuschlag: bleibt die Quittung aus, wartet der Empfänger TPUART_FRAME_ACK_US (4ms) ab,
    // bevor er das Telegramm abschließt.
    fx->pump((uint32_t)((stream.size() * 1354) / 1000) + 8);

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_EQUAL_HEX8(expectedFlags, fx->_frames[0]._flags);
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTxAcknowledges());
}

static void test_monitor_frame_without_acknowledge()
{
    expectMonitoredAcknowledge(-1, 0);
}

static void test_monitor_frame_with_acknowledge()
{
    expectMonitoredAcknowledge(0xCC, TP_FRAME_FLAG_ACK);
}

static void test_monitor_frame_with_nack()
{
    expectMonitoredAcknowledge(0x0C, TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_NACK);
}

static void test_monitor_frame_with_busy()
{
    expectMonitoredAcknowledge(0xC0, TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_BUSY);
}

// Im Busmonitor ist der Chip transparent - ein U_Ackn.req von uns hätte dort nichts verloren.
static void test_bus_monitor_never_acknowledges()
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed;

    TEST_ASSERT_TRUE(fx->_dll.startMonitoring());
    fx->pump(5);
    fx->_interface.clearWrittenBytes();

    fx->feedAndSettle(standardFrame());

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTxAcknowledges());
}

// Bringt die Vorrichtung in den Busmonitor und räumt die Mitschrift ab, sodass die Fälle danach nur noch
// gegen das prüfen, was IM Modus rausgeht. Der Modus gilt erst als aktiv, wenn der Tick U_Busmon.req
// tatsächlich abgesetzt hat - deshalb das Takten dazwischen.
static void enterMonitor()
{
    fx->connect();

    TEST_ASSERT_TRUE(fx->_dll.startMonitoring());
    fx->pump(5);
    TEST_ASSERT_TRUE(fx->_dll.isBusMonitor());

    fx->_interface.clearWrittenBytes();
    fx->_errors.clear();
}

// Tabelle 11 (S. 32) führt U_L_DataStart/Cont/End im Busmonitor als "I" - der Chip ignoriert sie. Also
// gar nicht erst annehmen.
static void test_monitor_rejects_send_frame()
{
    enterMonitor();

    std::vector<uint8_t> frame = standardFrame();
    TEST_ASSERT_FALSE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));

    fx->pump(30);
    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().empty());
}

// Der interessante Fall ist NICHT das Ablehnen beim Einreihen, sondern ein Telegramm, das VOR dem
// Umschalten in die Warteschlange kam: es liegt beim Umschalten noch da und ginge ohne abort() später
// raus - und zwar dann, wenn es längst überholt ist.
static void test_monitor_drops_queued_telegram()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));

    TEST_ASSERT_TRUE(fx->_dll.startMonitoring());
    fx->pump(40);
    TEST_ASSERT_TRUE(fx->_dll.isBusMonitor());

    // Der Busmonitor wird nur per Reset verlassen. Danach darf das verworfene Telegramm NICHT nachträglich
    // auftauchen - genau das wäre der Unterschied zwischen "verworfen" und bloß "eingefroren".
    fx->_interface.clearWrittenBytes();
    TEST_ASSERT_TRUE(fx->_dll.reset());
    fx->pump(5);
    fx->_interface.addByte((char)U_RESET_IND, 0);
    fx->pump(40);

    for (uint8_t value : fx->_interface.writtenBytes())
        TEST_ASSERT_TRUE((value & 0x80) == 0); // kein U_L_Data*-Positionsbyte
}

// Ein Telegramm, das beim Umschalten bereits LÄUFT. Ohne den Abbruch schöbe der Tick die restlichen
// Oktette weiter raus und wartete am Ende auf ein L_Data.con, das nie kommt - nach dem Ablauf der Frist
// ginge ein U_Reset.req raus, der den Busmonitor beendete.
static void test_monitor_aborts_running_transmission()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame(0x1101, 0x0801, 15); // 23 Oktette, dauert mehrere Ticks
    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));

    fx->pump(4); // ein paar Oktette sind jetzt draußen, das Telegramm ist aber nicht fertig
    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().size() > 0);

    TEST_ASSERT_TRUE(fx->_dll.startMonitoring());
    fx->pump(10);
    TEST_ASSERT_TRUE(fx->_dll.isBusMonitor());

    fx->_interface.clearWrittenBytes();
    fx->pump(60);

    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().empty());

    // Die Zustandsprüfung ist der eigentliche Punkt: dass nichts mehr rausgeht, besorgt schon der Wächter
    // in Transmitter::process(). Nur Idle belegt, dass die Übertragung wirklich ABGEBROCHEN wurde und nicht
    // bloß eingefroren - sonst liefe sie nach dem Reset als halbes Telegramm weiter.
    TEST_ASSERT_EQUAL(TxState::Idle, fx->_dll.getTransmitter().state());
}

// U_State.req ist im Busmonitor "I": ignoriert, OHNE Antwort. Die Abfrage wäre also nicht nur wirkungslos,
// sie verunreinigte auch eine Spur, die passiv sein soll.
static void test_monitor_suppresses_state_request()
{
    enterMonitor();

    TEST_ASSERT_FALSE(fx->_dll.requestState());

    fx->pump(TPUART_STATE_INTERVAL_MS + 200); // mehr als ein Abfrageintervall
    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().empty());
}

// U_SetBusy.req/U_QuitBusy.req sind im Busmonitor "I". Busmon an heißt Busy aus.
static void test_monitor_rejects_busy_mode()
{
    enterMonitor();

    TEST_ASSERT_FALSE(fx->_dll.busyMode(true));

    fx->pump(30);
    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().empty());
}

// Busmon an heißt Busy aus - der Chip nimmt im Modus keine Telegramme mehr an, ein Modus, der das
// Quittieren nur ersetzt, kann es dort nicht geben.
//
// Der zweite Teil ist der wichtigere: OHNE die Abmeldung liefe checkBusyMode() in eine Sackgasse. Nach
// Ablauf der Frist ruft es busyMode(false), und das lehnt im Busmonitor ab - _busyModeSince bliebe stehen
// und der Versuch wiederholte sich bei jedem loop(), endlos.
static void test_monitor_clears_busy_mode()
{
    fx->connect();

    TEST_ASSERT_TRUE(fx->_dll.busyMode(true));
    fx->pump(5);
    TEST_ASSERT_TRUE(fx->_dll.isBusyMode());

    TEST_ASSERT_TRUE(fx->_dll.startMonitoring());
    fx->pump(5);
    TEST_ASSERT_TRUE(fx->_dll.isBusMonitor());

    fx->_interface.clearWrittenBytes();
    fx->pump(TPUART_BUSY_MODE_MS + 100); // über die Frist hinaus

    TEST_ASSERT_FALSE(fx->_dll.isBusyMode());
    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().empty()); // kein U_QuitBusy.req in die passive Spur
}

// Im Busmonitor ist der Chip durchsichtig und quittiert nichts. Das Flag beschreibt den Chip, nicht
// unseren Wunsch - es muss also mit umschalten.
static void test_monitor_clears_auto_acknowledge()
{
    fx->connect();

    TEST_ASSERT_TRUE(fx->_dll.setOwnAddress(0x1101));
    fx->pump(20);
    TEST_ASSERT_TRUE(fx->_dll.isAutoAcknowledge());

    TEST_ASSERT_TRUE(fx->_dll.startMonitoring());
    fx->pump(5);

    TEST_ASSERT_FALSE(fx->_dll.isAutoAcknowledge());
}

// U_SetAddress.req ist im Busmonitor "I". Die Konfiguration wird deshalb NICHT abgesetzt - aber auch nicht
// verworfen: die Epoche bleibt offen, und nach dem Verlassen holt der erste loop() sie nach. Ginge sie
// verloren, verlöre das Gerät stillschweigend die Quittung für alles, was an seine Adresse geht.
static void test_monitor_defers_configuration()
{
    enterMonitor();

    // FALSE ist hier richtig und heißt "jetzt nicht abgesetzt", nicht "abgelehnt": der Wert ist gemerkt
    // und die Epoche offen. Dieselbe Antwort gibt setOwnAddress() auch, wenn die Steuerwarteschlange
    // gerade voll war - der zweite Teil dieses Falls prüft, dass darauf wirklich das Nachholen folgt.
    TEST_ASSERT_FALSE(fx->_dll.setOwnAddress(0x1101));
    TEST_ASSERT_EQUAL_HEX16(0x1101, fx->_dll.ownAddress());

    fx->pump(30);
    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().empty());

    TEST_ASSERT_TRUE(fx->_dll.reset());
    fx->pump(5);
    fx->_interface.addByte((char)U_RESET_IND, 0);
    fx->pump(40);

    // Jetzt muss die Adresse draußen sein - nachgeholt, nicht vergessen.
    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();
    bool found = false;
    for (size_t i = 0; i + 3 < written.size(); i++)
        if (written[i] == U_NCN5120_SET_ADDRESS_REQ && written[i + 1] == 0x11 && written[i + 2] == 0x01) found = true;

    TEST_ASSERT_TRUE(found);
}

// DER FALL, DER DEN MODUS SONST NACH FÜNF SEKUNDEN SELBST BEENDET. Die Verbindungsüberwachung ruht auf
// zwei Beinen - Busverkehr und der sekündlichen Statusabfrage -, und im Busmonitor bricht das zweite weg.
// Auf einem ruhigen Bus liefe die Frist deshalb zwangsläufig ab, der Reconnect schickte U_Reset.req, und
// der beendet den Busmonitor.
//
// Der zweite Teil prüft das Verlassen: _lastReceivedAt ist nach der Stille beliebig alt, zwischen unserem
// U_Reset.req und der U_Reset.ind darf trotzdem kein Abbruch gemeldet werden.
static void test_monitor_survives_silence_and_exits_cleanly()
{
    enterMonitor();

    fx->pump(TPUART_CONNECTION_TIMEOUT_MS + 500); // länger als die Frist, ohne ein einziges Byte

    TEST_ASSERT_TRUE(fx->_dll.isConnected());
    TEST_ASSERT_TRUE(fx->_dll.isBusMonitor());
    TEST_ASSERT_TRUE(fx->_interface.writtenBytes().empty()); // kein Reconnect-Reset

    TEST_ASSERT_TRUE(fx->_dll.reset());
    fx->pump(5);
    fx->_interface.addByte((char)U_RESET_IND, 0);
    fx->pump(20);

    TEST_ASSERT_FALSE(fx->_dll.isBusMonitor());
    TEST_ASSERT_TRUE(fx->_dll.isConnected());
    TEST_ASSERT_EQUAL(0, fx->_errors.size());
}

// ---------------------------------------------------------------------------------------------------
// Senden
// ---------------------------------------------------------------------------------------------------

// Der Aufrufer übergibt ein VOLLSTÄNDIGES Telegramm einschließlich Prüfsumme - die Library prüft sie und
// rechnet sie nicht mehr selbst. Auf der Leitung steht danach U_L_DataStart, je ein U_L_DataCont und zum
// Schluss U_L_DataEnd.
static void test_send_frame_produces_correct_sequence()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));
    fx->pump(40);

    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();

    // Je Oktett zwei Hostbytes (Dienst mit Index, dann das Datenbyte) - und davor EIN Offset-Byte:
    // beginTransmission() setzt _chipOffsetValid zurück, das erste Oktett trägt seinen Offset also mit.
    // Bei neun Oktetten bleibt es bei diesem einen, der nächste fiele erst bei Position 64 an.
    TEST_ASSERT_EQUAL(1 + frame.size() * 2, written.size());
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_OFFSET_REQ | 0, written[0]);
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_START_REQ | 0, written[1]);
    TEST_ASSERT_EQUAL_HEX8(frame[0], written[2]);

    // Im U_L_DataEnd steht die Zahl der DATEN-Oktette; das Prüfsummen-Byte folgt dahinter und zählt nicht
    // als indiziertes Oktett mit.
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_END_REQ | (uint8_t)(frame.size() - 1), written[written.size() - 2]);
    // Genau das Prüfsummen-Byte, das der Aufrufer mitgegeben hat - es wird nicht mehr neu gerechnet.
    TEST_ASSERT_EQUAL_HEX8(frame.back(), written[written.size() - 1]);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxFrames());
}

// Die vier Größen-/Formatkombinationen auf der Sendeseite. Geprüft wird jeweils die vollständige Folge
// auf der Leitung und - der eigentliche Punkt - die ZAHL DER OFFSET-BYTES: der Dienst trägt nur sechs Bit
// Position, alles darüber muss per U_L_DataOffset.req nachgereicht werden. Sparsam heißt: nur beim
// Wechsel, nicht bei jedem Oktett.
static void expectSendSequence(const std::vector<uint8_t> &frame, size_t expectedOffsets)
{
    fx->connect();

        TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));

    fx->pump((uint32_t)(frame.size() * 2 + 60));

    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();

    TEST_ASSERT_EQUAL(frame.size() * 2 + expectedOffsets, written.size());
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_OFFSET_REQ | 0, written[0]);
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_START_REQ | 0, written[1]);
    TEST_ASSERT_EQUAL_HEX8(frame[0], written[2]);
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_END_REQ | (uint8_t)(frame.size() - 1), written[written.size() - 2]);
    TEST_ASSERT_EQUAL_HEX8(frame.back(), written.back());
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxFrames());
}

static void test_send_standard_minimal()
{
    expectSendSequence(standardFrame(0x1101, 0x0801, 1), 1); // 9 Oktette
}

// Das Längste, was das Standard-Format hergibt: 8 + 15 = 23 Oktette. Immer noch weit unter 64, also
// bleibt es bei einem einzigen Offset-Byte.
static void test_send_standard_maximal()
{
    std::vector<uint8_t> frame = standardFrame(0x1101, 0x0801, 15);
    TEST_ASSERT_EQUAL(23, frame.size());

    expectSendSequence(frame, 1);
}

static void test_send_extended_minimal()
{
    expectSendSequence(extendedFrame(0x1101, 0x0801, 1), 1); // 10 Oktette
}

// DAS GRÖSSTMÖGLICHE TELEGRAMM, und hier zahlt sich die Sparsamkeit aus: bei 263 Oktetten wechselt der
// Offset an den Positionen 0, 64, 128, 192 und 256, macht FÜNF Offset-Bytes. Würde er pauschal bei jedem
// Oktett mitgeschickt - drei Bytes je Oktett statt zwei -, wären es 263 statt 5, also 789 Hostbytes statt
// 531. Bei 38400 Baud sind das 74ms Unterschied, die vor jedem Senden anfallen. Genau deshalb prüft
// process() zur Laufzeit, ob der Offset überhaupt nötig ist.
static void test_send_extended_maximal_uses_offsets_sparingly()
{
    std::vector<uint8_t> frame = extendedFrame(0x0E01, 0xF000, 254);
    TEST_ASSERT_EQUAL(263, frame.size());

    expectSendSequence(frame, 5);

    // Und sie stehen an den richtigen Stellen: der Wechsel auf Offset 1 liegt hinter 64 Oktetten, also bei
    // 1 + 64 * 2; der nächste 129 Bytes später. Die Längenprüfung davor ist kein Selbstzweck - ein
    // Direktzugriff hinter das Ende wäre auf dem Ziel kein Testfehler, sondern ein Absturz.
    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();
    TEST_ASSERT_GREATER_THAN(258u, written.size());
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_OFFSET_REQ | 1, written[129]);
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_OFFSET_REQ | 2, written[258]);
}

// ---------------------------------------------------------------------------------------------------
// Echo, Bestätigung und Wachhund
// ---------------------------------------------------------------------------------------------------
//
// Was der Chip mit einem übergebenen Telegramm macht, sieht der Host an zwei Dingen: er bekommt JEDES
// Oktett zurück, während es auf den Bus geht (das Echo), und danach ein L_Data.con mit dem Ergebnis.
// Wiederholt der Chip nach einem NACK oder BUSY, kommt das Echo erneut - bis zu dreimal. Wie es dabei
// im Einzelnen lief, erfährt der Host NICHT: die Zwischenschritte bleiben ihm verborgen, nur das
// abschließende L_Data.con sagt, ob es am Ende geklappt hat.
//
// Der Helfer spielt das durch: senden, das Echo so oft zurückspiegeln wie angegeben, dann optional
// bestätigen. Zwischen den Wiederholungen liegt jeweils mehr als die halbe Wachhund-Frist - damit ist die
// Auffrischung durch das Echo mitgeprüft und nicht nur behauptet: ohne sie liefe die Frist während der
// Wiederholungen ab und der Wachhund würde mitten in eine laufende Übertragung greifen.
static void sendWithEcho(const std::vector<uint8_t> &frame, uint8_t echoCount, int confirmByte)
{
        TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));

    fx->pump((uint32_t)(frame.size() * 2 + 60));
    fx->_interface.clearWrittenBytes();

    // DER GANZE ABLAUF WIRD ALS EIN SKRIPT GESTELLT und danach in einem Zug durchgetaktet. Der erste
    // Versuch hat die Bestätigung stattdessen nach einer Pumprunde nachgeschoben - und damit zu spät:
    // nach einem erkannten Echo wartet der Empfänger nur TPUART_FRAME_ACK_US (4ms) auf das L_Data.con,
    // danach meldet er das Telegramm ohne Bestätigungs-Flags. Die Quittung muss also DIREKT hinter dem
    // letzten Echo stehen, im normalen Zeichenabstand.
    constexpr uint32_t REPEAT_GAP_US = 600000; // zwischen Wiederholungen: über die halbe Wachhund-Frist

    for (uint8_t i = 0; i < echoCount; i++)
    {
        bool last = (i + 1 == echoCount);
        uint32_t trailing = last ? 1354 : REPEAT_GAP_US;

        if (i == 0)
            fx->feed(frame, 1354, trailing);
        else
            fx->append(frame, 1354, trailing);
    }

    if (confirmByte >= 0)
    {
        uint8_t con = (uint8_t)confirmByte;
        fx->append(&con, 1);
    }

    uint32_t scriptMs = (uint32_t)((frame.size() * echoCount * 1354) / 1000) + (echoCount - 1) * (REPEAT_GAP_US / 1000);
    fx->pump(scriptMs + 60);
}

// MAN QUITTIERT NICHT SICH SELBST. Der Chip spiegelt jedes gesendete Oktett zurück, das eigene Telegramm
// läuft also wie ein fremdes ein - und ein Aufrufer, der auf die eigene Zieladresse hört, würde beim
// Quittungs-Callback "meins" antworten. Genau das ist an einem echten IP-Router gemessen worden: 4
// Quittungen je eigenem Telegramm, keine einzige für ein fremdes.
//
// Der Callback antwortet hier deshalb bewusst bejahend - ohne das prüft der Fall gar nichts, denn mit
// AckType::None (der Vorgabe der Vorrichtung) endet die Kette schon vor der Echo-Erkennung.
static void test_own_echo_is_not_acknowledged()
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed;

    // sendWithEcho() löscht die geschriebenen Bytes, NACHDEM das Telegramm draußen ist - was danach noch
    // dazukommt, kann nur die Quittung zum eintreffenden Echo sein.
    sendWithEcho(standardFrame(), 1, 0x8B);

    TEST_ASSERT_EQUAL(0, fx->_interface.writtenBytes().size());
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTxAcknowledges());
}

// OHNE BESTÄTIGUNG TRÄGT DAS EIGENE TELEGRAMM KEINE QUITTUNGSFLAGS - weder ADDRESSED noch ACK. ADDRESSED
// heißt "wir sind dafür zuständig", und für ein Telegramm, das wir selbst gesendet haben, sind wir der
// Absender und nicht der zuständige Empfänger. ACK heißt "es wurde quittiert", und ohne L_Data.con weiß
// niemand, ob das geschah.
static void test_own_echo_without_confirmation_has_no_ack_flags()
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed;

    sendWithEcho(standardFrame(), 1, -1); // kein L_Data.con

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_TX) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ADDRESSED) == 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK) == 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_DATA_CON) == 0);
}

// MIT BESTÄTIGUNG KOMMT ACK - aber weiterhin NICHT ADDRESSED. Das ist der Punkt, an dem die beiden Flags
// auseinanderfallen: die Quittung ist da (jemand auf dem Bus hat quittiert, gemeldet über L_Data.con),
// zuständig für den Empfang war trotzdem nicht dieses Gerät.
static void test_own_echo_with_confirmation_is_acked_but_not_addressed()
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed;

    sendWithEcho(standardFrame(), 1, 0x8B);

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_DATA_CON) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ADDRESSED) == 0);
}

// DER WÄCHTER GEGEN DIE NAHELIEGENDE FEHLIMPLEMENTIERUNG. Die Echo-Erkennung darf NICHT "läuft gerade eine
// Übertragung" heißen: TxState::Await hält bis zur Bestätigung an, und in dieser ganzen Zeit würde kein
// einziges FREMDES Telegramm mehr quittiert - auf einem belegten Bus sind das leicht hunderte Millisekunden.
//
// Hier wartet der Sendeweg also auf sein L_Data.con, und mitten hinein kommt ein fremdes Telegramm (andere
// Quelladresse). Es muss ganz normal quittiert werden.
static void test_foreign_frame_is_acknowledged_while_awaiting_confirmation()
{
    fx->connect();
    fx->_ackAnswer = AckType::Addressed;

    std::vector<uint8_t> own = standardFrame();
    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(own.data(), own.size()));
    fx->pump((uint32_t)(own.size() * 2 + 60));

    // Der Sendeweg hängt jetzt in Await - genau der Zustand, den eine Zustandsprüfung fälschlich als
    // "nicht quittieren" lesen würde.
    TEST_ASSERT_EQUAL(TxState::Await, fx->_dll.getTransmitter().state());
    fx->_interface.clearWrittenBytes();

    // Andere Quelladresse, also kein Echo. Zieladresse ebenfalls anders, damit auch der Anfang abweicht.
    std::vector<uint8_t> foreign = standardFrame(0x2202, 0x0901);
    fx->feedAndSettle(foreign);

    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxAcknowledges());
    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ADDRESSED) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_TX) == 0);
}

// Der Normalfall: ein Echo, dann die positive Bestätigung. Das Telegramm wird als eigenes gemeldet (TX),
// mit Bestätigung (DATA_CON) und positiv quittiert (ACK, ohne NACK). Danach ist der Sendeweg frei.
static void test_echo_and_positive_confirmation()
{
    fx->connect();
    sendWithEcho(standardFrame(), 1, 0x8B);

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_TX) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_DATA_CON) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK_NACK) == 0);

    TEST_ASSERT_EQUAL(TxState::Idle, fx->_dll.getTransmitter().state());
}

// EIN NEGATIVES L_Data.con IST DIE EINZIGE AUSKUNFT ÜBER EIN SCHEITERN, und es trägt genau ein Bit: der
// Chip hat seine Wiederholungen selbst abgearbeitet - bis zu dreimal nach NACK, dreimal nach BUSY - und
// meldet nur das Endergebnis. Deshalb kommt hier ACK_NACK und NIEMALS ACK_BUSY: welche der beiden
// Absagen es war, weiß der Host nicht. Die Unterscheidung gibt es nur im Busmonitor, wo das rohe
// Quittungsbyte durchkommt (siehe die vier Fälle dort).
static void test_echo_and_negative_confirmation_flags_nack()
{
    fx->connect();
    sendWithEcho(standardFrame(), 1, 0x0B);

    TEST_ASSERT_EQUAL(1, fx->_frames.size());
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_DATA_CON) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK_NACK) != 0);
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_ACK_BUSY) == 0);

    TEST_ASSERT_EQUAL(TxState::Idle, fx->_dll.getTransmitter().state());
}

// DREI WIEDERHOLUNGEN, also vier Echos insgesamt, und dazwischen jeweils mehr als die halbe Frist. Der
// Wachhund darf dabei nicht zuschlagen - jedes Echo setzt seine Frist neu auf, weil der Chip damit ja
// beweist, dass er noch am Senden ist. Ohne diese Auffrischung käme hier ein U_Reset.req dazwischen.
static void test_echo_repetitions_keep_watchdog_quiet()
{
    fx->connect();
    sendWithEcho(standardFrame(), 4, 0x8B);

    // Vier gemeldete Telegramme (jedes Echo ist eines), aber kein einziges Reset-Byte auf der Leitung.
    TEST_ASSERT_EQUAL(4, fx->_frames.size());

    // UND DAS ERGEBNIS HÄNGT AM LETZTEN. Der Chip bestätigt erst, wenn er fertig ist - die
    // Zwischenversuche tragen deshalb keine Bestätigungs-Flags, und das ist richtig so: zu ihnen gibt es
    // noch kein Ergebnis. Das L_Data.con folgt dem letzten Echo im normalen Zeichenabstand und landet
    // damit innerhalb des Quittungsfensters.
    TEST_ASSERT_TRUE((fx->_frames[0]._flags & TP_FRAME_FLAG_DATA_CON) == 0);
    TEST_ASSERT_TRUE((fx->_frames[2]._flags & TP_FRAME_FLAG_DATA_CON) == 0);
    TEST_ASSERT_TRUE((fx->_frames[3]._flags & TP_FRAME_FLAG_DATA_CON) != 0);
    TEST_ASSERT_TRUE((fx->_frames[3]._flags & TP_FRAME_FLAG_ACK) != 0);

    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();
    for (size_t i = 0; i < written.size(); i++)
        TEST_ASSERT_NOT_EQUAL_MESSAGE(U_RESET_REQ, written[i], "Wachhund hat waehrend der Wiederholungen zugeschlagen");

    TEST_ASSERT_EQUAL(TxState::Idle, fx->_dll.getTransmitter().state());
}

// DIE BESTÄTIGUNG KOMMT AUCH OHNE ERKANNTES ECHO AN - nur eben nicht mehr am Telegramm.
//
// Das ist der Fall, den handleControlEntry() beschreibt: ist das Echo verstümmelt, im Resync verlorenoder
// steht der Empfänger aus anderem Grund nicht mehr auf FrameAck, landet das L_Data.con als gewöhnliche
// Steuerbyte-Sequenz. Hier nachgestellt, indem gar kein Echo eingespeist wird.
//
// WICHTIG IST, WAS DABEI TROTZDEM PASSIERT: der Sendeweg wird freigegeben. Das entscheidet
// Receiver::processControlByte() und nicht der Frame-Pfad - sonst bliebe der Weg bis zum Wachhund belegt,
// obwohl die BCU längst geantwortet hat. Verloren geht nur die Auskunft AM TELEGRAMM; als Aussage über
// den Empfangsweg ist das gewollt, nicht als Verlust der Bestätigung.
static void test_confirmation_without_echo_still_releases_path()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    
    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));
    fx->pump((uint32_t)(frame.size() * 2 + 60));
    fx->_interface.clearWrittenBytes();

    uint8_t con = 0x8B;
    fx->feed(&con, 1);
    fx->pump(20);

    TEST_ASSERT_EQUAL(0, fx->_frames.size()); // kein Echo, also auch kein gemeldetes Telegramm
    TEST_ASSERT_EQUAL(TxState::Idle, fx->_dll.getTransmitter().state());

    // Und der Wachhund bleibt still - es gibt ja nichts mehr aufzuräumen.
    fx->pump(1500);

    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();
    for (size_t i = 0; i < written.size(); i++)
        TEST_ASSERT_NOT_EQUAL_MESSAGE(U_RESET_REQ, written[i], "Reset trotz eingegangener Bestaetigung");
}

// NACH EINEM VERSTÜMMELTEN ECHO HÄNGT ALLES AN DER PAUSE, und zwar an einer Lücke, die knapp bemessen ist.
//
// Ein kaputtes Echo führt in den Resync, und dort wird JEDES Byte verworfen - auch das L_Data.con.
// Herauskommt die Maschine nur über eine verifizierte Pause (TPUART_FRAME_WAIT_US, 2800µs). Auf dem Bus
// liegen zwischen dem letzten Echo-Byte am Host und der Bestätigung aber nur rund 2,7ms: die Quittung
// beginnt 15 Bitzeiten nach dem Frame, braucht 11 Bit bei 9600 Baud, und dann reicht der Chip weiter.
//
// Die beiden Fälle unten stellen genau diese Grenze nach - einmal knapp darunter, einmal darüber.

// UNTERHALB DER SCHWELLE: keine Pause. Die Bestätigung fällt im Resync unter den Tisch, der Sendeweg
// bleibt belegt, und aufräumen kann nur noch der Wachhund. Das ist der Preis der pausenbasierten
// Erkennung an ihrer engsten Stelle - sichtbar gemacht, damit niemand später über den Wachhund-Reset
// rätselt. Und zugleich die Wache über die Schwelle selbst: würde jemand TPUART_FRAME_WAIT_US wieder
// anheben, bis 2,7ms darunterfallen, schlüge dieser Fall zusammen mit dem nächsten fehl.
static void test_confirmation_after_broken_echo_is_lost_without_pause()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    
    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));
    fx->pump((uint32_t)(frame.size() * 2 + 60));

    std::vector<uint8_t> broken = frame;
    broken.back() ^= 0xFF; // Prüfsumme kaputt -> INVALID und Resync

    uint8_t con = 0x8B;
    fx->feed(broken, 1354, 2400); // Lücke unter der Pausenschwelle (2600)
    fx->append(&con, 1);
    fx->pump(60);

    TEST_ASSERT_EQUAL(TxState::Await, fx->_dll.getTransmitter().state());
}

// UND JETZT MIT DER ECHTEN BUSZAHL: rund 2,7ms liegen zwischen dem letzten Echo-Byte am Host und dem
// L_Data.con - die Quittung beginnt 15 Bitzeiten nach dem Telegramm, braucht 11 Bit bei 9600 Baud, dann
// reicht der Chip weiter. Mit der Schwelle bei 2600 fällt diese Lücke auf die richtige Seite: die Pause
// wird erkannt, die Maschine steht wieder auf Idle, die Bestätigung kommt als Steuerbyte an und der
// Sendeweg ist frei, ohne dass der Wachhund eingreifen musste.
//
// Mit den früheren 2800 war genau das nicht der Fall. Der Fall ist damit die Probe darauf, was die
// gesenkte Schwelle wirklich bringt - und nicht bloß eine Wiederholung des Falls darüber mit anderen
// Zahlen.
static void test_confirmation_after_broken_echo_arrives_after_pause()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    
    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));
    fx->pump((uint32_t)(frame.size() * 2 + 60));

    std::vector<uint8_t> broken = frame;
    broken.back() ^= 0xFF;

    uint8_t con = 0x8B;
    fx->feed(broken, 1354, 2700); // die echte Lücke bis zum L_Data.con, jetzt über der Schwelle
    fx->append(&con, 1);
    fx->pump(60);

    TEST_ASSERT_EQUAL(TxState::Idle, fx->_dll.getTransmitter().state());
}

// BLEIBT DIE BESTÄTIGUNG AUS, ist unbekannt, was im Sendepuffer der BCU noch liegt - der einzige Weg zu
// einem definierten Zustand ist der Reset. Er kommt nach TPUART_TX_CONFIRM_TIMEOUT_MS (im Testbau auf 1s
// gesenkt) und beendet damit auch die Hängepartie eines nie bestätigten Telegramms.
static void test_missing_confirmation_triggers_reset()
{
    fx->connect();
    sendWithEcho(standardFrame(), 1, -1); // kein L_Data.con

    fx->_interface.clearWrittenBytes();
    fx->pump(1500); // Frist abwarten

    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();

    bool sawReset = false;
    for (size_t i = 0; i < written.size(); i++)
        if (written[i] == U_RESET_REQ) sawReset = true;

    TEST_ASSERT_TRUE_MESSAGE(sawReset, "Wachhund hat keinen Reset abgesetzt");

    // Der Wachhund-Fall ist der EINZIGE Sendefehler, der gezählt wird. Ein negatives L_Data.con ist
    // keiner: es sagt nur, dass auf dem Bus niemand quittiert hat - eine Aussage über den Bus, nicht über
    // die Strecke zum Chip. Hier dagegen kam gar keine Antwort.
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxConfirmTimeouts());
}

// Auf einem ruhigen Bus ohne ein einziges Byte laeuft die Verbindungsueberwachung ab. Der Zaehler macht
// sichtbar, was sonst nur als einmalige Konsolenmeldung vorbeikommt - im Feld die Antwort auf "haengt die
// BCU?".
static void test_connection_loss_is_counted()
{
    fx->connect();

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getConnectionLosses());

    fx->pump(TPUART_CONNECTION_TIMEOUT_MS + 500);

    TEST_ASSERT_FALSE(fx->_dll.isConnected());
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getConnectionLosses());
}

// Ein zu langes Telegramm wird abgelehnt, nicht abgeschnitten.
static void test_send_frame_rejects_oversized()
{
    fx->connect();

    std::vector<uint8_t> frame(300, 0x00);
    frame[0] = 0xBC;

    TEST_ASSERT_FALSE(fx->_dll.pushTransmitQueue(frame.data(), frame.size()));
}

// ---------------------------------------------------------------------------------------------------
// Schnittstelle und Statistik
// ---------------------------------------------------------------------------------------------------

// DER SENDEPUFFER LÄUFT VOLL. Über uns puffert nichts - der knx-Stack verwirft ein abgelehntes Telegramm
// und meldet ein negatives L_Data.con nach oben -, jede Ablehnung ist also unmittelbar ein verlorenes
// Telegramm. Der Zähler ist damit kein Schönheitswert.
//
// Ohne pump() holt der Tick nichts ab, der Puffer füllt sich also bis zur Grenze. standardFrame() ist Low,
// es greift damit die kleinere Grenze (siehe TPUART_TX_PRIORITY_RESERVE).
static void test_tx_queue_overflow_is_counted()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();

    int accepted = 0;
    while (fx->_dll.pushTransmitQueue(frame.data(), frame.size()))
        accepted++;

    TEST_ASSERT_GREATER_THAN(0, accepted);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxQueueOverflows());

    // Und oberhalb von Low ist trotzdem noch Platz - genau dafür gibt es die Reserve.
    std::vector<uint8_t> urgent = standardFrame();
    urgent.pop_back();                                // alte Prüfsumme runter
    urgent[0] = (uint8_t)((urgent[0] & ~0x0C) | 0x08); // Urgent statt Low
    urgent.push_back(checksum(urgent));                // und über den geänderten Inhalt neu

    TEST_ASSERT_TRUE(fx->_dll.pushTransmitQueue(urgent.data(), urgent.size()));
}

// Die Steuercode-Warteschlange ist klein (TPUART_CTRL_QUEUE_SIZE), und ein Überlauf dort verwirft einen
// Befehl an den Chip. Gemeldet wird er einmalig, gezählt bei jedem Vorkommen.
static void test_control_queue_overflow_is_counted()
{
    fx->connect();

    int accepted = 0;
    while (fx->_dll.reset()) // U_Reset.req, ein Byte je Eintrag
        accepted++;

    TEST_ASSERT_GREATER_THAN(0, accepted);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxControlQueueOverflows());
}

// DER RX-RING LÄUFT ÜBER, wenn der Hauptloop stehenbleibt, während der Bus weiterliefert. Die Politik ist
// dabei ausdrücklich "den NEUEN Eintrag verwerfen, das bereits Angenommene behalten" - hier wird geprüft,
// dass das überhaupt gemeldet wird.
//
// Nachgestellt mit tickOnly(): der Tick zerlegt und stellt ein, aber niemand holt ab. Ohne Zeitraster
// eingespeist, damit die 100 Telegramme in wenigen Millisekunden durch sind statt in anderthalb Sekunden.
static void test_rx_queue_overflow_is_counted()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    std::vector<uint8_t> stream;

    // Ein Eintrag kostet 3 Byte Kopf plus das Telegramm; in TPUART_RX_QUEUE_SIZE passen damit gut 80.
    for (int i = 0; i < 100; i++)
        stream.insert(stream.end(), frame.begin(), frame.end());

    fx->feed(stream, 0);
    fx->tickOnly(80);

    TEST_ASSERT_GREATER_THAN(0, fx->_dll.getStatistics().getRxQueueOverflows());

    // Verworfen wird der NEUE Eintrag - was schon drinsteht, bleibt lesbar. Nach einem loop() müssen die
    // frühen Telegramme also ankommen.
    fx->pump(5);
    TEST_ASSERT_GREATER_THAN(0, fx->_frames.size());
}

static void test_interface_overflow_is_counted()
{
    fx->connect();

    fx->_interface.forceOverflow(true);
    uint8_t byte = U_RESET_IND;
    fx->feed(&byte, 1);
    fx->pump(6);

    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getRxInterfaceOverflows());
}

// DER TAKT MISST SICH SELBST. Ohne diese Zähler war der Antrieb der einzige Wert der Schicht, den man
// nicht ablesen konnte - man sah nur seine Folgen und musste auf die Ursache raten.
static void test_tick_rate_is_measured()
{
    fx->connect();
    fx->_dll.getStatistics().reset();

    fx->pump(5);

    // Der Fall prüft die Größenordnung, nicht die Zahl: nativ tickt pump() in festen 50µs-Schritten (also
    // genau 100 mal), auf der Hardware so schnell wie der Prozessor durch die Schleife kommt - dort sind es
    // deutlich mehr. Beides muss durch dieselbe Zusicherung passen.
    TEST_ASSERT_GREATER_THAN(50, fx->_dll.getStatistics().getTicks());
    TEST_ASSERT_GREATER_THAN(0, fx->_dll.getStatistics().getTicks());
}

// DIE PAUSE ZWISCHEN ZWEI BETRIEBSPHASEN IST KEIN AUSSETZER. Ohne das Zurücksetzen von _tickLastUs in
// begin() zählte die Zeit zwischen end() und begin() beim ersten Tick danach als Verzögerung - und zwar
// genau auf einem Gerät, das die BCU neu verbindet.
static void test_tick_gap_survives_restart()
{
    fx->connect();
    fx->pump(5);

    fx->_dll.end();
    fx->pump(20); // hier tickt nichts - genau diese Lücke darf nicht gezählt werden
    fx->_dll.getStatistics().reset();

    fx->_dll.begin(BcuType::Ncn5120);
    fx->pump(5);

    // 20ms lägen weit über der Schwelle und wären als Verzögerung gezählt worden.
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getTickDeferrals());
}

// DER RÜCKSTAND IM INTERFACE, in seiner ursprünglichen Einheit. Kommen alle Bytes auf einmal (gapUs 0),
// liegen sie beim ersten Tick vollständig bereit - genau das Bild, das ein ausgehungerter Tick erzeugt.
static void test_interface_backlog_is_measured()
{
    fx->connect();
    fx->_dll.getStatistics().reset();

    std::vector<uint8_t> frame = standardFrame();
    fx->feed(frame, 0);
    fx->pump(1);

    TEST_ASSERT_EQUAL(frame.size(), fx->_dll.getStatistics().getRxInterfacePeakBytes());
}

// EIN ZU LANGSAMER ANTRIEB MUSS SICH MELDEN. Das ist die Lehre aus einem Fehler, der im Router Stunden
// gekostet hat: die Taktrate lag bei 457/s statt 2000/s, und sichtbar war davon ausschließlich der
// Folgeschaden - 12% unterdrückte Quittungen. Die Ursache stand nirgends.
//
// Der Wächter misst die ERREICHTE Rate, nicht die eingestellte, greift also bei jedem Antrieb. Gemeldet
// wird einmal je Störung; erholt sich die Rate, wird der Melder wieder scharf.
static void test_deferred_tick_is_reported()
{
    fx->connect();
    fx->pump(5); // setzt den Bezugspunkt des Wächters
    fx->_errors.clear();

    // Zeit vergeht ohne einen einzigen Tick - der schlimmste Fall, und zugleich der, der ohne
    // ausdrückliche Behandlung eine Division durch null wäre.
    // ZWEI Fenster, nicht eines: das erste endet mit den Ticks aus connect() im Zähler und ist damit
    // grenzwertig, und es setzt den Bezugspunkt neu. Erst das zweite ist garantiert tickfrei.
    fx->idle(TPUART_TICK_RATE_WINDOW_MS * 2 + 100);

    TEST_ASSERT_EQUAL(1, fx->_errors.size());
    TEST_ASSERT_TRUE(fx->_errors[0].find("Tick") != std::string::npos);

    // EINMAL, NICHT JE FENSTER: die Meldung ist eine Diagnose, keine Dauerbeobachtung. Bliebe sie stehen,
    // liefe die Konsole eines betroffenen Geräts voll und die eigentliche Ursache ginge darin unter.
    // ZWEI Fenster, nicht eines: das erste endet mit den Ticks aus connect() im Zähler und ist damit
    // grenzwertig, und es setzt den Bezugspunkt neu. Erst das zweite ist garantiert tickfrei.
    fx->idle(TPUART_TICK_RATE_WINDOW_MS * 2 + 100);
    TEST_ASSERT_EQUAL(1, fx->_errors.size());
}

// Läuft der Antrieb wie vorgesehen, darf der Wächter SCHWEIGEN. Ohne diesen Fall wäre ein Wächter, der
// grundsätzlich meldet, genauso "grün" wie der richtige - und im Betrieb dann reines Rauschen.
static void test_healthy_tick_is_not_reported()
{
    fx->connect();
    fx->_errors.clear();

    fx->pump(TPUART_TICK_RATE_WINDOW_MS + 100);

    TEST_ASSERT_EQUAL(0, fx->_errors.size());
}

// Reicht der Platz im Sendepuffer nicht für die ganze Steuersequenz, muss sie WARTEN statt zerteilt zu
// werden - zwischen ihren Bytes darf nichts anderes stehen.
static void test_control_sequence_is_never_split()
{
    fx->connect();
    fx->_interface.setWriteCapacity(2); // eine 4-Byte-Sequenz passt hier nie am Stück
    fx->_interface.clearWrittenBytes();

    TEST_ASSERT_TRUE(fx->_dll.setOwnAddress(0x1101)); // U_SetAddress.req, 4 Byte
    fx->pump(20);

    TEST_ASSERT_EQUAL(0, fx->_interface.writtenBytes().size());

    // Mit genug Platz geht sie dann am Stück raus. Ob dahinter noch weitere Steuercodes folgen (die
    // Konfiguration setzt auch den Wiederholungszähler), ist hier egal - geprüft wird, DASS die Sequenz
    // vollständig kommt und mit ihrem eigenen Dienst beginnt.
    fx->_interface.setWriteCapacity(4);
    fx->pump(20);

    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();
    TEST_ASSERT_GREATER_OR_EQUAL(4, written.size());
    TEST_ASSERT_EQUAL_HEX8(U_NCN5120_SET_ADDRESS_REQ, written[0]);
}

// Die Buslast zählt nur Telegramm-Bytes, keine Steuerbytes - die kommen von der BCU und nicht vom Bus.
static void test_bus_load_counts_only_frame_bytes()
{
    fx->connect();

    uint32_t before = fx->_dll.getStatistics().getRxFrameBytes();

    uint8_t control = U_RESET_IND;
    fx->feed(&control, 1);
    fx->pump(6);

    TEST_ASSERT_EQUAL(before, fx->_dll.getStatistics().getRxFrameBytes());

    fx->feedAndSettle(standardFrame());

    TEST_ASSERT_EQUAL(before + 9, fx->_dll.getStatistics().getRxFrameBytes());
}

// ---------------------------------------------------------------------------------------------------
// Der zentrale Timer
// ---------------------------------------------------------------------------------------------------
//
// GEPRÜFT WIRD DIE BUCHFÜHRUNG, NICHT DER TAKT. Ob wirklich ein Hardware-Timer läuft, hängt an der
// Plattform (Timer::supported()) und ist im nativen Lauf gar nicht gegeben - deshalb ist das Verzeichnis
// bewusst plattformunabhängig gebaut: eingetragen wird immer, und nur der Rückgabewert von add() sagt, ob
// tatsächlich getickt wird. Ohne diese Trennung wäre hier nichts prüfbar.
//
// Die Vorrichtung hält den Takt auf 0, jeder Fall beginnt also mit angehaltenem Timer - aber mit einer
// Instanz, die begin() eingetragen hat, sobald connect() gelaufen ist.

// begin() TRÄGT EIN, end() TRÄGT AUS - ohne Nachfrage, denn einen Schalter je Instanz gibt es bewusst
// nicht. Das ist der Normalfall und die Grundlage aller weiteren Fälle.
static void test_timer_registers_on_begin_and_releases_on_end()
{
    Timer &timer = Timer::instance();

    TEST_ASSERT_FALSE(timer.contains(fx->_dll));
    TEST_ASSERT_EQUAL(0, timer.clients());

    fx->connect();

    TEST_ASSERT_TRUE(timer.contains(fx->_dll));
    TEST_ASSERT_EQUAL(1, timer.clients());

    fx->_dll.end();

    TEST_ASSERT_FALSE(timer.contains(fx->_dll));
    TEST_ASSERT_EQUAL(0, timer.clients());
}

// ZWEIMAL EINTRAGEN BELEGT EINEN PLATZ, nicht zwei. Ein zweites begin() ist nichts Exotisches - jeder
// Neustart der Verbindung geht darüber -, und mit TPUART_TIMER_MAX_CLIENTS = 1 wäre danach sonst Schluss.
static void test_timer_registration_is_idempotent()
{
    Timer &timer = Timer::instance();

    fx->connect();
    TEST_ASSERT_EQUAL(1, timer.clients());

    timer.add(fx->_dll);
    fx->_dll.begin(BcuType::Ncn5120);

    TEST_ASSERT_EQUAL(1, timer.clients());
}

// MEHR INSTANZEN ALS PLÄTZE: die überzählige wird ABGEWIESEN, nicht stillschweigend eingetauscht - die
// bereits eingetragene läuft weiter. Die abgewiesene wird gar nicht getickt, und usesTimer() sagt es.
//
// Der Fall hängt an der Vorgabe TPUART_TIMER_MAX_CLIENTS = 1. Wer sie hochsetzt, muss ihn mitziehen -
// deshalb steht die Zahl hier ausdrücklich in einer Zusicherung und nicht bloß im Kommentar.
static void test_timer_rejects_more_clients_than_slots()
{
    TEST_ASSERT_EQUAL(1, TPUART_TIMER_MAX_CLIENTS);

    Timer &timer = Timer::instance();

    fx->connect();
    TEST_ASSERT_EQUAL(1, timer.clients());

    Interface::Dummy second;
    DataLinkLayer secondDll(second);

    TEST_ASSERT_FALSE(timer.add(secondDll));
    TEST_ASSERT_FALSE(timer.contains(secondDll));

    // Die erste bleibt unangetastet - das ist der Punkt.
    TEST_ASSERT_TRUE(timer.contains(fx->_dll));
    TEST_ASSERT_EQUAL(1, timer.clients());
}

// DER ZERSTÖRER MUSS AUSTRAGEN. Der Timer hält einen ZEIGER; überlebte er die Instanz, liefe der nächste
// Tick in freigegebenen Speicher - auf dem RP2040 aus einem Interrupt heraus. end() zu verlangen genügt
// nicht, denn niemand ist verpflichtet, es vor dem Wegwerfen zu rufen.
static void test_timer_slot_is_released_on_destruction()
{
    Timer &timer = Timer::instance();

    // Der einzige Platz muss frei sein, sonst wird die zweite Instanz gar nicht erst eingetragen.
    TEST_ASSERT_EQUAL(0, timer.clients());

    {
        Interface::Dummy scoped;
        DataLinkLayer scopedDll(scoped);

        timer.add(scopedDll);
        TEST_ASSERT_TRUE(timer.contains(scopedDll));
        TEST_ASSERT_EQUAL(1, timer.clients());
    }

    TEST_ASSERT_EQUAL(0, timer.clients());
}

// DAS INTERVALL IST GLOBAL, nicht je Instanz - eine Zahl, eine Stelle. Geprüft wird auch der Weg zurück:
// 0 hält den Timer an, ein Wert danach ist wieder gültig.
//
// Ohne eingetragene Instanz, damit der Fall auf echter Hardware keinen laufenden Timer erzeugt, der gegen
// die übrigen Fälle antickt.
static void test_timer_interval_is_global()
{
    Timer &timer = Timer::instance();

    TEST_ASSERT_EQUAL(0, timer.clients());
    TEST_ASSERT_EQUAL(0, timer.interval());

    timer.setInterval(250);
    TEST_ASSERT_EQUAL(250, timer.interval());
    TEST_ASSERT_FALSE(timer.running()); // ohne Instanz läuft er nicht an

    timer.setInterval(0);
    TEST_ASSERT_EQUAL(0, timer.interval());
    TEST_ASSERT_FALSE(timer.running());
}

// DER HAUPTLOOP TICKT NIE - process() ruft ausschliesslich loop(). Das ist eine ausdrueckliche Festlegung
// und keine Auslegungsfrage: ein Rueckfall auf den Hauptloop macht genau den Zustand zum stillen
// Normalfall, der im Router 457 Ticks/s statt 2000 ergab und 12% der Quittungen gekostet hat. Er sieht aus
// wie "es laeuft", und niemand erfaehrt, dass der Antrieb fehlt.
//
// Der Fall haelt das fest, damit es niemand als fehlenden Rueckfall "repariert".
static void test_process_never_ticks()
{
    fx->connect();

    TEST_ASSERT_FALSE(fx->_dll.usesTimer());

    uint32_t before = fx->_dll.getStatistics().getTicks();
    fx->_dll.process();
    fx->_dll.process();

    TEST_ASSERT_EQUAL(before, fx->_dll.getStatistics().getTicks());
}

// VON HAND TREIBEN: trigger() tut genau das, was der plattformeigene Callback tut - ein tick() je
// eingetragener Instanz. Das ist der Weg für eine Plattform ohne Timer-Unterstützung, und der Aufrufer
// ruft dann loop() statt process().
static void test_timer_trigger_ticks_registered_clients()
{
    fx->connect();

    uint32_t before = fx->_dll.getStatistics().getTicks();
    Timer::instance().trigger();

    TEST_ASSERT_EQUAL(before + 1, fx->_dll.getStatistics().getTicks());

    // Nach dem Austragen darf trigger() die Instanz NICHT mehr anfassen - sonst wäre das Austragen
    // wirkungslos, und genau daran hängt die Sicherheit des Destruktors.
    Timer::instance().remove(fx->_dll);

    uint32_t after = fx->_dll.getStatistics().getTicks();
    Timer::instance().trigger();

    TEST_ASSERT_EQUAL(after, fx->_dll.getStatistics().getTicks());
}

// DIE BUSLAST IN PROZENT IST EINE ZEITRECHNUNG, keine umgerechnete Bytezahl - und genau das nagelt dieser
// Fall fest. Die Telegrammzahl geht mit ein, weil vor jedem Telegramm 50 Bitzeiten frei sein müssen; aus
// den Oktetts allein wäre das nicht ableitbar, denn 270 Oktetts sind ein großes Telegramm oder dreißig
// kleine, und dreißig belegen den Bus 156ms länger.
//
// Die Schranken sind so gelegt, dass JEDER weggelassene Anteil darunter fällt. 270 Oktetts über die
// Messspanne von 1000ms (sie ist reproduzierbar, weil loop() im Test dicht läuft und die Sekunde damit auf
// ein, zwei Millisekunden genau abgeschlossen wird):
//
//     nur Oktetts                   366ms -> 36%
//     + Pause (30 x 5208µs)         522ms -> 52%
//     + Quittungsslot (30 x 2708µs) 603ms -> 60%   <- das Sollverhalten
//
// Mit 56..65 fällt also sowohl der fehlende Pausen- als auch der fehlende Quittungsanteil durch. Wer hier
// eine Konstante ändert, muss die Tabelle mitrechnen - sie ist der eigentliche Inhalt des Falls.
static void test_bus_load_percent_counts_frame_gaps()
{
    fx->connect();

    const Statistics &st = fx->_dll.getStatistics();

    // Ohne Verkehr 0 - und das darf nicht mit "noch keine Messung" verwechselt werden.
    TEST_ASSERT_EQUAL(0, st.getBusLoadPercent());

    std::vector<uint8_t> frame = standardFrame();
    std::vector<uint8_t> stream;
    for (int i = 0; i < 30; i++)
        stream.insert(stream.end(), frame.begin(), frame.end());

    fx->feed(stream, 0);

    // Die Momentaufnahme läuft aus loop(), es muss also über eine Fensterbreite hinweg gepumpt werden -
    // vorher gibt es keine Spanne und der Wert ist 0.
    fx->pump(1100);

    TEST_ASSERT_EQUAL(30, st.getRxFrames());
    TEST_ASSERT_EQUAL(270, st.getRxFrameBytes());

    // Die Spanne ist die GEMESSENE Zeit, deshalb Schranken statt eines festen Werts - aber eng genug, dass
    // die Tabelle oben sie trennt.
    uint16_t percent = st.getBusLoadPercent();
    TEST_ASSERT_GREATER_THAN(55, percent);
    TEST_ASSERT_LESS_THAN(66, percent);
}

// ÜBER 100 WIRD BEWUSST NICHT GEDECKELT, und dieser Fall hält das fest - sonst "repariert" es später
// jemand als offensichtlichen Fehler. Physikalisch kann der Bus nicht voller als voll sein, ein Wert über
// 100 ist also ein Befund, und solange niemand die Last im Betrieb angesehen hat, soll er sichtbar sein.
//
// 100 Telegramme à 9 Oktetts belegen rechnerisch 900 × 1354µs + 100 × 7916µs = 2,01s; über die Messspanne
// von 1000ms sind das rund 201%. Die Schranken lassen Luft, prüfen aber beides: dass gar nicht gedeckelt
// wird (sonst käme 100) und dass der Wert nicht in einem 8-Bit-Typ überläuft (dann käme 201-256, also ein
// kleiner Wert statt eines großen).
static void test_bus_load_percent_is_not_capped()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    std::vector<uint8_t> stream;
    for (int i = 0; i < 100; i++)
        stream.insert(stream.end(), frame.begin(), frame.end());

    fx->feed(stream, 0);
    fx->pump(1100);

    uint16_t percent = fx->_dll.getStatistics().getBusLoadPercent();
    TEST_ASSERT_GREATER_THAN(150, percent);
    TEST_ASSERT_LESS_THAN(220, percent);
}

// ---------------------------------------------------------------------------------------------------

// DIE LISTE DER FÄLLE, für beide Einstiegspunkte dieselbe - auf der Hardware ruft setup() sie, nativ
// main(). Das ist die einzige Stelle, die von einem Fall weiß.
// Die Fälle der Sendewarteschlange liegen in einer eigenen Datei: sie brauchen weder Vorrichtung noch
// Interface noch Zeit, und genau das ist der Grund, warum die Klasse herausgezogen wurde.
void runTransmitQueueTests();

static int runAllTests()
{
    UNITY_BEGIN();

    runTransmitQueueTests();

    RUN_TEST(test_connect_reports_connected);
    RUN_TEST(test_connect_without_answer_stays_disconnected);

    RUN_TEST(test_standard_frame_is_delivered);
    RUN_TEST(test_short_extended_frame_is_delivered);
    RUN_TEST(test_extended_frame_maximum_length);
    RUN_TEST(test_bad_checksum_is_reported_as_invalid);
    RUN_TEST(test_bad_checksum_standard_maximal);
    RUN_TEST(test_extended_bad_checksum_is_reported_as_invalid);
    RUN_TEST(test_bad_checksum_extended_maximal);
    RUN_TEST(test_truncated_frame_is_reported_as_invalid);
    RUN_TEST(test_resync_recovers_at_next_frame);
    RUN_TEST(test_resyncs_are_counted);
    RUN_TEST(test_rx_queue_peak_is_tracked);
    RUN_TEST(test_chip_errors_are_counted_individually);
    RUN_TEST(test_repeated_frame_is_filtered);
    RUN_TEST(test_fragment_then_pause_then_valid_frame);
    RUN_TEST(test_fragment_without_pause_swallows_next_frame);
    RUN_TEST(test_fragment_with_overlong_length_still_ends_at_pause);
    RUN_TEST(test_fragment_with_length_254_still_ends_at_pause);
    RUN_TEST(test_fragment_with_short_length_leaves_discarded_bytes);

    RUN_TEST(test_unknown_control_byte_is_reported_as_error);
    RUN_TEST(test_system_state_is_a_two_byte_sequence);

    RUN_TEST(test_poll_control_byte_only_is_not_invalid);
    RUN_TEST(test_poll_full_cycle_completes_without_pause);
    RUN_TEST(test_poll_bad_checksum_is_rejected);
    RUN_TEST(test_poll_impossible_slot_count_is_rejected);
    RUN_TEST(test_poll_is_never_acknowledged);

    RUN_TEST(test_addressed_frame_is_acknowledged);
    RUN_TEST(test_extended_addressed_frame_is_acknowledged);
    RUN_TEST(test_unaddressed_frame_is_not_acknowledged);
    RUN_TEST(test_acknowledge_is_suppressed_when_behind);
    RUN_TEST(test_nack_is_sent_and_flagged);
    RUN_TEST(test_busy_is_sent_and_flagged);
    RUN_TEST(test_bus_monitor_never_acknowledges);

    RUN_TEST(test_monitor_frame_without_acknowledge);
    RUN_TEST(test_monitor_frame_with_acknowledge);
    RUN_TEST(test_monitor_frame_with_nack);
    RUN_TEST(test_monitor_frame_with_busy);
    RUN_TEST(test_monitor_rejects_send_frame);
    RUN_TEST(test_monitor_drops_queued_telegram);
    RUN_TEST(test_monitor_aborts_running_transmission);
    RUN_TEST(test_monitor_suppresses_state_request);
    RUN_TEST(test_monitor_rejects_busy_mode);
    RUN_TEST(test_monitor_clears_busy_mode);
    RUN_TEST(test_monitor_clears_auto_acknowledge);
    RUN_TEST(test_monitor_defers_configuration);
    RUN_TEST(test_monitor_survives_silence_and_exits_cleanly);

    RUN_TEST(test_send_frame_produces_correct_sequence);
    RUN_TEST(test_send_standard_minimal);
    RUN_TEST(test_send_standard_maximal);
    RUN_TEST(test_send_extended_minimal);
    RUN_TEST(test_send_extended_maximal_uses_offsets_sparingly);
    RUN_TEST(test_send_frame_rejects_oversized);

    RUN_TEST(test_own_echo_is_not_acknowledged);
    RUN_TEST(test_own_echo_without_confirmation_has_no_ack_flags);
    RUN_TEST(test_own_echo_with_confirmation_is_acked_but_not_addressed);
    RUN_TEST(test_foreign_frame_is_acknowledged_while_awaiting_confirmation);
    RUN_TEST(test_echo_and_positive_confirmation);
    RUN_TEST(test_echo_and_negative_confirmation_flags_nack);
    RUN_TEST(test_echo_repetitions_keep_watchdog_quiet);
    RUN_TEST(test_confirmation_without_echo_still_releases_path);
    RUN_TEST(test_confirmation_after_broken_echo_is_lost_without_pause);
    RUN_TEST(test_confirmation_after_broken_echo_arrives_after_pause);
    RUN_TEST(test_missing_confirmation_triggers_reset);
    RUN_TEST(test_connection_loss_is_counted);

    RUN_TEST(test_tick_rate_is_measured);
    RUN_TEST(test_tick_gap_survives_restart);
    RUN_TEST(test_interface_backlog_is_measured);
    RUN_TEST(test_deferred_tick_is_reported);
    RUN_TEST(test_healthy_tick_is_not_reported);
    RUN_TEST(test_rx_queue_overflow_is_counted);
    RUN_TEST(test_interface_overflow_is_counted);
    RUN_TEST(test_tx_queue_overflow_is_counted);
    RUN_TEST(test_control_queue_overflow_is_counted);
    RUN_TEST(test_control_sequence_is_never_split);
    RUN_TEST(test_timer_registers_on_begin_and_releases_on_end);
    RUN_TEST(test_timer_registration_is_idempotent);
    RUN_TEST(test_timer_rejects_more_clients_than_slots);
    RUN_TEST(test_timer_slot_is_released_on_destruction);
    RUN_TEST(test_timer_interval_is_global);
    RUN_TEST(test_process_never_ticks);
    RUN_TEST(test_timer_trigger_ticks_registered_clients);

    RUN_TEST(test_bus_load_counts_only_frame_bytes);
    RUN_TEST(test_bus_load_percent_counts_frame_gaps);
    RUN_TEST(test_bus_load_percent_is_not_capped);

    return UNITY_END();
}

#ifndef ARDUINO

// NATIVER EINSTIEGSPUNKT. Kein setup()/loop(), keine serielle Schnittstelle - Unity schreibt hier auf
// stdout, und der Rückgabewert entscheidet über Erfolg oder Fehlschlag des Laufs.
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    return runAllTests();
}

#else

void setup()
{
    // AUF DEN LESER WARTEN, nicht bloß pauschal schlafen. Nach dem Upload zählt sich der USB-CDC neu auf,
    // und der Testläufer öffnet den Port erst danach - wer vorher losschreibt, verliert die ersten Zeilen.
    // Genau das war zu sehen: ein Lauf meldete 23 Fälle statt 24, der erste fehlte einfach.
    //
    // Serial.begin() steht hier, obwohl UNITY_OUTPUT_START() es ohnehin täte: sonst gäbe es nichts, worauf
    // sich warten ließe. Der Zeitausstieg ist dafür da, dass die Sammlung auch ohne angeschlossenen Leser
    // durchläuft - etwa wenn jemand die Firmware nur flasht und mitliest.
    Serial.begin(115200);

    uint32_t until = millis() + 5000;
    while (!Serial && (int32_t)(millis() - until) < 0)
        delay(10);

    delay(500); // dem Leser Zeit lassen, wirklich zu lesen - das Öffnen allein genügt ihm nicht

    runAllTests();
}

// Nach UNITY_END() ist inhaltlich nichts mehr zu tun - die Schleife hält nur die Verbindung offen und
// nimmt zwei Tasten entgegen. Leer sollte sie trotzdem nicht sein: eine Endlosschleife mit Vollgas kommt
// nie an einen Punkt, an dem der USB-Stack bedient wird.
//
//   r - neu starten, also die Sammlung noch einmal durchlaufen lassen, ohne neu zu flashen
//   b - in den Bootloader (nur RP2040), wenn der nächste Upload ansteht und der Port zickt
void loop()
{
    static bool hintShown = false;
    if (!hintShown)
    {
        hintShown = true;
#if defined(ARDUINO_ARCH_RP2040)
        Serial.println("[r] neu starten  [b] Bootloader");
#else
        Serial.println("[r] neu starten");
#endif
    }

    if (Serial.available())
    {
        int input = Serial.read();

        if (input == 'r' || input == 'R')
        {
            Serial.println("[REBOOT]");
            Serial.flush();
#if defined(ARDUINO_ARCH_RP2040)
            rp2040.reboot();
#elif defined(ARDUINO_ARCH_ESP32)
            ESP.restart();
#endif
        }

#if defined(ARDUINO_ARCH_RP2040)
        if (input == 'b' || input == 'B')
        {
            Serial.println("[BOOTLOADER]");
            Serial.flush();

            // OHNE MASSENSPEICHER, deshalb nicht rp2040.rebootToBootloader(). Das ruft reset_usb_boot(0, 0)
            // und lässt das Bootrom beide Schnittstellen anbieten - dann meldet Windows ein Laufwerk und
            // der Explorer springt auf. Maske 1 schaltet den Massenspeicher ab; PICOBOOT bleibt, und genau
            // darüber lädt PlatformIO ohnehin hoch (picotool, siehe "picotool info -d" in dessen
            // Upload-Skript). Beim automatischen 1200-Baud-Reset geht das nicht: den löst der Kern selbst
            // mit Maske 0 aus, und der Sketch sieht die Baudrate nicht.
            reset_usb_boot(0, 1);
        }
#endif
    }

    delay(10);
}

#endif // ARDUINO
