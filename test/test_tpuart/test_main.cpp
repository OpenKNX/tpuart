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
        // KEIN eigener Takt: process() führt den Tick dann synchron aus, und jeder Fall bestimmt selbst,
        // wann wie oft getickt wird. Muss vor begin() stehen, sonst startet dort der Timer.
        _dll.setTickInterval(0);

        _dll.registerFrameCallback([this](Frame &frame) {
            CapturedFrame captured;
            captured._data.assign((const uint8_t *)frame.data(), (const uint8_t *)frame.data() + frame.length());
            captured._flags = frame.flags();
            captured._valid = frame.isValid();
            captured._isFrame = frame.isFrame();
            captured._source = frame.source();
            captured._destination = frame.destination();
            _frames.push_back(captured);
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
    void pump(uint32_t ms)
    {
        uint32_t until = millis() + ms;
        while ((int32_t)(millis() - until) < 0)
            _dll.process();
    }

    // Verbindungsaufnahme durchspielen. Die Baudratensuche liest das Interface DIREKT (nicht über den
    // Empfänger), es genügt also, die Antwort einzuspeisen, sobald der Request draußen ist.
    void connect(BcuType type = BcuType::Ncn5120)
    {
        _dll.begin(type);
        _dll.process(); // schickt U_Reset.req an den ersten Kandidaten

        _interface.addByte((char)U_RESET_IND, 0);

        for (int i = 0; i < 50 && !_dll.isConnected(); i++)
            _dll.process();

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
    fx->_dll.setTickInterval(0);
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

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getAcknowledgesSent());
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

    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getAcknowledgesSent());
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

    fx->feedAndSettle(standardFrame());

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getAcknowledgesSent());
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

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getAcknowledgesSent());
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getAcknowledgesSuppressed());

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

    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getAcknowledgesSent());
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
    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getAcknowledgesSent());
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

    TEST_ASSERT_EQUAL(0, fx->_dll.getStatistics().getAcknowledgesSent());
}

// ---------------------------------------------------------------------------------------------------
// Senden
// ---------------------------------------------------------------------------------------------------

// Der Aufrufer übergibt das Telegramm OHNE Prüfsumme - die hängt die Library an, damit sie nicht falsch
// sein kann. Auf der Leitung steht danach U_L_DataStart, je ein U_L_DataCont und zum Schluss U_L_DataEnd.
static void test_send_frame_produces_correct_sequence()
{
    fx->connect();

    std::vector<uint8_t> frame = standardFrame();
    frame.pop_back(); // ohne Prüfsumme übergeben

    TEST_ASSERT_TRUE(fx->_dll.sendFrame(frame.data(), frame.size()));
    fx->pump(40);

    const std::vector<uint8_t> &written = fx->_interface.writtenBytes();

    // Je Oktett zwei Hostbytes (Dienst mit Index, dann das Datenbyte), plus die angehängte Prüfsumme -
    // und davor EIN Offset-Byte: beginTransmission() setzt _chipOffsetValid zurück, das erste Oktett
    // trägt seinen Offset also mit. Bei neun Oktetten bleibt es bei diesem einen, der nächste fiele erst
    // bei Position 64 an.
    TEST_ASSERT_EQUAL(1 + (frame.size() + 1) * 2, written.size());
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_OFFSET_REQ | 0, written[0]);
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_START_REQ | 0, written[1]);
    TEST_ASSERT_EQUAL_HEX8(frame[0], written[2]);

    // Im U_L_DataEnd steht die Zahl der DATEN-Oktette; das Prüfsummen-Byte folgt dahinter und zählt nicht
    // als indiziertes Oktett mit.
    TEST_ASSERT_EQUAL_HEX8(U_L_DATA_END_REQ | (uint8_t)frame.size(), written[written.size() - 2]);
    TEST_ASSERT_EQUAL_HEX8(checksum(frame), written[written.size() - 1]);
    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getTxFrames());
}

// Die vier Größen-/Formatkombinationen auf der Sendeseite. Geprüft wird jeweils die vollständige Folge
// auf der Leitung und - der eigentliche Punkt - die ZAHL DER OFFSET-BYTES: der Dienst trägt nur sechs Bit
// Position, alles darüber muss per U_L_DataOffset.req nachgereicht werden. Sparsam heißt: nur beim
// Wechsel, nicht bei jedem Oktett.
static void expectSendSequence(const std::vector<uint8_t> &frame, size_t expectedOffsets)
{
    fx->connect();

    std::vector<uint8_t> payload(frame.begin(), frame.end() - 1); // ohne Prüfsumme übergeben
    TEST_ASSERT_TRUE(fx->_dll.sendFrame(payload.data(), payload.size()));

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
    std::vector<uint8_t> payload(frame.begin(), frame.end() - 1);
    TEST_ASSERT_TRUE(fx->_dll.sendFrame(payload.data(), payload.size()));

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
    std::vector<uint8_t> payload(frame.begin(), frame.end() - 1);

    TEST_ASSERT_TRUE(fx->_dll.sendFrame(payload.data(), payload.size()));
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
    std::vector<uint8_t> payload(frame.begin(), frame.end() - 1);

    TEST_ASSERT_TRUE(fx->_dll.sendFrame(payload.data(), payload.size()));
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
    std::vector<uint8_t> payload(frame.begin(), frame.end() - 1);

    TEST_ASSERT_TRUE(fx->_dll.sendFrame(payload.data(), payload.size()));
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
}

// Ein zu langes Telegramm wird abgelehnt, nicht abgeschnitten.
static void test_send_frame_rejects_oversized()
{
    fx->connect();

    std::vector<uint8_t> frame(300, 0x00);
    frame[0] = 0xBC;

    TEST_ASSERT_FALSE(fx->_dll.sendFrame(frame.data(), frame.size()));
}

// ---------------------------------------------------------------------------------------------------
// Schnittstelle und Statistik
// ---------------------------------------------------------------------------------------------------

static void test_interface_overflow_is_counted()
{
    fx->connect();

    fx->_interface.forceOverflow(true);
    uint8_t byte = U_RESET_IND;
    fx->feed(&byte, 1);
    fx->pump(6);

    TEST_ASSERT_EQUAL(1, fx->_dll.getStatistics().getInterfaceOverflows());
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

    uint32_t before = fx->_dll.getStatistics().getRxBusBytes();

    uint8_t control = U_RESET_IND;
    fx->feed(&control, 1);
    fx->pump(6);

    TEST_ASSERT_EQUAL(before, fx->_dll.getStatistics().getRxBusBytes());

    fx->feedAndSettle(standardFrame());

    TEST_ASSERT_EQUAL(before + 9, fx->_dll.getStatistics().getRxBusBytes());
}

// ---------------------------------------------------------------------------------------------------

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

    UNITY_BEGIN();

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

    RUN_TEST(test_send_frame_produces_correct_sequence);
    RUN_TEST(test_send_standard_minimal);
    RUN_TEST(test_send_standard_maximal);
    RUN_TEST(test_send_extended_minimal);
    RUN_TEST(test_send_extended_maximal_uses_offsets_sparingly);
    RUN_TEST(test_send_frame_rejects_oversized);

    RUN_TEST(test_echo_and_positive_confirmation);
    RUN_TEST(test_echo_and_negative_confirmation_flags_nack);
    RUN_TEST(test_echo_repetitions_keep_watchdog_quiet);
    RUN_TEST(test_confirmation_without_echo_still_releases_path);
    RUN_TEST(test_confirmation_after_broken_echo_is_lost_without_pause);
    RUN_TEST(test_confirmation_after_broken_echo_arrives_after_pause);
    RUN_TEST(test_missing_confirmation_triggers_reset);

    RUN_TEST(test_interface_overflow_is_counted);
    RUN_TEST(test_control_sequence_is_never_split);
    RUN_TEST(test_bus_load_counts_only_frame_bytes);

    UNITY_END();
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
