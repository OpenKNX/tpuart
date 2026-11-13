// TESTS DER SENDEWARTESCHLANGE, bewusst OHNE die Vorrichtung aus test_main.cpp: diese Klasse kennt weder
// Interface noch Tick noch Zeit. Sie ist reine Datenstruktur, und genau deshalb wurde sie herausgezogen -
// die Verschiebearithmetik ist das Risiko des ganzen Umbaus, und hier lässt sie sich erschöpfend prüfen,
// nativ in Mikrosekunden statt gegen Fristen.

#include <unity.h>

#include <vector>

#include "TPUart/Frame.h"
#include "TPUart/TransmitQueue.h"

using namespace TPUart;

// ---------------------------------------------------------------------------------------------------
// Helfer
// ---------------------------------------------------------------------------------------------------

// Ein Standard-Telegramm mit gewählter ROHER Priorität (0 = System, 1 = Normal, 2 = Urgent, 3 = Low) und
// einem Markerbyte, an dem sich das Telegramm beim Herauskommen wiedererkennen lässt.
//
// Steuerbyte: 0xB0 erfüllt (ctrl & L_DATA_MASK) == L_DATA_STANDARD_IND, die Bits 3-2 bleiben frei für die
// Priorität. Die Prüfsumme ist beliebig - diese Klasse prüft sie nicht, das tut der Aufrufer.
static Frame makeFrame(uint8_t rawPriority, uint8_t marker, uint8_t apduSize = 1)
{
    std::vector<uint8_t> data;
    data.push_back((uint8_t)(0xB0 | (rawPriority << 2)));
    data.push_back(0x11);
    data.push_back(0x01);
    data.push_back(0x08);
    data.push_back(0x01);
    data.push_back((uint8_t)(0xE0 | (apduSize & 0x0F)));
    data.push_back(marker); // TPCI-Position, hier als Wiedererkennung

    for (uint8_t i = 0; i < apduSize; i++)
        data.push_back(i);

    data.push_back(0x00); // Prüfsummen-Position

    return Frame(data.data(), data.size());
}

// Das größtmögliche Telegramm: extended mit 254 APDU-Oktetts = 263 Byte.
static Frame makeMaximalFrame(uint8_t rawPriority, uint8_t marker)
{
    std::vector<uint8_t> data(263, 0x00);
    data[0] = (uint8_t)(0x30 | (rawPriority << 2)); // (ctrl & L_DATA_MASK) == L_DATA_EXTENDED_IND
    data[6] = 254;                                  // Längen-Oktett des Extended-Frames
    data[7] = marker;

    return Frame(data.data(), data.size());
}

// Nimmt den vordersten Eintrag ab und liefert sein Markerbyte, oder -1 wenn nichts da ist.
static int takeMarker(TransmitQueue &queue)
{
    size_t length = 0;
    const uint8_t *data = queue.front(length);
    if (data == nullptr) return -1;

    int marker = data[6];

    queue.pin();
    queue.pop();

    return marker;
}

// ---------------------------------------------------------------------------------------------------
// Reihenfolge
// ---------------------------------------------------------------------------------------------------

static void test_queue_fifo_within_class()
{
    TransmitQueue queue;

    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 1)));
    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 2)));
    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 3)));

    TEST_ASSERT_EQUAL(1, takeMarker(queue));
    TEST_ASSERT_EQUAL(2, takeMarker(queue));
    TEST_ASSERT_EQUAL(3, takeMarker(queue));
    TEST_ASSERT_TRUE(queue.empty());
}

// DER FALL, UM DEN ES GEHT: ein System-Telegramm überholt einen bereits wartenden Low-Rückstau. Die
// Rangfolge ist System > Urgent > Normal > Low, und die Rohwerte sind dabei NICHT sortiert - roh 1 ist
// Normal, roh 2 ist Urgent.
static void test_queue_priority_order()
{
    TransmitQueue queue;

    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 10))); // Low
    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 11))); // Low
    TEST_ASSERT_TRUE(queue.push(makeFrame(1, 20))); // Normal
    TEST_ASSERT_TRUE(queue.push(makeFrame(0, 30))); // System
    TEST_ASSERT_TRUE(queue.push(makeFrame(2, 40))); // Urgent

    TEST_ASSERT_EQUAL(30, takeMarker(queue)); // System
    TEST_ASSERT_EQUAL(40, takeMarker(queue)); // Urgent
    TEST_ASSERT_EQUAL(20, takeMarker(queue)); // Normal
    TEST_ASSERT_EQUAL(10, takeMarker(queue)); // Low, in Eingangsreihenfolge
    TEST_ASSERT_EQUAL(11, takeMarker(queue));
}

// Nimmt den vordersten Eintrag ab und vergleicht ihn BYTEWEISE mit dem erwarteten Telegramm. Der Marker
// allein genügt hier nicht: eine verrutschte Verschiebung kann den Kopf richtig und die Nutzlast falsch
// hinterlassen.
static void expectNext(TransmitQueue &queue, const Frame &expected)
{
    size_t length = 0;
    const uint8_t *data = queue.front(length);

    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL(expected.length(), length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY((const uint8_t *)expected.data(), data, (int)length);

    queue.pin();
    queue.pop();
}

// ---------------------------------------------------------------------------------------------------
// Vordrängeln - der eigentliche Zweck der Prioritäten
// ---------------------------------------------------------------------------------------------------

// GANZ NACH VORN: ein System-Telegramm kommt an, während ein Low-Rückstau wartet. Das ist der Fall, um
// dessentwillen es die Prioritäten überhaupt gibt - die ETS-Verbindungssteuerung darf nicht hinter einem
// Massenversand warten.
//
// ALLE TELEGRAMME SIND VERSCHIEDEN LANG, und das ist Absicht: bei gleich langen Einträgen sähe ein Versatz
// um eine ganze Eintragsgrenze noch plausibel aus, und der Test bemerkte ihn nicht.
static void test_queue_high_priority_jumps_to_front()
{
    TransmitQueue queue;

    Frame low1 = makeFrame(3, 1, 1);
    Frame low2 = makeFrame(3, 2, 5);
    Frame low3 = makeFrame(3, 3, 12);
    Frame system = makeFrame(0, 9, 3);

    TEST_ASSERT_TRUE(queue.push(low1));
    TEST_ASSERT_TRUE(queue.push(low2));
    TEST_ASSERT_TRUE(queue.push(low3));

    TEST_ASSERT_TRUE(queue.push(system));

    expectNext(queue, system); // überholt alle drei
    expectNext(queue, low1);   // und die bleiben untereinander in Eingangsreihenfolge
    expectNext(queue, low2);
    expectNext(queue, low3);
    TEST_ASSERT_TRUE(queue.empty());
}

// IN DIE MITTE: der schwierigere Fall. Ein Telegramm muss sich zwischen zwei belegte Bereiche schieben,
// hat also etwas vor sich UND etwas hinter sich - erst dann verschiebt push() wirklich einen Block, und
// erst dann kann die Arithmetik daneben liegen.
//
// Durchgespielt werden alle vier Klassen: Normal landet zwischen System und Low, Urgent danach noch enger
// zwischen System und Normal.
static void test_queue_priority_inserts_into_the_middle()
{
    TransmitQueue queue;

    Frame system = makeFrame(0, 10, 2);
    Frame low1 = makeFrame(3, 40, 7);
    Frame low2 = makeFrame(3, 41, 1);

    TEST_ASSERT_TRUE(queue.push(system));
    TEST_ASSERT_TRUE(queue.push(low1));
    TEST_ASSERT_TRUE(queue.push(low2));

    Frame normal = makeFrame(1, 30, 11); // zwischen System und Low
    TEST_ASSERT_TRUE(queue.push(normal));

    Frame urgent = makeFrame(2, 20, 4); // zwischen System und Normal
    TEST_ASSERT_TRUE(queue.push(urgent));

    expectNext(queue, system);
    expectNext(queue, urgent);
    expectNext(queue, normal);
    expectNext(queue, low1);
    expectNext(queue, low2);
    TEST_ASSERT_TRUE(queue.empty());
}

// BEIDES ZUSAMMEN: in die Mitte drängeln, während der vorderste Eintrag beim Tick liegt. Das ist die
// unangenehmste Kombination - die Verschiebung muss den gepinnten Bereich aussparen UND trotzdem an der
// richtigen Stelle einsortieren.
static void test_queue_middle_insert_spares_pinned_entry()
{
    TransmitQueue queue;

    Frame low1 = makeFrame(3, 1, 6);
    Frame low2 = makeFrame(3, 2, 3);

    TEST_ASSERT_TRUE(queue.push(low1));
    TEST_ASSERT_TRUE(queue.push(low2));

    size_t length = 0;
    const uint8_t *pinnedData = queue.front(length);
    std::vector<uint8_t> before(pinnedData, pinnedData + length);
    queue.pin(); // low1 liegt jetzt beim Tick

    Frame normal = makeFrame(1, 5, 9);
    Frame system = makeFrame(0, 7, 2);

    TEST_ASSERT_TRUE(queue.push(normal));
    TEST_ASSERT_TRUE(queue.push(system)); // vor normal, aber HINTER den gepinnten

    TEST_ASSERT_EQUAL_UINT8_ARRAY(before.data(), pinnedData, (int)length);

    queue.pop(); // der Tick ist fertig
    expectNext(queue, system);
    expectNext(queue, normal);
    expectNext(queue, low2);
    TEST_ASSERT_TRUE(queue.empty());
}

// ---------------------------------------------------------------------------------------------------
// Der Pin - die Zusicherung, auf der der ganze Entwurf ruht
// ---------------------------------------------------------------------------------------------------

// Der gepinnte Eintrag darf sich NICHT bewegen, auch nicht durch eine Aufnahme, die ihn sonst verschöbe.
// Der Tick liest aus genau diesem Speicher, während der Hauptkontext weiter einreiht.
static void test_queue_pin_holds_against_insert()
{
    TransmitQueue queue;

    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 77)));

    size_t length = 0;
    const uint8_t *pinnedData = queue.front(length);
    TEST_ASSERT_NOT_NULL(pinnedData);

    std::vector<uint8_t> before(pinnedData, pinnedData + length);
    queue.pin();

    // Ein System-Telegramm gehört VOR das Low - ohne den Pin würde es dessen Bytes verschieben.
    TEST_ASSERT_TRUE(queue.push(makeFrame(0, 88)));

    TEST_ASSERT_EQUAL_UINT8_ARRAY(before.data(), pinnedData, (int)length);
    TEST_ASSERT_EQUAL(77, pinnedData[6]);

    // Und danach kommt das System-Telegramm, nicht etwa erneut das gepinnte.
    queue.pop();
    TEST_ASSERT_EQUAL(88, takeMarker(queue));
    TEST_ASSERT_TRUE(queue.empty());
}

// Beim Wechsel in den Busmonitor räumt der Hauptkontext die Warteschlange - der gepinnte Eintrag muss
// dabei stehenbleiben, weil der Tick noch daraus lesen kann.
static void test_queue_clear_keeps_pinned_entry()
{
    TransmitQueue queue;

    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 5)));
    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 6)));
    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 7)));

    size_t length = 0;
    const uint8_t *pinnedData = queue.front(length);
    std::vector<uint8_t> before(pinnedData, pinnedData + length);
    queue.pin();

    queue.clear();

    TEST_ASSERT_TRUE(queue.empty()); // der Rest ist weg
    TEST_ASSERT_TRUE(queue.pinned());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(before.data(), pinnedData, (int)length);

    queue.pop();
    TEST_ASSERT_TRUE(queue.empty());
    TEST_ASSERT_FALSE(queue.pinned());
}

// ---------------------------------------------------------------------------------------------------
// Kapazität und Reserve
// ---------------------------------------------------------------------------------------------------

// OHNE DIE RESERVE NÜTZT DIE SORTIERUNG NICHTS: ein Massenversand füllt den Puffer, und das
// System-Telegramm wird an der Tür abgewiesen, bevor irgendetwas sortiert werden könnte.
static void test_queue_reserve_lets_high_priority_in()
{
    TransmitQueue queue;

    int accepted = 0;
    while (queue.push(makeFrame(3, 1)))
        accepted++;

    TEST_ASSERT_GREATER_THAN(0, accepted);

    // Nicht exakt 0: der Rest reicht nur nicht mehr für ein weiteres Telegramm. Genau das ist die
    // Bedingung, die zählt - "voll" heißt hier "das nächste passt nicht", nicht "kein Byte mehr frei".
    TEST_ASSERT_LESS_THAN(9, queue.freeFor(TP_PRIORITY_LOW));

    // Low ist am Ende, oberhalb davon passt weiterhin ein maximal großes Telegramm.
    TEST_ASSERT_TRUE(queue.push(makeMaximalFrame(0, 99)));

    size_t length = 0;
    const uint8_t *data = queue.front(length);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL(263, length);
    TEST_ASSERT_EQUAL(99, data[7]);
}

// Eine abgelehnte Aufnahme darf den Puffer NICHT anfassen - sonst hinterlässt der Fehlerfall genau die
// Desynchronisation, die die Prüfung verhindern soll.
static void test_queue_rejected_push_leaves_queue_intact()
{
    TransmitQueue queue;

    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 1)));
    TEST_ASSERT_TRUE(queue.push(makeFrame(3, 2)));

    size_t usedBefore = queue.used();

    // Bis zur Ablehnung füllen, dann muss der Füllstand stehen und der Inhalt unversehrt sein.
    while (queue.push(makeFrame(3, 9)))
        ;

    size_t usedAfterFull = queue.used();
    TEST_ASSERT_FALSE(queue.push(makeFrame(3, 9)));
    TEST_ASSERT_EQUAL(usedAfterFull, queue.used());
    TEST_ASSERT_GREATER_THAN(usedBefore, usedAfterFull);

    // Die beiden ersten kommen unverändert und in Reihenfolge heraus.
    TEST_ASSERT_EQUAL(1, takeMarker(queue));
    TEST_ASSERT_EQUAL(2, takeMarker(queue));
}

// Nach vielen kleinen Ein- und Ausgängen muss wieder ein maximal großes Telegramm hineinpassen. Ohne
// Kompaktierung wanderte der Inhalt nach rechts und der Puffer wäre irgendwann trotz freiem Platz voll.
static void test_queue_compaction_reclaims_space()
{
    TransmitQueue queue;

    for (int round = 0; round < 200; round++)
    {
        TEST_ASSERT_TRUE(queue.push(makeFrame(3, (uint8_t)(round & 0x7F))));
        TEST_ASSERT_EQUAL(round & 0x7F, takeMarker(queue));
    }

    TEST_ASSERT_TRUE(queue.empty());
    TEST_ASSERT_EQUAL(0, queue.used());
    TEST_ASSERT_TRUE(queue.push(makeMaximalFrame(3, 42)));
}

// ---------------------------------------------------------------------------------------------------
// Referenzmodell
// ---------------------------------------------------------------------------------------------------

// DER TEST, DER RECHENFEHLER IN DER VERSCHIEBUNG TATSÄCHLICH FINDET. Die Einzelfälle oben prüfen, was man
// sich vorstellen konnte; hier läuft eine lange, deterministisch erzeugte Folge von Aufnahmen und
// Entnahmen gegen ein Modell aus Vektoren - verglichen werden Reihenfolge UND Bytes.
//
// Deterministisch statt zufällig: ein Fehlschlag muss reproduzierbar sein. Die Folge bleibt bewusst weit
// unter der Kapazität, damit hier die Ordnung geprüft wird und nicht die Ablehnung.
static void test_queue_matches_reference_model()
{
    TransmitQueue queue;
    std::vector<std::vector<uint8_t>> model;

    uint32_t random = 12345;
    uint8_t marker = 0;

    for (int step = 0; step < 2000; step++)
    {
        random = random * 1103515245u + 12345u;
        bool doPush = (model.size() < 30) && ((random >> 16) & 1);

        if (doPush)
        {
            uint8_t rawPriority = (uint8_t)((random >> 20) & 0x03);
            uint8_t rank = telegramPriorityRank((uint8_t)(0xB0 | (rawPriority << 2)));

            Frame frame = makeFrame(rawPriority, marker++);
            TEST_ASSERT_TRUE(queue.push(frame));

            // Stabil nach Rang einsortieren: hinter alle Einträge mit gleichem oder höherem Rang.
            size_t at = model.size();
            while (at > 0 && telegramPriorityRank(model[at - 1][0]) > rank)
                at--;

            const uint8_t *bytes = (const uint8_t *)frame.data();
            model.insert(model.begin() + at, std::vector<uint8_t>(bytes, bytes + frame.length()));
        }
        else if (!model.empty())
        {
            size_t length = 0;
            const uint8_t *data = queue.front(length);

            TEST_ASSERT_NOT_NULL(data);
            TEST_ASSERT_EQUAL(model.front().size(), length);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(model.front().data(), data, (int)length);

            queue.pin();
            queue.pop();
            model.erase(model.begin());
        }

        TEST_ASSERT_EQUAL(model.empty(), queue.empty());
    }
}

void runTransmitQueueTests()
{
    RUN_TEST(test_queue_fifo_within_class);
    RUN_TEST(test_queue_priority_order);
    RUN_TEST(test_queue_high_priority_jumps_to_front);
    RUN_TEST(test_queue_priority_inserts_into_the_middle);
    RUN_TEST(test_queue_middle_insert_spares_pinned_entry);
    RUN_TEST(test_queue_pin_holds_against_insert);
    RUN_TEST(test_queue_clear_keeps_pinned_entry);
    RUN_TEST(test_queue_reserve_lets_high_priority_in);
    RUN_TEST(test_queue_rejected_push_leaves_queue_intact);
    RUN_TEST(test_queue_compaction_reclaims_space);
    RUN_TEST(test_queue_matches_reference_model);
}
