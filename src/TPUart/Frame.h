#pragma once
#include <stddef.h>
#include <stdint.h>

#include <string>

#include "TPUart/Types.h"

namespace TPUart
{

// Übernommen aus Frame.h der alten Library - das ist die API, mit der der übrige KNX-Stack arbeitet.
// Methodennamen und Semantik sind deshalb bewusst unverändert. Abweichungen gegenüber dem Original:
//
//  - DAS FRAME BESITZT SEINE DATEN, aber ohne Heap. Man sagt beim Anlegen, wie viele Zeichen gebraucht
//    werden, und schreibt hinein - der Speicher steckt im Objekt. Die alte Klasse besaß ihn auch, nur per
//    malloc und mit einem _deleteData-Flag; ohne Kopierkonstruktor hätte eine Kopie denselben Zeiger
//    doppelt freigegeben. Hier gibt es weder Zeiger noch Freigabe: gefahrlos kopierbar, und eine Kopie
//    behält ihre Daten auch, wenn das Original längst weg ist.
//
//    Der Preis ist die feste Größe (TPUART_BUFFER_SIZE): ein Frame ist immer so groß wie das größtmögliche
//    Telegramm, egal wie kurz das echte ist. Angelegt wird es dort, wo es hingehört - auf dem STACK des
//    Verbrauchers, für die Dauer eines Durchlaufs. Genau deshalb nicht auf dem Heap: der Empfangsweg würde
//    sonst im Bustakt allozieren und freigeben, dauerhaft, neben den längerlebigen Blöcken der Sendeseite.
//    Wer ein Frame aufheben will, kopiert es und weiß, was es kostet.
//  - uint8_t statt char. Ob char vorzeichenbehaftet ist, ist implementierungsabhängig; mit uint8_t hängt
//    keine Adress- oder Längenberechnung mehr daran.
//  - Die tatsächlich empfangene Länge wird mitgeführt (length()). Nötig, weil eine Pause ein Telegramm
//    vorzeitig beenden kann - size() sagt dann, wie lang es laut Kopf sein SOLLTE, length() wie viel
//    wirklich ankam. Alle Zugriffe sind gegen length() abgesichert, damit ein abgeschnittenes Frame nicht
//    über den Puffer hinaus liest.
//  - isValid() liest das INVALID-Flag, statt die CRC8 jedes Mal neu zu rechnen: der Receiver bildet die
//    Prüfsumme bereits inkrementell beim Einlesen und kennt zusätzlich die Fälle, die eine Neuberechnung
//    gar nicht erkennen könnte (abgeschnitten, Längenbyte korrupt). calcCRC8() gibt es weiterhin, wer
//    selbst nachrechnen will.
//  - Nicht übernommen: checkCRC16CCITT()/checkCRC16SPI() - Extended-CRC ist eine bewusste Entscheidung
//    gegen den Dienst, nicht eine offene Baustelle (Begründung in AGENTS.md) - sowie
//    awaitDestination()/awaitSize(), reine Parser-Hilfen des alten Receivers, die der Receiver hier
//    selbst erledigt. cemiSize()/cemiData() standen ebenfalls auf dieser Liste und sind zurück: der
//    knx-Stack braucht sie, samt der Eigenheit mit dem malloc-Puffer (siehe unten).
class Frame
{
  private:
    uint8_t _data[TPUART_BUFFER_SIZE];
    size_t _length; // tatsächlich empfangene Bytes - kann kleiner als size() sein
    uint8_t _flags;

    // Jeder Zugriff läuft hierüber: bei einem abgeschnittenen Frame liegen die hinteren Positionen gar
    // nicht vor, und ohne diese Grenze würden destination() & Co. Datenmüll hinter dem Telegramm lesen.
    uint8_t at(size_t pos) const;

  public:
    // Legt ein Frame der gewünschten Länge an; der Inhalt wird anschließend über buffer() geschrieben.
    // Länge über TPUART_BUFFER_SIZE wird gekappt - mehr kann kein gültiges Telegramm haben.
    explicit Frame(size_t length, uint8_t flags = 0);

    // Legt ein Frame an und KOPIERT die Daten hinein.
    Frame(const uint8_t *data, size_t length, uint8_t flags = 0);

    // KOMPAT: Signatur der alten Library (char statt uint8_t, keine Flags). Dort kopierte dieser
    // Konstruktor ebenfalls - er setzte _deleteData und gab im Destruktor wieder frei.
    Frame(const char *data, size_t length);

    // LESEZUGRIFF, und die Signatur ist die der alten Library: `const char *`, nicht `const uint8_t *`.
    // Das ist keine Nachlässigkeit, sondern Pflicht - OFM-Network schreibt `const char *d = frame.data()`,
    // und ein anderer Zeigertyp wäre dort ein Übersetzungsfehler. Über den Rückgabetyp lässt sich nicht
    // überladen, deshalb heißt der Schreibzugriff buffer() und nicht ebenfalls data().
    const char *data() const;

    uint8_t data(size_t pos) const;

    // Schreibzugriff auf den eigenen Speicher - für den, der das Frame befüllt. Es sind genau length()
    // Bytes, mehr hat das Objekt nicht zugesagt.
    uint8_t *buffer();

    // Wie viele Bytes tatsächlich empfangen wurden - nicht zu verwechseln mit size().
    size_t length() const;

    uint8_t flags() const;
    void addFlags(uint8_t flags);
    void resetFlags();

    bool isExtended() const;
    bool isFrame() const;

    // Metadaten am Anfang des Frames plus die Prüfsumme am Ende.
    uint8_t metadataSize() const;

    uint8_t apduSize() const;

    // Gesamtlänge laut Kopf, inklusive Metadaten und APDU. Bei einem abgeschnittenen Frame größer als
    // length() - dann ist das hier die Soll-, nicht die Ist-Länge.
    //
    // WIRD NIE 0, auch nicht bei einem Fragment von zwei Bytes; siehe die Implementierung. Mehrere
    // Aufrufstellen rechnen size() - 1.
    uint16_t size() const;

    // DIESELBE ABLEITUNG AUF ROHEN BYTES, ohne dass ein Frame entstehen muss. Sie ist hier statisch, weil
    // zwei Stellen sie brauchen, an denen es (noch) kein Frame gibt: der Empfänger zerlegt in seinen
    // eigenen Puffer und darf im Tick kein 263-Byte-Objekt auf den Stack legen, und die Sendewarteschlange
    // trennt damit ihre Einträge - dort steht bewusst kein Längenpräfix, weil nur Geprüftes hineinkommt.
    //
    // DAS IST DIE EINZIGE STELLE MIT DIESER RECHNUNG. Sie stand einmal zusätzlich inline im Empfänger;
    // zwei Kopien einer Regel laufen früher oder später auseinander.
    //
    // 0 heißt "noch nicht entscheidbar" - für ein Standard-Telegramm werden 6 Bytes gebraucht (das
    // Längen-Nibble sitzt in Byte 5), für ein Extended 7. Der Wert kann TPUART_BUFFER_SIZE ÜBERSCHREITEN
    // (ein Längenoktett von 255 ergibt 264); das ist ein korruptes Telegramm, und wie darauf zu reagieren
    // ist, entscheidet der Aufrufer - der Empfänger meldet es als kaputt, der Sendeweg lehnt ab.
    //
    // NUR L_DATA. Ein Poll (L_POLL_DATA_IND) hat einen anderen Aufbau, seine Länge folgt aus dem
    // Slot-Count; dafür ist Receiver::processPollByte() zuständig.
    static uint16_t sizeOf(const uint8_t *data, size_t available);

    uint16_t source() const;
    uint16_t destination() const;
    bool isGroupAddress() const;
    bool isRepeated() const;

    // CRC-8/GSM-A: XOR über alles vor der Prüfsumme, invertiert.
    uint8_t calcCRC8() const;

    // Vom Receiver beim Einlesen entschieden, siehe Klassenkommentar.
    // IST DAS EIN INTAKTES TELEGRAMM? Vier Bedingungen, und alle müssen stimmen:
    //
    //   - das INVALID-Flag ist nicht gesetzt, der Empfänger hat es also nicht als kaputt gemeldet
    //   - das Steuerbyte benennt L_Data (standard oder extended)
    //   - die Länge entspricht exakt der aus dem Kopf abgeleiteten - damit sind Mindestlänge und
    //     Vollständigkeit in einem erledigt, ein abgeschnittenes Telegramm fällt durch
    //   - die Prüfsumme stimmt
    //
    // BEIDES ZUSAMMEN, und keins davon reicht allein. Das Flag ist das Urteil des EMPFÄNGERS und stützt
    // sich auf Wissen, das den Bytes nicht mehr anzusehen ist (eine Pause mitten im Telegramm etwa) - es
    // zu übergehen hieße, ein bekannt kaputtes Telegramm für gut zu erklären. Umgekehrt ist es bei einem
    // Telegramm, das ein Aufrufer selbst zum SENDEN baut, immer 0 und damit ohne Aussage; dort trägt
    // allein die Rechnung. Deshalb prüft der Sendeweg hiermit, bevor er annimmt.
    //
    // Der Preis ist ein Durchlauf über das Telegramm - aber nur, wenn das Flag sauber ist. Gerufen wird
    // das aus loop(), nicht aus dem Tick; dort führt der Receiver die Prüfsumme inkrementell mit.
    bool isValid() const;
    bool isInvalid() const;

    bool isFiltered() const;
    void setFiltered();

    bool isTransmitted() const;
    void setTransmitted();

    // Nur bei selbst gesendeten Telegrammen: die BCU hat den Versand bestätigt (L_Data.con).
    bool isDataCon() const;

    // Wir haben dieses Frame selbst quittiert - im Unterschied zu einer bloß beobachteten Quittung.
    bool isAddressed() const;

    bool isAck() const;
    bool isNack() const;
    bool isBusy() const;

    // WIR haben quittiert: setzt ADDRESSED zusammen mit ACK (+BUSY/NACK), identisch zur alten Library.
    // ACK bedeutet durchgängig "dieses Telegramm wurde quittiert" - egal ob von uns, oder im Busmonitor
    // auf dem Bus beobachtet, oder als Antwort auf ein selbst gesendetes Telegramm. ADDRESSED unterscheidet
    // davon nur den ersten Fall.
    void setAcknowledge(AckType acknowledge);

    // Eine auf dem Bus BEOBACHTETE Quittung: ohne ADDRESSED, denn sie kam nicht von uns.
    void setAcknowledge(bool busy, bool nack);

    // KOMPAT: die cEMI-Sicht auf das Telegramm, für die Schicht darüber (knx-Stack). Wie in der alten
    // Library, inklusive ihrer Eigenheit: cemiData() liefert einen mit malloc angelegten Puffer, den DER
    // AUFRUFER freigeben muss. Das ist der Grund, warum es als "nicht übernommen" begann - der Stack
    // braucht es aber, also ist es zurück.
    //
    // Aufbau: cEMI erwartet Extended-Format mit zwei zusätzlichen Bytes am Anfang (0x29 = L_Data.ind,
    // 0x00 = keine Zusatzinfo). Ein Standard-Telegramm wird dabei umsortiert, weil dort Länge und
    // Hop-Count in einem Byte stecken. Die Prüfsumme fällt in beiden Fällen weg.
    uint16_t cemiSize() const;
    uint8_t *cemiData() const;

    // Die drei Textausgaben allozieren über std::string auf dem Heap - bewusst so aus der alten Library
    // übernommen, damit die API identisch bleibt. Sie dürfen deshalb nur aus dem Hauptkontext gerufen
    // werden, nie aus einem Interrupt.
    std::string humanSource() const;
    std::string humanDestination() const;
    std::string printFrame() const;
};

} // namespace TPUart
