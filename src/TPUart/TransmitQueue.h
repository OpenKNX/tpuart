#pragma once
#include <stddef.h>
#include <stdint.h>

#include "TPUart/Types.h"

namespace TPUart
{

class Frame;

// GRÖSSE DES SENDEPUFFERS IN BYTES, nicht in Telegrammen - und das ist der Punkt. Ein gewöhnliches
// Gruppentelegramm ist 9 bis 15 Oktetts, ein maximales 263; feste Plätze verschenkten also rund 95%.
// Dieselben 2048 Byte fassen ~140 gewöhnliche Telegramme, als feste Plätze wären es 7.
//
// WIE VIEL MAN BRAUCHT, ergibt sich aus der Anwendung, nicht aus dem Bus: der knx-Stack sendet je
// loop()-Durchlauf genau EIN Telegramm (bau_systemB_device.cpp, sendNextGroupTelegram), der Loop läuft
// aber um ein Vielfaches schneller, als der Bus abfließt (~50 Telegramme/s). Wer 100 Kommunikationsobjekte
// in einem Rutsch ändert, hat sie nach ~200ms alle eingereiht, während der Bus davon ~10 geschafft hat -
// es warten also ~90. Die Faustregel lautet deshalb: so viele Bytes, wie die Zahl der gleichzeitig
// sendebereiten KOs mal deren typischer Telegrammgröße. Nach oben zusammengefasst wird bereits im KO
// selbst (mehrfaches Schreiben vor dem Senden ergibt ein Telegramm mit dem letzten Wert).
//
// ÜBER UNS PUFFERT NICHTS. Lehnt diese Queue ab, verwirft der knx-Stack das Telegramm und meldet ein
// negatives L_Data.con nach oben - jede Ablehnung ist also unmittelbar ein verlorenes Telegramm.
#ifndef TPUART_TX_BUFFER_SIZE
    #define TPUART_TX_BUFFER_SIZE 2048
#endif

// BYTES, DIE LOW NICHT BELEGEN DARF. Ohne sie nützt die Prioritätsordnung nichts: ein Massenversand füllt
// den Puffer, und das System-Telegramm der ETS-Verbindung wird schon an der Tür abgewiesen - lange bevor
// irgendetwas sortiert werden könnte. Der Wert ist abgeleitet, nicht geraten: ein maximal großes Telegramm
// oberhalb von Low passt immer noch hinein.
#ifndef TPUART_TX_PRIORITY_RESERVE
    #define TPUART_TX_PRIORITY_RESERVE TPUART_BUFFER_SIZE
#endif

// DIE SENDEWARTESCHLANGE: ein linearer Bytepuffer, in dem die Telegramme nach Priorität geordnet liegen.
//
// SIE GEHÖRT DEM HAUPTKONTEXT ALLEIN. Das ist die Bedingung, unter der hier überhaupt umsortiert werden
// darf - der Tick fasst sie nicht an, er bekommt über eine Vorlage genau ein Telegramm gereicht (siehe
// Transmitter). Ein früherer Entwurf ließ den Tick den vordersten Eintrag selbst holen; dann darf niemand
// mehr etwas verschieben, und Priorität wird unmöglich.
//
// AUFBAU. Die Einträge liegen dicht an dicht, nach Rang gruppiert, innerhalb eines Rangs in
// Eingangsreihenfolge:
//
//     0 ........ _head ...................................... _end[3] ........ SIZE
//     [gepinnt]  [Rang 0][Rang 1][Rang 2][Rang 3            ]  [frei         ]
//                        _end[0] _end[1] _end[2]
//
// KEIN LÄNGENPRÄFIX. Ein Telegramm beschreibt seine eigene Länge (telegramSize in Types.h), und hier
// kommt nur Geprüftes hinein - der Aufrufer weist alles ab, was er nicht sicher deuten kann. Das spart
// zwei Byte je Eintrag, also rund 14% bei einem gewöhnlichen Telegramm, und es gibt die Länge nur einmal.
// DER EMPFANGSRING MACHT ES ANDERS UND ZU RECHT: der bewahrt auch INVALID-Einträge auf, und bei genau
// denen ist die Selbstbeschreibung das, was man nicht glauben darf.
//
// KEIN UMBRUCH. Anders als der Empfangsring ist dieser Puffer linear und wird kompaktiert - keine
// Modulo-Indizierung, keine geteilten Einträge. Das ist einfacher, nicht komplizierter.
//
// KEIN HEAP. Vorher lag jedes wartende Telegramm in einem eigenen malloc-Block variabler Größe - die
// einzige fragmentierende Allokation der Library, im Bustakt über die Lebensdauer des Geräts.
class TransmitQueue
{
  private:
    uint8_t _buffer[TPUART_TX_BUFFER_SIZE];

    // Ende je Rang. _end[TP_PRIORITY_COUNT - 1] ist zugleich das Ende des Belegten, ein eigener
    // _tail wäre also nur ein zweiter Name dafür.
    size_t _end[TP_PRIORITY_COUNT] = {};

    // Beginn des logischen Inhalts. 0, solange nichts gepinnt ist; sonst das Ende des gepinnten Eintrags.
    size_t _head = 0;

    bool _pinned = false;

    // Einmalig, wie die Überlaufmelder der Schicht: die Selbstbeschreibung eines Eintrags war unplausibel.
    // Im geordneten Betrieb unerreichbar - hereingelassen wird nur Geprüftes -, aber ein Rechenfehler in
    // der Verschiebung sähe genau so aus, und dann ist Weiterlesen das Schlechteste von allem.
    bool _corrupted = false;

    void compact();

  public:
    // Nimmt ein Telegramm auf. Der Rang kommt aus dem Steuerbyte, die Länge aus dem Frame - beides steckt
    // im Telegramm selbst, es wird also nichts doppelt übergeben.
    //
    // ERWARTET EIN VOLLSTÄNDIGES, GEPRÜFTES TELEGRAMM einschließlich Prüfsummen-Oktett. Diese Klasse prüft
    // NICHT: sie kennt weder CRC noch die Plausibilität eines Steuerbytes, das ist Sache des Aufrufers
    // (Transmitter::pushTransmitQueue). Wird die Zusage gebrochen, desynchronisiert der Puffer - ohne
    // Längenpräfix beginnt der nächste Eintrag dann an der falschen Stelle.
    //
    // Liefert false, wenn der Platz nicht reicht. Für TP_PRIORITY_LOW gilt dabei die kleinere Grenze
    // (siehe TPUART_TX_PRIORITY_RESERVE); der Puffer bleibt in diesem Fall unverändert.
    bool push(const Frame &frame);

    // Der vorderste Eintrag - also der höchstpriore, bei gleichem Rang der älteste. Zeigt IN DEN PUFFER,
    // es wird nichts kopiert: das ist der Grund, warum die Vorlage an den Tick kopierfrei ist.
    //
    // Nicht const, weil ein unplausibler Eintrag hier zum Verwerfen des Puffers führt - siehe _corrupted.
    const uint8_t *front(size_t &length);

    // Hält den vordersten Eintrag fest: ab jetzt bewegt ihn nichts mehr, weder eine Aufnahme noch die
    // Kompaktierung. DAS IST DIE ZUSICHERUNG, AUF DER DER GANZE ENTWURF RUHT - der Tick liest aus diesem
    // Speicher, während der Hauptkontext weiter einreiht und umsortiert.
    void pin();

    // Verwirft den gepinnten Eintrag und kompaktiert. Erst danach darf der Speicher wieder benutzt werden.
    void pop();

    // Leert die Warteschlange - ABER NICHT DEN GEPINNTEN EINTRAG. Der kann noch vom Tick gelesen werden;
    // er fällt beim nächsten pop(). Gerufen wird das beim Wechsel in den Busmonitor, wo alles Wartende
    // ohnehin überholt wäre.
    void clear();

    bool empty() const;
    bool pinned() const;

    // Belegte Bytes einschließlich eines gepinnten Eintrags - die Zahl, gegen die sich die Auslegung
    // lesen lässt (TPUART_TX_BUFFER_SIZE).
    size_t used() const;

    // Was für diesen Rang noch hineinpasst. Für TP_PRIORITY_LOW kleiner als für die übrigen.
    size_t freeFor(uint8_t rank) const;

    // Einmalig abzuholen, wie die übrigen Melder der Schicht.
    bool corrupted();
};

} // namespace TPUart
