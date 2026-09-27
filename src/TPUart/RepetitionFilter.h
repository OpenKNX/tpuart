#pragma once
#include <stddef.h>
#include <stdint.h>

namespace TPUart
{

class Frame;

// Anzahl beobachteter Absender. Wird es eng, verdrängt der am längsten nicht gesehene.
#ifndef TPUART_REPETITION_FILTER_COUNT
#define TPUART_REPETITION_FILTER_COUNT 50
#endif

// Erkennt Wiederholungen (TP_FRAME_FLAG_FILTERED). Ein Eintrag je Absender mit 16-Bit-Fingerabdruck seines
// letzten Telegramms - auf TP1 hat ein Absender nie zwei Telegramme gleichzeitig offen.
//
// Bewusst KEINE Verfallsfrist: wann die Wiederholung kommt, ist nicht begrenzt. Nicht wieder einbauen.
// Hingenommen: fehlt das Original und stammt der Fingerabdruck von einem identischen früheren Telegramm,
// wird das einzige Exemplar als Wiederholung markiert.
//
// Nur Hauptkontext (Receiver::processQueue()).
class RepetitionFilter
{
  private:
    // 6 Byte je Absender. Frei ist ein Platz mit Absender 0.0.0 - als Absenderadresse nicht vergeben, im
    // knx-Stack der Platzhalter für "keine Adresse".
    struct Entry
    {
        uint16_t _source;
        uint16_t _checksum;
        uint16_t _seen; // _counter beim letzten Telegramm - nur für die Verdrängung
    };

    Entry _entries[TPUART_REPETITION_FILTER_COUNT] = {};

    // Zählt je check(). Läuft über; das Alter rechnet über die Differenz, erst ab 65536 Telegrammen ohne
    // Wiederkehr erscheint ein Eintrag fälschlich jung und wird später verdrängt.
    uint16_t _counter = 0;

    // CRC-16/SPI-FUJITSU ohne Prüfsumme, Wiederholungs-Bit fest gesetzt - Original und Wiederholung gleich.
    static uint16_t fingerprint(const Frame &frame);

  public:
    // War genau dieses Telegramm von diesem Absender gerade da? Merkt es sich. Für JEDES Telegramm aufrufen.
    bool check(const Frame &frame);

    void clear();

    // Belegte Plätze - für die Diagnose.
    size_t size() const;
};

} // namespace TPUart
