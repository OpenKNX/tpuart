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
    struct Entry
    {
        uint16_t _source;
        uint16_t _checksum;
        uint32_t _timestamp;
        bool _used;
    };

    Entry _entries[TPUART_REPETITION_FILTER_COUNT] = {};

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
