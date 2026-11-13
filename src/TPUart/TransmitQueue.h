#pragma once
#include <stddef.h>
#include <stdint.h>

#include "TPUart/Types.h"

namespace TPUart
{

class Frame;

// Sendepuffer in Bytes (~140 gewöhnliche Telegramme). Bemessen nach der Zahl gleichzeitig sendebereiter
// KOs mal typischer Telegrammgröße. Eine Ablehnung ist ein verlorenes Telegramm - darüber puffert nichts.
#ifndef TPUART_TX_BUFFER_SIZE
    #define TPUART_TX_BUFFER_SIZE 2048
#endif

// Bytes, die Low nicht belegen darf - ein maximales höherpriores Telegramm passt immer.
#ifndef TPUART_TX_PRIORITY_RESERVE
    #define TPUART_TX_PRIORITY_RESERVE TPUART_BUFFER_SIZE
#endif

// Die Sendewarteschlange: linearer Bytepuffer, nach Rang gruppiert, je Rang in Eingangsreihenfolge. Gehört
// allein dem Hauptkontext - nur deshalb darf umsortiert werden; der Tick bekommt eine gepinnte Vorlage.
//
//     0 ........ _head ...................................... _end[3] ........ SIZE
//     [gepinnt]  [Rang 0][Rang 1][Rang 2][Rang 3            ]  [frei         ]
//                        _end[0] _end[1] _end[2]
//
// Kein Längenpräfix (die Länge steht im Telegramm, hinein kommt nur Geprüftes), kein Umbruch, kein Heap.
class TransmitQueue
{
  private:
    uint8_t _buffer[TPUART_TX_BUFFER_SIZE];

    // Ende je Rang; das letzte ist zugleich das Ende des Belegten.
    size_t _end[TP_PRIORITY_COUNT] = {};

    // Beginn des logischen Inhalts: 0 oder das Ende des gepinnten Eintrags.
    size_t _head = 0;

    bool _pinned = false;

    // Ein Eintrag war unplausibel - im geordneten Betrieb unerreichbar.
    bool _corrupted = false;

    void compact();

  public:
    // Nimmt ein vollständiges, GEPRÜFTES Telegramm auf (prüft selbst nicht - sonst desynchronisiert der
    // Puffer). false, wenn der Platz nicht reicht; für Low gilt die Reserve.
    bool push(const Frame &frame);

    // Vorderster Eintrag, zeigt in den Puffer. Nicht const: ein unplausibler Eintrag verwirft den Puffer.
    const uint8_t *front(size_t &length);

    // Hält den vordersten Eintrag fest - nichts bewegt ihn mehr, solange der Tick daraus liest.
    void pin();

    // Verwirft den gepinnten Eintrag und kompaktiert.
    void pop();

    // Leert alles außer dem gepinnten Eintrag (Wechsel in den Busmonitor).
    void clear();

    bool empty() const;
    bool pinned() const;

    // Belegte Bytes einschließlich eines gepinnten Eintrags.
    size_t used() const;

    // Freier Platz für diesen Rang - für Low kleiner.
    size_t freeFor(uint8_t rank) const;

    // Einmalig abzuholen, wie die übrigen Melder der Schicht.
    bool corrupted();
};

} // namespace TPUart
