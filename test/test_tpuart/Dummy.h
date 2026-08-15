#pragma once
#include <stddef.h>
#include <stdint.h>

#include <vector>

#include "TPUart/Interface/Abstract.h"
#include "TPUart/Transmitter.h" // nur für TPUART_TX_INTERFACE_BUFFER - die Anforderung gehört dem Transmitter

namespace TPUart
{
namespace Interface
{

// Simulierter UART für Tests ohne echte Hardware-Gegenseite (nutzt aber weiterhin das Arduino-Framework der
// jeweiligen Plattform).
//
// DIESE KLASSE LIEGT BEWUSST IM TESTORDNER UND NICHT IN DER LIBRARY. Sie ist ein Testdoppel, benutzt wird
// sie ausschließlich von test_main.cpp - in src/ wäre sie totes Gewicht in jeder Produktionsfirmware und
// bräuchte eine #ifdef-Klammer, die jemand vergessen kann. Nebenbei ist sie so die Gegenprobe, dass sich
// Interface::Abstract von außen überhaupt implementieren lässt, ganz wie es ein Aufrufer täte.
class Dummy : public Abstract
{
  private:
    struct QueuedByte
    {
        char _data;
        uint32_t _pauseUs; // Pause NACH diesem Byte, bevor das nächste "ankommt" - ist Teil der Auswertung (Timeout-Erkennung)
    };

    std::vector<QueuedByte> _queue;
    size_t _pos = 0;
    uint32_t _nextAvailableAt = 0;
    bool _running = false;
    bool _forcedOverflow = false;
    // Standardmäßig genau die Tiefe, die ein Interface haben DARF (siehe TPUART_TX_INTERFACE_BUFFER) - so
    // verhält sich der Dummy wie echte Hardware und nicht großzügiger. Tests, die den Fall "zu wenig Platz
    // für eine zusammengehörige Byte-Folge" brauchen, stellen ihn mit setWriteCapacity() kleiner.
    size_t _writeCapacity = TPUART_TX_INTERFACE_BUFFER;
    size_t _writeUsed = 0;

    // Der Sendepuffer läuft mit der Zeit leer, wie bei echter Hardware. Ohne das bliebe er nach
    // _writeCapacity Bytes für immer voll, availableForWrite() lieferte dauerhaft 0 und der gesamte
    // Sendeweg stünde - inklusive Quittung, Steuercodes und der Baudratenerkennung, die pro Versuch ein
    // Byte schreibt. Bei 64 Byte Kapazität und einem Versuch pro Sekunde war die Dummy-Umgebung damit nach
    // gut einer Minute tot.
    uint32_t _byteTimeUs = 573; // 11 Bit bei 19200, bis begin() die echte Baudrate liefert
    uint32_t _lastDrainAt = 0;

    // Mitschrift des Gesendeten für Tests. GEDECKELT, weil sie sonst in einem Dauerlauf unbegrenzt wächst -
    // ein Testfall, der die Sammlung wiederholt fährt, hält sie sonst nicht aus.
    static constexpr size_t WRITTEN_LIMIT = 4096;
    std::vector<uint8_t> _written;

    // Gibt frei, was die "Hardware" seit dem letzten Blick abgeschickt hätte.
    void drainByTime();

  public:
    // Fügt ein simuliertes Empfangsbyte hinzu. pauseUs ist die Zeit, die nach diesem Byte vergehen soll,
    // bevor das nächste Byte als "angekommen" gilt - relevant, da die Pause selbst Teil der Auswertung
    // (Timeout-/Frame-Ende-Erkennung) ist.
    void addByte(char data, uint32_t pauseUs);

    void clearData();

    const std::vector<uint8_t> &writtenBytes() const;
    void clearWrittenBytes();

    void forceOverflow(bool state);

    // Frei einstellbar, damit Tests auch den Fall "zu wenig Platz für eine zusammengehörige Byte-Folge"
    // abbilden können.
    void setWriteCapacity(size_t capacity);

    // Gibt den Sendepuffer sofort komplett frei - für Tests, die nicht auf die Bytezeit warten wollen.
    void drainWritten();

    // Spult das Skript bewusst NICHT zurück. Echte Interfaces verwerfen beim end()/begin()-Paar alles
    // Gepufferte und liefern danach nur, was neu ankommt. Ein Rücksetzen von _pos würde die Queue erneut
    // von vorn abspielen - und damit ausgerechnet die Baudratenerkennung gegen ein Verhalten testen, das
    // die Hardware nicht hat: sie macht pro Kandidat end()/begin() und sähe dann jedes Mal dasselbe
    // U_Reset.ind wieder, das real längst weg wäre. Zum Neuaufsetzen gibt es clearData().
    void begin(uint32_t baud) override;
    void end() override;

    // Verwirft alles noch nicht Gelesene - NICHT zurückspulen, sonst käme die Queue ein zweites Mal.
    void flush() override;

    // Zählt alle Bytes, deren Ankunftszeitpunkt bereits erreicht ist - nicht nur das nächste. Nur so lässt
    // sich in Tests der Fall nachstellen, dass die Verarbeitung dem Bus hinterherhinkt und ein Frame längst
    // vollständig eingetroffen ist. Vergleiche wrap-sicher (micros() läuft nach ~71min über).
    size_t available() override;

    size_t availableForWrite() override;

    int read() override;
    bool write(char value) override;
    bool overflow() override;
};

} // namespace Interface
} // namespace TPUart
