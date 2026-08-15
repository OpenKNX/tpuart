#pragma once

#include <stddef.h>
#include <stdint.h>

namespace TPUart
{
namespace Interface
{


// Destruktor und overflow() haben absichtlich einen Rumpf in Abstract.cpp und nicht hier: damit hat die
// Klasse eine Übersetzungseinheit, in der ihre vtable landet, statt sie in jede einzelne zu kopieren.
class Abstract
{
  public:
    virtual ~Abstract();

    virtual void begin(uint32_t baud) = 0;
    virtual void end() = 0;

    // Verwirft alles Empfangene, das noch nicht gelesen wurde. Bezieht sich ausdrücklich NICHT auf den
    // Sendeweg und darf nicht blockieren - anders als Arduinos Stream::flush(), das auf das Ende des
    // Sendens wartet.
    virtual void flush() = 0;

    // Anzahl der bereitliegenden, noch nicht gelesenen Bytes - keine bloße Ja/Nein-Auskunft. Der Aufrufer
    // erkennt daran, ob er dem Bus hinterherhinkt: liegen mehr Bytes bereit, als vom laufenden Frame noch
    // ausstehen, ist dieses Frame auf dem Bus längst vorbei.
    virtual size_t available() = 0;
    virtual int read() = 0;

    // Anzahl der Bytes, die write() JETZT ohne Blockieren annehmen kann - keine bloße Ja/Nein-Auskunft.
    // Der Aufrufer muss mehrere zusammengehörige Bytes am Stück absetzen können (ein Steuerbyte mit bis zu
    // zwei Positionsbytes, oder eine 4-byte-Steuersequenz), ohne dass sich ein dazwischen entstehendes
    // Acknowledge einschiebt. Da alle Implementierungen die Reihenfolge erhalten, genügt es dafür, vorab
    // genug Platz zu prüfen.
    //
    // EINE ZAHL, DIE STIMMEN MUSS, und zwar in beide Richtungen - nicht zu klein und nicht zu groß. Wie
    // viele Bytes am Stück gehen müssen und wie viele höchstens ausstehen dürfen, sagt
    // TPUART_TX_INTERFACE_BUFFER in Transmitter.h; dort steht auch, warum eine tiefe Hardware-FIFO hier
    // schadet statt zu helfen. Beide Fehler sind schon passiert: ArduinoSerial meldete 0/1 (zu klein, eine
    // 4-Byte-Sequenz kam nie durch), ESP32 den freien Platz eines 512 Byte tiefen Treiberpuffers (zu groß,
    // die Quittung hätte hinter allem Übrigen gewartet).
    virtual size_t availableForWrite() = 0;

    // Muss nicht blockieren dürfen: der Aufrufer prüft vorher per availableForWrite(), ob Platz ist.
    virtual bool write(char value) = 0;

    virtual bool overflow();
};

} // namespace Interface
} // namespace TPUart
