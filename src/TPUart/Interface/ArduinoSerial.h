#pragma once
#include <Arduino.h>

#include "TPUart/Interface/Abstract.h"
#include "TPUart/Transmitter.h" // nur für TPUART_TX_INTERFACE_BUFFER - die Anforderung gehört dem Transmitter

namespace TPUart
{
namespace Interface
{

// DIE EINZIGE KLASSE DER LIBRARY, DIE VOLLSTÄNDIG IM HEADER STEHT - und das ist kein Versehen, sondern
// unvermeidlich: als Template wird sie erst beim Aufrufer instanziiert (mit dessen Serial-Typ, siehe
// test/test_tpuart/interface_check.cpp: ArduinoSerial<decltype(Serial1)>). Der Compiler braucht die Rümpfe
// an genau dieser Stelle. Eine
// .cpp würde nur helfen, wenn man die möglichen Typen hier vorab explizit instanziierte - womit der Sinn des
// Templates weg wäre, denn welche Stream-Klasse eine Plattform mitbringt, weiß nur der Aufrufer.
//
// DER SCHWIERIGE TEIL DIESES ADAPTERS IST availableForWrite(), und er hat beide Fehler gemacht, die dort
// möglich sind (siehe TPUART_TX_INTERFACE_BUFFER):
//
//   ZU KLEIN: durchgereicht, was die Serial-Klasse meldet. SerialUART von arduino-pico liefert
//   "(uart_is_writable(_uart)) ? 1 : 0", also höchstens 1, und Print::availableForWrite() gibt als Vorgabe
//   sogar 0 zurück. Eine 4-Byte-Steuersequenz bekam damit nie ihren Platz, blieb ungeteilt in der
//   Warteschlange stehen (richtig so, zerteilen darf sie sich nicht) und blockierte damit auch den
//   Telegrammpfad - beobachtet als Dauerlauf von "CTRL OVERFLOW" in der Env pico_serial.
//
//   ZU GROSS: der naheliegende Gegenfehler, und er ist hier nicht zu sehen. SerialUART schreibt mit
//   uart_putc_raw() direkt in die Hardware, und uart_init() des SDK lässt die TX-FIFO AN - 32 Byte tief.
//   "uart_is_writable" bleibt also wahr, bis 32 Bytes drinliegen. Wer einfach darauf losschreibt, hat bei
//   19200 Baud 18ms Rückstau unter sich, und das nächste Byte ist womöglich ein U_Ackn.req mit 2,8ms Frist.
//
// Beides löst dieselbe Rechnung: der Adapter FÜHRT BUCH ÜBER DIE LEITUNG, statt zu fragen. Aus der Baudrate
// folgt die Bytezeit (8E1 = 11 Bit je Zeichen), jedes übergebene Byte schreibt den Fahrplan fort, und daraus
// ergibt sich jederzeit, wie viele Bytes noch unter uns stecken. Gemeldet wird der Platz bis zur erlaubten
// Tiefe - eine Zahl, die stimmt, und eine Tiefe, die die Quittung nicht begräbt.
//
// DAZU EIN VIER BYTE TIEFER ZWISCHENPUFFER. Hier stand einmal, den brauche es nicht ("die FIFO darunter
// IST der Ring") - das war falsch, und zwar nicht wegen der Tiefe, sondern wegen der ZUSAGE:
// availableForWrite() ist eine Reservierung, und ohne Puffer hing deren Einhaltung an der eingepackten
// Klasse. Die kann ablehnen, ohne dass diese Ebene den Grund kennt - SerialPIO tut es, wenn seine
// CoreMutex belegt ist. Weil der Aufrufer Gruppen unteilbar absetzt, lag dann eine halbe Sequenz auf der
// Leitung: der Chip meldete PE und bestätigte das Telegramm nie. Der Puffer nimmt an, was zugesagt war,
// und schiebt es nach; die erlaubte Tiefe bleibt dieselbe, weil beides zusammen gezählt wird.
//
// Die Schätzung irrt in die unschädliche Richtung: die 11 Bit sind exakt (8E1), und ein Byte kann nur später
// fertig sein als berechnet, nie früher - dann wird eben etwas mehr gebremst als nötig.
template <class T>
class ArduinoSerial : public Abstract
{
  private:
    T &_serial;
    bool _running = false;

    uint32_t _byteTimeUs = 0;    // 0 = unbekannt, dann wird nicht gebremst
    uint32_t _lineBusyUntil = 0; // geschätzter Zeitpunkt, an dem alles Übergebene draußen ist

    // DER ZWISCHENPUFFER, und er ist der Preis für eine haltbare Zusage. availableForWrite() ist eine
    // RESERVIERUNG: wer n meldet, muss die nächsten n Schreibvorgänge annehmen, weil der Aufrufer Gruppen
    // unteilbar absetzt. Ohne Puffer hing das an der eingepackten Klasse - und die kann aus Gründen
    // ablehnen, die diese Ebene nicht kennt: SerialPIO nimmt in write() eine CoreMutex und liefert 0,
    // wenn sie belegt ist (Reentranz zwischen Tick und Hauptkontext). Genau das zerriss Sequenzen.
    //
    // ER VERSTECKT KEINEN RÜCKSTAND, und das war der Einwand gegen einen Ring an dieser Stelle: die Tiefe
    // bleibt gedeckelt, gezählt wird jetzt nur die Summe aus "noch bei uns" und "geschätzt noch unter uns".
    // Mehr als TPUART_TX_INTERFACE_BUFFER Bytes stecken damit nie zwischen Aufrufer und Leitung - dieselbe
    // Zusage wie vorher, nur eingehalten.
    uint8_t _txBuffer[TPUART_TX_INTERFACE_BUFFER] = {};
    uint8_t _txCount = 0;

    // Schiebt so viel wie möglich weiter, ohne je zu blockieren. Was nicht durchgeht, bleibt liegen und
    // wird beim nächsten Tick erneut versucht - der Aufrufer merkt davon nichts, denn seine Reservierung
    // rechnet die liegengebliebenen Bytes mit.
    void pump()
    {
        while (_txCount > 0)
        {
            if (_reportsWriteSpace && _serial.availableForWrite() <= 0) return;
            if (_serial.write(_txBuffer[0]) != 1) return;

            for (uint8_t i = 1; i < _txCount; i++)
                _txBuffer[i - 1] = _txBuffer[i];

            _txCount--;

            // Fahrplan fortschreiben: ist die Leitung noch belegt, hängt das Byte hinten dran, sonst
            // beginnt es jetzt.
            uint32_t now = micros();
            if ((int32_t)(_lineBusyUntil - now) < 0) _lineBusyUntil = now;
            _lineBusyUntil += _byteTimeUs;
        }
    }

    // Meldet die Serial-Klasse ihren freien Sendeplatz überhaupt? Festgestellt in begin(): unmittelbar
    // danach ist der Sendepuffer leer, eine funktionierende Implementierung MUSS dort also mehr als 0
    // melden. Tut sie es nicht, kennt sie den Dienst nicht (siehe Print::availableForWrite) - dann wird
    // ohne diese Rückfrage geschrieben und allein die Zeitrechnung bremst. Umgekehrt ist die Rückfrage der
    // Schutz davor, dass ein write() blockiert, falls die Leitung langsamer ist als die Baudrate vermuten
    // lässt.
    bool _reportsWriteSpace = false;

    // Wie viele Bytes stecken geschätzt noch unter uns? Aufgerundet - ein angebrochenes Byte zählt mit.
    size_t outstanding() const
    {
        if (_byteTimeUs == 0) return 0;

        int32_t remaining = (int32_t)(_lineBusyUntil - micros());
        if (remaining <= 0) return 0;

        return (size_t)(((uint32_t)remaining + _byteTimeUs - 1) / _byteTimeUs);
    }

  public:
    ArduinoSerial(T &serial) : _serial(serial) {}
    ~ArduinoSerial() { end(); }

    void begin(uint32_t baud) override
    {
        if (_running) end();

        _serial.begin(baud, SERIAL_8E1);

        _byteTimeUs = baud > 0 ? (uint32_t)(11000000UL / baud) : 0;
        _lineBusyUntil = micros();
        _reportsWriteSpace = _serial.availableForWrite() > 0;
        _txCount = 0; // was vor dem Neuaufsetzen liegenblieb, gehört zu einer anderen Baudrate
        _running = true;
    }

    void end() override
    {
        if (!_running) return;
        _running = false;
        _serial.end();
    }

    // Bewusst NICHT _serial.flush() - das wartet in Arduino auf das Ende des Sendens und fasst den Empfang
    // gar nicht an. Der Vertrag hier lautet: Empfangenes verwerfen, ohne zu blockieren.
    void flush() override
    {
        if (!_running) return;
        while (_serial.available())
            _serial.read();
    }

    size_t available() override
    {
        if (!_running) return 0;

        int count = _serial.available();
        return count > 0 ? (size_t)count : 0;
    }

    // Der Platz bis zur erlaubten Tiefe - nicht der freie Platz der Hardware. Siehe Klassenkommentar.
    //
    // GEZÄHLT WIRD BEIDES: was noch bei uns liegt und was geschätzt noch unter uns steckt. Nur so ist die
    // Zahl eine Reservierung, die write() auch einhalten kann - siehe _txBuffer.
    size_t availableForWrite() override
    {
        if (!_running) return 0;

        pump();

        size_t pending = outstanding() + _txCount;
        if (pending >= TPUART_TX_INTERFACE_BUFFER) return 0;

        return TPUART_TX_INTERFACE_BUFFER - pending;
    }

    int read() override
    {
        if (!available()) return -1;
        return _serial.read();
    }

    bool write(char value) override
    {
        if (!_running) return false;

        pump();

        // DIE EINZIGE ABLEHNUNG, die es noch gibt, und sie hat der Aufrufer selbst zu verantworten: er hat
        // nicht gefragt oder mehr geschrieben als zugesagt. Alles, was die eingepackte Klasse betrifft,
        // wird gepuffert statt abgelehnt - deshalb kann eine Gruppe hier nicht mehr zerreissen.
        if (outstanding() + _txCount >= TPUART_TX_INTERFACE_BUFFER) return false;

        _txBuffer[_txCount++] = (uint8_t)value;

        pump();

        return true;
    }

    bool overflow() override
    {
        if (!_running) return false;

#ifdef ARDUINO_ARCH_RP2040
        return _serial.overflow();
#else
        return false;
#endif
    }
};

} // namespace Interface
} // namespace TPUart
