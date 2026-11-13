#pragma once
#include <Arduino.h>

#include "TPUart/Interface/Abstract.h"
#include "TPUart/Transmitter.h" // nur für TPUART_TX_INTERFACE_BUFFER - die Anforderung gehört dem Transmitter

namespace TPUart
{
namespace Interface
{

// Adapter für eine Arduino-Stream-Klasse. Als Template vollständig im Header.
//
// availableForWrite() fragt nicht die Serial-Klasse (SerialUART meldet höchstens 1, Print 0; die TX-FIFO
// darunter wäre 32 Byte tief und begrübe die Quittungsfrist), sondern führt Buch über die Leitung: aus der
// Baudrate folgt die Bytezeit (8E1 = 11 Bit), jedes Byte schreibt den Fahrplan fort.
//
// Der 4-Byte-Zwischenpuffer hält die Zusage von availableForWrite(), auch wenn die Serial-Klasse ablehnt
// (SerialPIO bei belegter CoreMutex) - sonst zerrisse eine Gruppe.
template <class T>
class ArduinoSerial : public Abstract
{
  private:
    T &_serial;
    bool _running = false;

    uint32_t _byteTimeUs = 0;    // 0 = unbekannt, dann wird nicht gebremst
    uint32_t _lineBusyUntil = 0; // geschätzter Zeitpunkt, an dem alles Übergebene draußen ist

    // Zwischenpuffer; gezählt wird zusammen mit dem Geschätzten unter uns - die Tiefe bleibt gedeckelt.
    uint8_t _txBuffer[TPUART_TX_INTERFACE_BUFFER] = {};
    uint8_t _txCount = 0;

    // Schiebt weiter, ohne zu blockieren; Liegengebliebenes kommt beim nächsten Aufruf.
    void pump()
    {
        while (_txCount > 0)
        {
            if (_reportsWriteSpace && _serial.availableForWrite() <= 0) return;
            if (_serial.write(_txBuffer[0]) != 1) return;

            for (uint8_t i = 1; i < _txCount; i++)
                _txBuffer[i - 1] = _txBuffer[i];

            _txCount--;

            // Fahrplan fortschreiben: hinten anhängen oder jetzt beginnen.
            uint32_t now = micros();
            if ((int32_t)(_lineBusyUntil - now) < 0) _lineBusyUntil = now;
            _lineBusyUntil += _byteTimeUs;
        }
    }

    // Meldet die Serial-Klasse freien Sendeplatz? Festgestellt in begin() bei leerem Puffer.
    bool _reportsWriteSpace = false;

    // Geschätzt noch unter uns steckende Bytes, aufgerundet.
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

    // Nicht _serial.flush() - das wartet aufs Senden. Hier: Empfangenes verwerfen, ohne zu blockieren.
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

    // Platz bis zur erlaubten Tiefe: Gepuffertes plus geschätzt noch Ausstehendes.
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

        // Abgelehnt wird nur, was über die Zusage hinausgeht.
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
