#pragma once
#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "TPUart/Interface/Abstract.h"
#include "TPUart/Transmitter.h" // nur für TPUART_TX_INTERFACE_BUFFER - die Anforderung gehört dem Transmitter

namespace TPUart
{
namespace Interface
{

// Zum Sendeweg, und das ist hier der Knackpunkt: der Treiber bekommt einen 512 Byte tiefen Sendepuffer
// (uart_driver_install lässt nur 0 oder mehr als eine FIFO-Länge zu, 4 wäre gar nicht einstellbar). Dessen
// freien Platz zu melden wäre falsch - was dort liegt, verzögert alles Folgende, und das nächste Byte ist
// womöglich ein U_Ackn.req mit 2,8ms Frist. Bei 512 Byte wären das Sekunden.
//
// Deshalb wird der Puffer nicht gefüllt, sondern nur als Durchlauf benutzt: der Adapter führt Buch über die
// Leitung (Bytezeit aus der Baudrate, 8E1 = 11 Bit je Zeichen) und gibt nur so viel weiter, wie bis zur
// erlaubten Tiefe passt - dieselbe Rechnung wie im ArduinoSerial, wo sie ausführlich begründet ist.
class ESP32 : public Abstract
{
  private:
    int _rx, _tx;
    uart_port_t _uart;
    bool _running = false;
    QueueHandle_t _eventQueue = nullptr;
    bool _overflow = false;

    uint32_t _byteTimeUs = 0;    // 0 = unbekannt, dann wird nicht gebremst
    uint32_t _lineBusyUntil = 0; // geschätzter Zeitpunkt, an dem alles Übergebene draußen ist

    void drainEventQueue();

    // Wie viele Bytes stecken geschätzt noch unter uns? Aufgerundet - ein angebrochenes Byte zählt mit.
    size_t outstanding() const;

  public:
    ESP32(int rx, int tx, uart_port_t uart);
    ~ESP32();

    void begin(uint32_t baud) override;
    void end() override;
    void flush() override;

    size_t available() override;
    size_t availableForWrite() override;
    int read() override;
    bool write(char value) override;
    bool overflow() override;
};

} // namespace Interface
} // namespace TPUart

#endif
