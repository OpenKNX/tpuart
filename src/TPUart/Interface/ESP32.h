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

// ESP32-UART. Der Treiber-Sendepuffer (512 Byte) dient nur als Durchlauf: gemeldet wird nicht sein freier
// Platz, sondern die erlaubte Tiefe aus der Buchführung über die Leitung - wie in ArduinoSerial.
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

    // Geschätzt noch unter uns steckende Bytes, aufgerundet.
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
