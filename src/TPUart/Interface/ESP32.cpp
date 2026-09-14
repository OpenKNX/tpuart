#include "TPUart/Interface/ESP32.h"
#ifdef ARDUINO_ARCH_ESP32

namespace TPUart
{
namespace Interface
{

constexpr int TPUART_ESP32_RX_BUFFER_SIZE = 512;
constexpr int TPUART_ESP32_TX_BUFFER_SIZE = 512;
constexpr int TPUART_ESP32_EVENT_QUEUE_SIZE = 32;

ESP32::ESP32(int rx, int tx, uart_port_t uart) : _rx(rx), _tx(tx), _uart(uart) {}
ESP32::~ESP32() { end(); }

void ESP32::begin(uint32_t baud)
{
    if (_running) end();

    // Alle Felder gesetzt - sonst warnt -Wextra (missing-field-initializers).
    uart_config_t uart_config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    uart_param_config(_uart, &uart_config);

    // ACHTUNG: Pins am internen Flash/PSRAM (ESP32-PICO-V3-02 GPIO 6-11, 16, 17; WROVER 16/17) hängen den
    // Kern ohne jede Ausgabe - Bootloop mit TG1WDT_SYS_RESET. Dann zuerst die Pins prüfen.
    uart_set_pin(_uart, _tx, _rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Fehlschlag: _eventQueue bleibt null, _running false.
    if (uart_driver_install(_uart, TPUART_ESP32_RX_BUFFER_SIZE, TPUART_ESP32_TX_BUFFER_SIZE, TPUART_ESP32_EVENT_QUEUE_SIZE, &_eventQueue, 0) != ESP_OK)
    {
        _eventQueue = nullptr;
        return; // _running bleibt false - alle Methoden liefern damit "nichts da"
    }

    // Jedes Byte sofort herausgeben - sonst stapelt der Treiber bis 120 Bytes oder 10 Symbolzeiten, und die
    // Pausenerkennung sieht erfundene Pausen.
    uart_set_rx_full_threshold(_uart, 1);
    uart_set_rx_timeout(_uart, 1);

    _byteTimeUs = baud > 0 ? (uint32_t)(11000000UL / baud) : 0;
    _lineBusyUntil = micros();
    _overflow = false;
    _running = true;
}

void ESP32::end()
{
    if (!_running) return;
    _running = false;

    uart_driver_delete(_uart);
    _eventQueue = nullptr;
}

// Nur für Overflow-Ereignisse; die Daten kommen über uart_read_bytes().
void ESP32::drainEventQueue()
{
    uart_event_t event;
    while (xQueueReceive(_eventQueue, (void *)&event, 0))
    {
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) _overflow = true;
    }
}

size_t ESP32::available()
{
    if (!_running) return 0;
    drainEventQueue();

    size_t len = 0;
    uart_get_buffered_data_len(_uart, &len);
    return len;
}

size_t ESP32::outstanding() const
{
    if (_byteTimeUs == 0) return 0;

    int32_t remaining = (int32_t)(_lineBusyUntil - micros());
    if (remaining <= 0) return 0;

    return (size_t)(((uint32_t)remaining + _byteTimeUs - 1) / _byteTimeUs);
}

// Platz bis zur erlaubten Tiefe, zusätzlich begrenzt durch den freien Treiberpuffer (sonst blockierte write).
size_t ESP32::availableForWrite()
{
    if (!_running) return 0;

    size_t pending = outstanding();
    if (pending >= TPUART_TX_INTERFACE_BUFFER) return 0;

    size_t allowed = TPUART_TX_INTERFACE_BUFFER - pending;

    size_t len = 0;
    uart_get_tx_buffer_free_size(_uart, &len);

    return len < allowed ? len : allowed;
}

// ACHTUNG: uart_write_bytes() nimmt einen Mutex und ist nicht ISR-fähig.
bool ESP32::write(char value)
{
    if (!_running) return false;

    // Tiefe auch hier prüfen - die Zusage hängt nicht daran, dass der Aufrufer vorher fragt.
    if (outstanding() >= TPUART_TX_INTERFACE_BUFFER) return false;
    if (uart_write_bytes(_uart, &value, 1) != 1) return false;

    // Fahrplan fortschreiben: hinten anhängen oder jetzt beginnen.
    uint32_t now = micros();
    if ((int32_t)(_lineBusyUntil - now) < 0) _lineBusyUntil = now;
    _lineBusyUntil += _byteTimeUs;

    return true;
}

int ESP32::read()
{
    if (!available()) return -1;

    uint8_t value;
    return uart_read_bytes(_uart, &value, 1, 0) == 1 ? value : -1;
}

bool ESP32::overflow()
{
    if (!_running) return false;
    drainEventQueue();

    if (_overflow)
    {
        _overflow = false;
        return true;
    }
    return false;
}

void ESP32::flush()
{
    if (!_running) return;
    uart_flush(_uart);
}

} // namespace Interface
} // namespace TPUart

#endif
