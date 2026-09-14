#include "TPUart/Interface/RP2040.h"
#ifdef ARDUINO_ARCH_RP2040
#include <functional>

// DMA_IRQ_0 teilen sich alle Kanäle: ein Handler prüft den Kanal und ruft die registrierte Instanz, die nur
// ein Flag setzt. Der RP2040 hat zwei UARTs, also höchstens zwei Instanzen.
constexpr uint TPUART_RP2040_UART_COUNT = 2;

struct TPUartDmaCompleteRegistration
{
    int _channel = -1;
    std::function<void(void)> _callback;
};

static TPUartDmaCompleteRegistration __tpuartDmaCompleteCallbacks[TPUART_RP2040_UART_COUNT];
static bool __tpuartDmaIrqInstalled = false;

static void __time_critical_func(__tpuartDmaIrqHandler)()
{
    for (uint i = 0; i < TPUART_RP2040_UART_COUNT; i++)
    {
        int channel = __tpuartDmaCompleteCallbacks[i]._channel;
        if (channel < 0 || !dma_channel_get_irq0_status(channel)) continue;
        dma_channel_acknowledge_irq0(channel);
        if (__tpuartDmaCompleteCallbacks[i]._callback) __tpuartDmaCompleteCallbacks[i]._callback();
    }
}

// TX-Interrupt: je UART eine eigene Leitung, also ohne Suche - und ohne std::function, der Pfad feuert je Byte.
static TPUart::Interface::RP2040 *__tpuartTxIrqInstance[TPUART_RP2040_UART_COUNT] = {};

static void __time_critical_func(__tpuartUart0IrqHandler)()
{
    if (__tpuartTxIrqInstance[0]) __tpuartTxIrqInstance[0]->onTxInterrupt();
}

static void __time_critical_func(__tpuartUart1IrqHandler)()
{
    if (__tpuartTxIrqInstance[1]) __tpuartTxIrqInstance[1]->onTxInterrupt();
}

namespace TPUart
{
namespace Interface
{

// Zweierpotenz, sonst stünde eine Division (Bibliotheksaufruf) im TX-Interrupt.
static_assert((TPUART_TX_INTERFACE_BUFFER & (TPUART_TX_INTERFACE_BUFFER - 1)) == 0, "TPUART_TX_INTERFACE_BUFFER muss eine Zweierpotenz sein - sonst steht eine Division im TX-Interrupt");

RP2040::RP2040(pin_size_t rx, pin_size_t tx, uart_inst_t *uart, bool irq, bool dma) : _rx(rx), _tx(tx), _uart(uart)
{
    (void)irq; // siehe Header - beide Schalter fallen weg: der IRQ wird immer gebraucht (DMA-Neustart und
    (void)dma; // Sendepuffer), die DMA immer benutzt.

    _dmaChannel = dma_claim_unused_channel(true);
    _dmaConfig = dma_channel_get_default_config(_dmaChannel);
    channel_config_set_transfer_data_size(&_dmaConfig, DMA_SIZE_8);
    channel_config_set_read_increment(&_dmaConfig, false);
    channel_config_set_write_increment(&_dmaConfig, true);
    channel_config_set_high_priority(&_dmaConfig, true);
    channel_config_set_ring(&_dmaConfig, true, TPUART_RP2040_RX_BUFFER_EXP);
    if (_uart == uart0) channel_config_set_dreq(&_dmaConfig, DREQ_UART0_RX);
    if (_uart == uart1) channel_config_set_dreq(&_dmaConfig, DREQ_UART1_RX);
    dma_channel_set_read_addr(_dmaChannel, &uart_get_hw(_uart)->dr, false);
    dma_channel_set_trans_count(_dmaChannel, TPUART_RP2040_TRANSFER_COUNT, false);
    dma_channel_set_config(_dmaChannel, &_dmaConfig, false);

    for (uint i = 0; i < TPUART_RP2040_UART_COUNT; i++)
    {
        if (__tpuartDmaCompleteCallbacks[i]._channel != -1) continue;
        __tpuartDmaCompleteCallbacks[i]._channel = _dmaChannel;
        __tpuartDmaCompleteCallbacks[i]._callback = std::bind(&RP2040::onDmaComplete, this);
        break;
    }
    // Geteilt, nicht exklusiv: andere Bibliotheken nutzen DMA_IRQ_0 auch - exklusiv panickte vor setup().
    if (!__tpuartDmaIrqInstalled)
    {
        irq_add_shared_handler(DMA_IRQ_0, __tpuartDmaIrqHandler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(DMA_IRQ_0, true);
        __tpuartDmaIrqInstalled = true;
    }
    dma_channel_set_irq0_enabled(_dmaChannel, true);

    critical_section_init(&_txSection);

    // TX-Interrupt im Konstruktor, nicht in begin() (läuft je Baudratenkandidat). Hat jemand die Leitung
    // exklusiv (arduino-pico SerialUART), bleibt er aus - dann leert der Tick den Ring.
    uint index = uartIndex();
    uint irqNum = index == 1 ? UART1_IRQ : UART0_IRQ;

    if (__tpuartTxIrqInstance[index] == nullptr && irq_get_exclusive_handler(irqNum) == nullptr)
    {
        __tpuartTxIrqInstance[index] = this;
        irq_add_shared_handler(irqNum, index == 1 ? __tpuartUart1IrqHandler : __tpuartUart0IrqHandler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(irqNum, true);
        _txIrqInstalled = true;
    }

    _txRestore = gpio_get_function(_tx);
    _rxRestore = gpio_get_function(_rx);
    gpio_set_function(_rx, GPIO_FUNC_UART);
    gpio_set_function(_tx, GPIO_FUNC_UART);
}

RP2040::~RP2040()
{
    end();

    dma_channel_set_irq0_enabled(_dmaChannel, false);
    for (uint i = 0; i < TPUART_RP2040_UART_COUNT; i++)
    {
        if (__tpuartDmaCompleteCallbacks[i]._channel != _dmaChannel) continue;
        __tpuartDmaCompleteCallbacks[i]._channel = -1;
        __tpuartDmaCompleteCallbacks[i]._callback = nullptr;
        break;
    }

    // Letzte Instanz: Handler abmelden. Die geteilte Leitung bleibt freigeschaltet.
    bool anyDmaLeft = false;
    for (uint i = 0; i < TPUART_RP2040_UART_COUNT; i++)
        if (__tpuartDmaCompleteCallbacks[i]._channel != -1) anyDmaLeft = true;

    if (__tpuartDmaIrqInstalled && !anyDmaLeft)
    {
        irq_remove_handler(DMA_IRQ_0, __tpuartDmaIrqHandler);
        __tpuartDmaIrqInstalled = false;
    }

    dma_channel_abort(_dmaChannel);
    dma_channel_cleanup(_dmaChannel);
    dma_channel_unclaim(_dmaChannel);

    if (_txIrqInstalled)
    {
        uint index = uartIndex();
        uint irqNum = index == 1 ? UART1_IRQ : UART0_IRQ;

        irq_remove_handler(irqNum, index == 1 ? __tpuartUart1IrqHandler : __tpuartUart0IrqHandler);

        // Nur abschalten, wenn niemand sonst an der Leitung hängt.
        if (!irq_has_shared_handler(irqNum)) irq_set_enabled(irqNum, false);

        __tpuartTxIrqInstance[index] = nullptr;
        _txIrqInstalled = false;
    }

    critical_section_deinit(&_txSection);

    gpio_set_function(_rx, _rxRestore);
    gpio_set_function(_tx, _txRestore);
}

void RP2040::begin(uint32_t baud)
{
    if (_running) end();

    uart_init(_uart, baud);
    uart_set_format(_uart, 8, 1, UART_PARITY_EVEN);
    uart_set_hw_flow(_uart, false, false);
    uart_set_fifo_enabled(_uart, false); // DMA liest direkt aus dem Datenregister

    // Kanal hier vollständig neu konfigurieren - end() darf die Konfiguration nicht tragen müssen.
    dma_channel_set_read_addr(_dmaChannel, &uart_get_hw(_uart)->dr, false);
    dma_channel_set_config(_dmaChannel, &_dmaConfig, false);
    dma_channel_set_trans_count(_dmaChannel, TPUART_RP2040_TRANSFER_COUNT, false);
    _dmaReaderCount = 0;
    _dmaTransferBase = 0;
    _pendingRestart = false;
    _ringOverflow = false;
    _txHead = 0;
    _txTail = 0;

    // TXIM ist nach uart_init() aus; Latch zur Vorsicht löschen.
    hw_clear_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
    uart_get_hw(_uart)->icr = UART_UARTICR_TXIC_BITS;

    dma_channel_set_write_addr(_dmaChannel, _dmaBuffer, true);

    _running = true;
}

void RP2040::end()
{
    if (!_running) return;

    // Erst die ISR stilllegen (_running, TXIM), dann den Block abschalten - beides in der Sperre.
    critical_section_enter_blocking(&_txSection);
    _running = false;
    hw_clear_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
    uart_get_hw(_uart)->icr = UART_UARTICR_TXIC_BITS;
    critical_section_exit(&_txSection);

    // Gepufferte Bytes gehen verloren - kein Warten.

    // Abort plus Quittieren des Completion-IRQ, sonst löste er nach begin() einen falschen Neustart aus.
    // NICHT dma_channel_cleanup(): das verwirft die Kanalkonfiguration.
    dma_channel_abort(_dmaChannel);
    dma_channel_acknowledge_irq0(_dmaChannel);
    uart_deinit(_uart);
}

// Fortlaufende Anzahl empfangener Bytes über DMA-Neustarts hinweg; nur Differenzen werden gebildet.
uint32_t RP2040::dmaTransferCount()
{
    return _dmaTransferBase + (TPUART_RP2040_TRANSFER_COUNT - dma_channel_hw_addr(_dmaChannel)->transfer_count);
}

void RP2040::onDmaComplete()
{
    _pendingRestart = true;
}

void RP2040::restartDma()
{
    // Lesezeiger bleibt. Basis ist der AKTUELLE Gesamtstand, nicht +TRANSFER_COUNT - sonst verschöbe ein
    // Neustart bei laufendem Zähler Ring-Index und Schreibposition dauerhaft.
    uint32_t total = dmaTransferCount();
    _dmaTransferBase = total;

    uint32_t writeIndex = total % TPUART_RP2040_RX_BUFFER_SIZE;
    dma_channel_set_trans_count(_dmaChannel, TPUART_RP2040_TRANSFER_COUNT, false);
    dma_channel_set_write_addr(_dmaChannel, _dmaBuffer + writeIndex, true);
}

void RP2040::checkRestart()
{
    if (!_pendingRestart) return;
    _pendingRestart = false;
    restartDma();
}

// Schiebt nach, was die UART sofort annimmt, und setzt TXIM. Vorbedingung: _txSection gehalten.
// Der TX-Interrupt entsteht nur am Übergang auf "Halteregister leer" - das eigene Schreiben erzeugt ihn.
void __time_critical_func(RP2040::pumpTx)()
{
    while (_txTail != _txHead && uart_is_writable(_uart))
        uart_get_hw(_uart)->dr = _txBuffer[_txTail++ % TPUART_TX_INTERFACE_BUFFER];

    // Ohne eigenen Handler nie TXIM setzen - ein fremder Handler quittierte ihn nicht (Interrupt-Sturm).
    if (_txTail == _txHead || !_txIrqInstalled)
        hw_clear_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
    else
        hw_set_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
}

// TX-Interrupt: hält die Leitung zwischen zwei Ticks versorgt.
void __time_critical_func(RP2040::onTxInterrupt)()
{
    // Auch die _running-Prüfung in der Sperre - sonst träfe ein Registerzugriff einen abgeschalteten Block.
    critical_section_enter_blocking(&_txSection);

    if (_running && (uart_get_hw(_uart)->mis & UART_UARTMIS_TXMIS_BITS))
    {
        // Erst quittieren, dann nachschieben - und nur das TX-Bit (kombinierter UART-Interrupt).
        uart_get_hw(_uart)->icr = UART_UARTICR_TXIC_BITS;
        pumpTx();
    }

    critical_section_exit(&_txSection);
}

size_t RP2040::available()
{
    if (!_running) return 0;

    critical_section_enter_blocking(&_txSection);
    pumpTx();
    critical_section_exit(&_txSection);

    checkRestart();

    // Mehr als eine Ringfüllung ist nicht lesbar (Rest meldet overflow()).
    uint32_t pending = dmaTransferCount() - _dmaReaderCount;
    return pending > TPUART_RP2040_RX_BUFFER_SIZE ? TPUART_RP2040_RX_BUFFER_SIZE : pending;
}

// Pumpt als Rückfallpfad, falls der TX-Interrupt nicht beansprucht werden konnte.
size_t RP2040::availableForWrite()
{
    if (!_running) return 0;

    critical_section_enter_blocking(&_txSection);
    pumpTx();
    uint32_t used = _txHead - _txTail;
    critical_section_exit(&_txSection);

    return TPUART_TX_INTERFACE_BUFFER - used;
}

// Legt das Byte in den Ring, ohne auf die Hardware zu warten.
bool RP2040::write(char value)
{
    if (!_running) return false;

    critical_section_enter_blocking(&_txSection);

    // Pumpen, anhängen, erneut pumpen - in einem Abschnitt, damit Platzprüfung und Anhängen denselben Stand sehen.
    pumpTx();

    bool full = (_txHead - _txTail) >= TPUART_TX_INTERFACE_BUFFER; // Aufrufer prüft vorher availableForWrite()
    if (!full)
    {
        _txBuffer[_txHead % TPUART_TX_INTERFACE_BUFFER] = (uint8_t)value;
        _txHead++;
        pumpTx();
    }

    critical_section_exit(&_txSection);
    return !full;
}

int RP2040::read()
{
    if (!available()) return -1;

    // Die DMA hat den Leser überholt: Verlust merken (die Korrektur beseitigt die Differenz, bevor
    // overflow() fragt) und am ältesten noch gültigen Byte aufsetzen.
    if (dmaTransferCount() - _dmaReaderCount > TPUART_RP2040_RX_BUFFER_SIZE)
    {
        _ringOverflow = true;
        _dmaReaderCount = dmaTransferCount() - TPUART_RP2040_RX_BUFFER_SIZE + 1;
    }

    return _dmaBuffer[_dmaReaderCount++ % TPUART_RP2040_RX_BUFFER_SIZE];
}

bool RP2040::overflow()
{
    if (!_running) return false;

    bool result = false;

    // Hardware-Overrun: die DMA hat das Datenregister nicht rechtzeitig geleert.
    if (uart_get_hw(_uart)->rsr & UART_UARTRSR_OE_BITS)
    {
        uart_get_hw(_uart)->rsr = 0; // Schreibzugriff löscht die Fehlerflags
        result = true;
    }

    // Ringüberlauf: der Leser war zu langsam. Erkannt in read(), daher das Latch.
    if (_ringOverflow)
    {
        _ringOverflow = false;
        result = true;
    }

    return result;
}

void RP2040::flush()
{
    if (!_running) return;
    while (available())
        read();
}

} // namespace Interface
} // namespace TPUart

#endif
