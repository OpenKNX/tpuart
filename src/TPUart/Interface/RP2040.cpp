#include "TPUart/Interface/RP2040.h"
#ifdef ARDUINO_ARCH_RP2040
#include <hardware/sync.h>

std::function<void(void)> __tpuartInterrupt0Callback;
std::function<void(void)> __tpuartInterrupt1Callback;

void __time_critical_func(__tpuartInterrupt0)()
{
    __tpuartInterrupt0Callback();
}

void __time_critical_func(__tpuartInterrupt1)()
{
    __tpuartInterrupt1Callback();
}

namespace TPUart
{
    namespace Interface
    {
        RP2040::RP2040(pin_size_t rx, pin_size_t tx, uart_inst_t *uart, bool irq, bool dma) : _rx(rx), _tx(tx), _uart(uart), _irq(irq), _dma(dma)
        {
            // The hardware ring masks the write address, so the buffer must start on a RING_BYTES
            // boundary. This object is heap-allocated, so that depends on over-aligned new being
            // honoured; without it the wrap lands mid-buffer and corrupts RX silently. Fall back instead.
            if (_dma && ((uintptr_t)_dmaBuffer & (TPUART_RP2040_RING_BYTES - 1)) != 0)
                _dma = false;

            if (_dma)
            {
                _dmaChannel = dma_claim_unused_channel(true); // get free channel for dma
                _dmaConfig = dma_channel_get_default_config(_dmaChannel);
                channel_config_set_transfer_data_size(&_dmaConfig, TPUART_RP2040_DMA_XFER);
                channel_config_set_read_increment(&_dmaConfig, false);
                channel_config_set_write_increment(&_dmaConfig, true);
                channel_config_set_high_priority(&_dmaConfig, true);
                channel_config_set_ring(&_dmaConfig, true, TPUART_RP2040_RING_EXP); // ring wraps on BYTES
                if (_uart == uart0) channel_config_set_dreq(&_dmaConfig, DREQ_UART0_RX);
                if (_uart == uart1) channel_config_set_dreq(&_dmaConfig, DREQ_UART1_RX);
                dma_channel_set_read_addr(_dmaChannel, &uart_get_hw(_uart)->dr, false);
                dma_channel_set_trans_count(_dmaChannel, TPUART_RP2040_TRANSFER_COUNT, false);
                dma_channel_set_config(_dmaChannel, &_dmaConfig, false);
            }

            _txRestore = gpio_get_function(_tx);
            _rxRestore = gpio_get_function(_rx);

            gpio_set_function(_rx, GPIO_FUNC_UART);
            gpio_set_function(_tx, GPIO_FUNC_UART);
        }

        RP2040::~RP2040()
        {
            end();

            if (_dma)
            {
                dma_channel_abort(_dmaChannel);
                dma_channel_cleanup(_dmaChannel);
                dma_channel_unclaim(_dmaChannel);
            }

            gpio_set_function(_rx, _rxRestore);
            gpio_set_function(_tx, _txRestore);
        }

        void RP2040::begin(int baud)
        {
            if (_running) end();
            _txHead = 0;
            _txTail = 0;
            _txUrgent = 0;
            _txGlued = false;
#ifdef OPENKNX_DUALCORE
            _txAsync = false; // core1 writes too (loop1); the ring has one producer on the IRQ core
#else
            _txAsync = _irq && _dma; // FIFOs off: one octet per TX interrupt, the first one written by the producer
#endif
            // The status latches survive an end()/begin() pair; a probe at another baud rate would
            // otherwise inherit the previous rate's framing error before a single octet has arrived.
            _lastByteErrored = false;
            _lastByteOverrun = false;
            _lastByteStatus = 0;
            uart_init(_uart, baud);
            uart_set_format(_uart, 8, 1, UART_PARITY_EVEN);
            uart_set_hw_flow(_uart, false, false);
            uart_set_fifo_enabled(_uart, !_dma); // enabled if not using DMA

            if (_irq)
            {
                if (_uart == uart0)
                {
                    __tpuartInterrupt0Callback = std::bind(&RP2040::interrupt, this);
                    irq_set_exclusive_handler(UART0_IRQ, __tpuartInterrupt0);
                    irq_set_enabled(UART0_IRQ, true);
                }

                if (_uart == uart1)
                {
                    __tpuartInterrupt1Callback = std::bind(&RP2040::interrupt, this);
                    irq_set_exclusive_handler(UART1_IRQ, __tpuartInterrupt1);
                    irq_set_enabled(UART1_IRQ, true);
                };

                uart_set_irqs_enabled(_uart, true, false);
            }

            if (_dma)
            {
                _dmaReaderCount = 0;
                dma_channel_set_write_addr(_dmaChannel, _dmaBuffer, true);
            }
            _running = true;
        }

        void RP2040::end()
        {
            if (!_running) return;
            _running = false;

            if (_dma)
            {
                dma_channel_abort(_dmaChannel);
            }

            if (_irq)
            {
                uart_set_irqs_enabled(_uart, false, false);
            }
            _txAsync = false;

            uart_deinit(_uart);
        }

        uint RP2040::dmaTransferCount()
        {
            return TPUART_RP2040_TRANSFER_COUNT - dma_channel_hw_addr(_dmaChannel)->transfer_count;
        }

        uint RP2040::dmaReaderCount()
        {
            return _dmaReaderCount;
        }

        bool RP2040::available()
        {
            if (!_running) return false;
            if (!_dma) return uart_is_readable(_uart);

            return (dmaTransferCount() - _dmaReaderCount) > 0;
        }

        bool RP2040::availableForWrite()
        {
            if (!_running) return false;
            return uart_is_writable(_uart);
        }

        bool RP2040::write(char value)
        {
            if (!_running) return false;
            if (_txAsync) return txPush(&value, 1);
            uart_tx_wait_blocking(_uart);
            uart_putc_raw(_uart, value);
            return true;
        }

        bool RP2040::writeGlued(const char *data, size_t length)
        {
            if (!_running) return false;
            if (_txAsync) return txPush(data, length);
            return Abstract::writeGlued(data, length);
        }

        /**
         * @brief Hands an acknowledge to the TX interrupt, sent at the next octet boundary outside a glued block.
         * serviceTx() tests its age before the write; one overwritten by a newer writeUrgent() first is not written.
         */
        bool RP2040::writeUrgent(char value)
        {
            if (!_running || !_txAsync) return false;
            const uint32_t irqState = save_and_disable_interrupts(); // also called from the RX interrupt
            _txUrgentAt = time_us_32();
            _txUrgent = (uint8_t)value;
            kickTx();
            restore_interrupts(irqState);
            return true;
        }

        unsigned int RP2040::droppedUrgent()
        {
            return _txUrgentDropped;
        }

        size_t RP2040::txFree()
        {
            if (!_txAsync) return SIZE_MAX;
            return (uint16_t)(_txTail - _txHead - 1) & kTxRingMask;
        }

        /**
         * @brief Drops queued octets but finishes a glued block already started: its rest would otherwise be read as
         * the data of the next request. A pending acknowledge stays, it belongs to a received frame.
         */
        void RP2040::txClear()
        {
            if (!_running || !_txAsync) return;
            const uint32_t irqState = save_and_disable_interrupts(); // also called from the RX interrupt (receivedReset)
            uint16_t keep = _txTail;
            if (_txGlued)
                while (keep != _txHead)
                {
                    const bool glued = (_txRing[keep] & kTxGlued) != 0;
                    keep = (keep + 1) & kTxRingMask;
                    if (!glued) break;
                }
            _txHead = keep;
            restore_interrupts(irqState);
        }

        /**
         * @brief Queues 1..4 octets for the TX interrupt; all but the last are glued to their successor.
         * Waits only while the ring is full, which a long frame can cause; the interrupt frees one entry per octet time.
         */
        bool RP2040::txPush(const char *data, size_t length)
        {
            while ((uint16_t)((uint16_t)(_txTail - _txHead - 1) & kTxRingMask) < length)
                if (!_running) return false;

            const uint32_t irqState = save_and_disable_interrupts(); // txClear() from the RX interrupt moves the head
            uint16_t head = _txHead;
            for (size_t i = 0; i < length; i++)
            {
                _txRing[head] = (uint8_t)data[i] | (i + 1 < length ? kTxGlued : 0);
                head = (head + 1) & kTxRingMask;
            }
            _txHead = head;
            kickTx();
            restore_interrupts(irqState);
            return true;
        }

        /**
         * @brief Unmasks the TX interrupt and fills an empty holding register: the interrupt follows only a transition to
         * empty (RP2040 DS 4.2.6.3), so an idle transmitter needs this first octet. Interrupts off.
         */
        void RP2040::kickTx()
        {
            hw_set_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
            serviceTx();
        }

        /**
         * @brief TX part of the UART interrupt: one octet into the empty holding register, an acknowledge first.
         */
        void RP2040::serviceTx()
        {
            uart_hw_t *const hw = uart_get_hw(_uart);
            if (hw->fr & UART_UARTFR_TXFF_BITS) return; // holding register still full: an RX interrupt, or a kick while sending

            const uint8_t urgent = _txUrgent;
            const uint16_t tail = _txTail;
            const bool queued = tail != _txHead;
            if (urgent && !(_txGlued && queued))
            {
                _txUrgent = 0;
                // Past the window the chip would apply it to the frame it receives then.
                if (time_us_32() - _txUrgentAt <= TPUART_ACK_WINDOW_US)
                {
                    hw->dr = urgent;
                    return;
                }
                _txUrgentDropped = _txUrgentDropped + 1;
            }
            if (queued)
            {
                const uint16_t entry = _txRing[tail];
                hw->dr = (uint8_t)entry;
                _txGlued = (entry & kTxGlued) != 0;
                _txTail = (tail + 1) & kTxRingMask;
                return;
            }
            hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS); // nothing to send: the empty holding register would fire again at once
        }

        int RP2040::read()
        {
            // if (!_running) return false; // not necessary, because available() is already checking this
            if (!available()) return -1;
            if (!_dma)
            {
#ifdef TPUART_BUSMON_INTEGRITY
                const uint32_t dr = uart_get_hw(_uart)->dr; // DATA + status in one access
                _lastByteErrored = (dr & 0x0300) != 0; // FE|PE -- same set the ESP32 path reports
                _lastByteOverrun = (dr & 0x0C00) != 0; // BE|OE -- host link, not the bus
                _lastByteStatus = (unsigned char)((dr >> 8) & 0x0F);
                return (int)(dr & 0xFF);
#else
                return uart_getc(_uart);
#endif
            }

            // when transfer count and read counter too far apart, then reset the read counter to current transfer count - 1
            if (overflow()) _dmaReaderCount = dmaTransferCount() - 1;

            const TpuartDmaWord word = _dmaBuffer[_dmaReaderCount++ % TPUART_RP2040_BUFFER_SIZE];
#ifdef TPUART_BUSMON_INTEGRITY
            // DR[11:8] = OE|BE|PE|FE: PE = NCN bus-side bit error (DS p.27), OE|BE = host-link loss (counted apart), FE either.
            _lastByteErrored = (word & 0x0300) != 0;
            _lastByteOverrun = (word & 0x0C00) != 0;
            _lastByteStatus = (unsigned char)((word >> 8) & 0x0F);
#endif
            int ret = (int)(word & 0xFF);

            // is the dma channel expired and last char, then restart the dma channel.
            if (!available() && !dma_channel_is_busy(_dmaChannel))
            {
                _dmaReaderCount = 0;
                dma_channel_abort(_dmaChannel);
                dma_channel_set_write_addr(_dmaChannel, _dmaBuffer, true);
            }

            return ret;
        }

        bool RP2040::overflow()
        {
            if (!_running) return false;
            if (!_dma) return (uart_get_hw(_uart)->fr & 0x00000040);

            return dmaTransferCount() - dmaReaderCount() > TPUART_RP2040_BUFFER_SIZE;
        };

        bool RP2040::dmaActive()
        {
            return _dma;
        }

        bool RP2040::lastByteErrored()
        {
            return _lastByteErrored;
        }

        bool RP2040::lastByteOverrun()
        {
            return _lastByteOverrun;
        }

        unsigned char RP2040::lastByteStatus()
        {
            return _lastByteStatus;
        }

        bool RP2040::hasCallback()
        {
            return _irq;
        }

        void RP2040::registerCallback(std::function<bool()> callback)
        {
            _callback = callback;
        }

        void RP2040::flush()
        {
            if (!_running) return;

            while (available())
                read();
        }

        void RP2040::interrupt()
        {

            // if a callback is registered
            if (_callback)
            {
                // Only with RX data: a TX interrupt usually has none.
                if (available()) _callback();
                // call a second time, as it is not always guaranteed that the callback is called for each character, especially when using DMA
                if (available()) _callback();
            }

            // reset the interrupt, except when DMA is used as it is automatically reset
            if (!_dma) uart_get_hw(_uart)->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS; // clear interrupt

            // TX part of the shared UART interrupt: refills the holding register once it is empty.
            if (_txAsync) serviceTx();
        }
    } // namespace Interface
} // namespace TPUart

#endif