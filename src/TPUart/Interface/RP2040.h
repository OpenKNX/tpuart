#pragma once
#ifdef ARDUINO_ARCH_RP2040
#include <Arduino.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>

#include "TPUart/Interface/Abstract.h"
#include "TPUart/Config.h"
#include "TPUart/Types.h"

constexpr uint32_t TPUART_RP2040_TRANSFER_COUNT = UINT32_MAX >> 1; // transfer count per channel start, halved because UINT32_MAX did not work; read() restarts the channel once it has run out
// constexpr unsigned int TPUART_RP2040_TRANSFER_COUNT = 5000;
// constexpr unsigned int TPUART_RP2040_TRANSFER_COUNT = 256;
constexpr unsigned long TPUART_RP2040_BUFFER_EXP = TPUART_RP2040_DMA_BUFFER_EXP; // 2**BufferExp entries, default 2048
// The DMA RX ring must outlast the longest noInterrupts() stall (LittleFS flash erase) without wrapping and hold one
// maximum extended frame (263 octets incl. FCS); read() discards the octets of a wrapped ring.
constexpr unsigned long TPUART_RP2040_BUFFER_SIZE = (1u << TPUART_RP2040_BUFFER_EXP); // entries

// 16-bit entries carry the UART status bits DR[11:8] with the octet; DMA_SIZE_8 copies only DR[7:0]
// and drops the chip's error signal. The ring wraps on BYTES, so 16-bit needs one exponent more.
#ifdef TPUART_BUSMON_INTEGRITY
typedef uint16_t TpuartDmaWord;
    #define TPUART_RP2040_DMA_XFER DMA_SIZE_16
constexpr unsigned long TPUART_RP2040_RING_EXP = TPUART_RP2040_BUFFER_EXP + 1;
#else
typedef uint8_t TpuartDmaWord;
    #define TPUART_RP2040_DMA_XFER DMA_SIZE_8
constexpr unsigned long TPUART_RP2040_RING_EXP = TPUART_RP2040_BUFFER_EXP;
#endif
constexpr unsigned long TPUART_RP2040_RING_BYTES = (1u << TPUART_RP2040_RING_EXP);
// RING_SIZE takes 1..15 (RP2040 DS 2.5, CH0_CTRL_TRIG.RING_SIZE). With TPUART_BUSMON_INTEGRITY RING_EXP is
// DMA_BUFFER_EXP + 1, so the knob tops out at 14 there.
static_assert(TPUART_RP2040_RING_EXP >= 1 && TPUART_RP2040_RING_EXP <= 15, "TPUART_RP2040_DMA_BUFFER_EXP out of the DMA ring range");

namespace TPUart
{
    namespace Interface
    {
        class RP2040 : public Abstract
        {
          private:
            int _dmaChannel;
            dma_channel_config _dmaConfig;

            volatile TpuartDmaWord __attribute__((aligned(TPUART_RP2040_RING_BYTES))) _dmaBuffer[TPUART_RP2040_BUFFER_SIZE] = {};
            volatile uint _dmaReaderCount = 0;
            volatile uint _dmaRestartDiff = 0;
            bool _lastByteErrored = false; // FE|PE of the octet last handed out by read()
            bool _lastByteOverrun = false;  // BE|OE of that octet -- host link lost data, bus unaffected
            unsigned char _lastByteStatus = 0;    // the four DR[11:8] bits, kept apart for diagnosis

            pin_size_t _rx, _tx;
            gpio_function_t _rxRestore, _txRestore;
            uart_inst_t *_uart;
            bool _irq;
            bool _dma;

            std::function<bool(void)> _callback;

          public:
            RP2040(pin_size_t rx, pin_size_t tx, uart_inst_t *uart, bool irq = false, bool dma = false);
            ~RP2040();
            void begin(int baud);
            void end();
            uint dmaTransferCount();
            uint dmaReaderCount();
            bool available() override;
            bool availableForWrite() override;
            bool write(char value) override;
            int read() override;
            bool overflow() override;
            bool lastByteErrored() override;
            bool lastByteOverrun() override;
            unsigned char lastByteStatus() override;
            bool dmaActive(); // false if the ring buffer came back misaligned and DMA was refused
            bool hasCallback() override;
            void registerCallback(std::function<bool()> callback) override;
            void flush() override;
            void interrupt();
        };
    } // namespace Interface
} // namespace TPUart

#endif
