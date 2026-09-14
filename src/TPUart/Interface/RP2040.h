#pragma once
#ifdef ARDUINO_ARCH_RP2040
#include <Arduino.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/sync.h>

#include "TPUart/Interface/Abstract.h"
#include "TPUart/Transmitter.h" // nur für TPUART_TX_INTERFACE_BUFFER - die Anforderung gehört dem Transmitter

namespace TPUart
{
namespace Interface
{

#ifndef TPUART_RP2040_RX_BUFFER_EXP
#define TPUART_RP2040_RX_BUFFER_EXP 8 // 2**BufferExp - per Build-Flag überschreibbar, z.B. -D TPUART_RP2040_RX_BUFFER_EXP=7 für 128 Byte
#endif
constexpr unsigned long TPUART_RP2040_RX_BUFFER_SIZE = (1u << TPUART_RP2040_RX_BUFFER_EXP);

// Bytes je DMA-Transfer, danach Neustart. Bewusst klein (2**20, bei voller Last ~24 Minuten), damit der
// Neustartpfad im Feld tatsächlich läuft; mit -D TPUART_RP2040_TRANSFER_COUNT_EXP=12 in Sekunden provozierbar.
// Im Neustartfenster (ein Tick) kommt höchstens ein Byte, das die UART hält. Höchstens 2**30, Vielfaches
// der Ringgröße.
#ifndef TPUART_RP2040_TRANSFER_COUNT_EXP
#define TPUART_RP2040_TRANSFER_COUNT_EXP 20
#endif
static_assert(TPUART_RP2040_TRANSFER_COUNT_EXP >= TPUART_RP2040_RX_BUFFER_EXP, "TPUART_RP2040_TRANSFER_COUNT_EXP muss mindestens so gross wie der Ring sein");
static_assert(TPUART_RP2040_TRANSFER_COUNT_EXP <= 30, "TPUART_RP2040_TRANSFER_COUNT_EXP zu gross - UINT32_MAX bringt die DMA-Hardware durcheinander");
constexpr uint32_t TPUART_RP2040_TRANSFER_COUNT = (1u << TPUART_RP2040_TRANSFER_COUNT_EXP) & ~(TPUART_RP2040_RX_BUFFER_SIZE - 1);

// RP2040-UART: Empfang per DMA-Ring, Senden über einen Software-Ring (TPUART_TX_INTERFACE_BUFFER), den der
// TX-Interrupt leert. Die UART-FIFO ist aus - sie gilt für RX und TX gemeinsam; eine FIFO würde die
// Quittungsfrist verletzen.
//
// Der TX-Ring hat zwei Kontexte (Tick füllt, ISR leert), daher _txSection - ein critical_section_t, weil
// der Tick auf dem anderen Kern laufen kann und PRIMASK nur je Kern wirkt.
class RP2040 : public Abstract
{
  private:
    pin_size_t _rx, _tx;
    uart_inst_t *_uart;
    gpio_function_t _rxRestore, _txRestore;

    // Die TX-ISR prüft es zuerst - nach end() darf ein gependeter Eintritt keine Register mehr anfassen.
    volatile bool _running = false;

    int _dmaChannel = -1;
    dma_channel_config _dmaConfig;
    volatile uint8_t __attribute__((aligned(TPUART_RP2040_RX_BUFFER_SIZE))) _dmaBuffer[TPUART_RP2040_RX_BUFFER_SIZE] = {};
    volatile uint32_t _dmaReaderCount = 0;
    volatile uint32_t _dmaTransferBase = 0;
    volatile bool _pendingRestart = false;

    // Ringüberlauf, gesetzt in read(), gelesen und gelöscht von overflow().
    bool _ringOverflow = false;

    uint8_t _txBuffer[TPUART_TX_INTERFACE_BUFFER] = {};
    volatile uint32_t _txHead = 0; // nächste Schreibposition - geschrieben NUR von write()
    volatile uint32_t _txTail = 0; // nächste zu sendende Position - geschrieben NUR von pumpTx()

    // Schützt _txBuffer, _txTail, Datenregister und Interruptmaske.
    critical_section_t _txSection;

    // false, wenn die UART-Leitung fremd belegt ist - dann leert nur der Tick den Ring.
    bool _txIrqInstalled = false;

    // 0 oder 1, je nach UART.
    uint uartIndex() const { return _uart == uart1 ? 1 : 0; }

    uint32_t dmaTransferCount();
    void checkRestart();
    void restartDma();

    // Einzige Stelle, die das Datenregister beschreibt, _txTail bewegt und TXIM setzt. Vorbedingung: _txSection gehalten.
    void pumpTx();

    // Aus dem DMA-Completion-IRQ: setzt nur ein Flag.
    void onDmaComplete();

  public:
    // Aus dem UART-Interrupt; öffentlich für den statischen Handler.
    void onTxInterrupt();

  private:

  public:
    // irq und dma werden ignoriert und fallen weg - beides wird immer benutzt. Nur noch für vorhandenen
    // Aufrufercode.
    RP2040(pin_size_t rx, pin_size_t tx, uart_inst_t *uart, bool irq = false, bool dma = true);
    ~RP2040();

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
