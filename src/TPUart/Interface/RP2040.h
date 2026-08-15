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

// Wie viele Bytes die DMA am Stück überträgt, bevor ihr Zähler abläuft und sie neu angeworfen werden muss
// (onDmaComplete -> checkRestart -> restartDma).
//
// HIER STAND EINMAL DER GRÖSSTMÖGLICHE WERT, und das war schlecht testbar: 2**31 Bytes sind bei voller
// Buslast über einen Monat Dauerbetrieb. Ein Pfad, der einmal im Monat läuft, ist ein Pfad, der nie
// ausprobiert wird - und dieser hier hat es in sich, weil Schreibposition und Zählerstand über den
// Neustart hinweg zusammenpassen müssen. 2**20 sind bei voller Buslast rund 24 Minuten, im normalen
// Verkehr einige Stunden: oft genug, dass er im Feld tatsächlich läuft, und mit
// -D TPUART_RP2040_TRANSFER_COUNT_EXP=12 in wenigen Sekunden zu provozieren.
//
// DASS DAS GEFAHRLOS GEHT, hängt an der Länge des Neustartfensters: die DMA steht zwischen dem Ablauf des
// Zählers und dem nächsten checkRestart(), also höchstens einen Tick (500µs). In dieser Zeit kommt vom Bus
// höchstens EIN Byte (ein TP1-Zeichen dauert 1354µs), und genau eines fasst das Halteregister der UART.
// Verloren geht also nichts, solange der Tick läuft; ein Überlauf bräuchte zwei Bytes im Fenster.
//
// Zwei Randbedingungen bleiben: UINT32_MAX führt zu Fehlverhalten der DMA-Hardware (deshalb die Halbierung
// als Obergrenze), und der Wert muss ein Vielfaches der Puffergröße sein, damit die Schreibposition im Ring
// nach dem Neustart deterministisch wieder bei Index 0 steht.
#ifndef TPUART_RP2040_TRANSFER_COUNT_EXP
#define TPUART_RP2040_TRANSFER_COUNT_EXP 20
#endif
static_assert(TPUART_RP2040_TRANSFER_COUNT_EXP >= TPUART_RP2040_RX_BUFFER_EXP, "TPUART_RP2040_TRANSFER_COUNT_EXP muss mindestens so gross wie der Ring sein");
static_assert(TPUART_RP2040_TRANSFER_COUNT_EXP <= 30, "TPUART_RP2040_TRANSFER_COUNT_EXP zu gross - UINT32_MAX bringt die DMA-Hardware durcheinander");
constexpr uint32_t TPUART_RP2040_TRANSFER_COUNT = (1u << TPUART_RP2040_TRANSFER_COUNT_EXP) & ~(TPUART_RP2040_RX_BUFFER_SIZE - 1);

// Software-Sendepuffer (_txBuffer unten, Größe und Begründung bei TPUART_TX_INTERFACE_BUFFER - die
// gelten für jedes Interface gleich). Nötig, weil uart_set_fifo_enabled() beim PL011 RX- und TX-FIFO
// GEMEINSAM schaltet und die FIFO hier aus ist: damit ist das TX-Halteregister nur 1 Byte tief, und mehrere
// zusammengehörige Bytes ließen sich nicht am Stück absetzen. Der Ring erhält die Reihenfolge und liefert
// eine echte Restkapazität für availableForWrite() - beides braucht die Schicht darüber unabhängig von der
// FIFO. Nebenbei beseitigt er das früher hier stehende, blockierende uart_tx_wait_blocking().
//
// GELEERT WIRD DER RING VOM TX-INTERRUPT, und das ist der Grund, warum es ihn gibt. Vorher lief pumpTx()
// (damals drainTx()) nur aus write() und availableForWrite(), also ausschließlich an einem Tick. Zwischen
// zwei Ticks hielten allein die 2 Bytes in der Hardware die Leitung am Laufen - und in die passt pro Tick
// meist nur EINES, weil das Halteregister erst frei wird, wenn das Schieberegister das vorige übernommen
// hat. Bei 38400 (286µs je Byte) und 500µs Takt ergibt das 3 Bytes je 1000µs statt 3,5, also rund 14%
// Leerlauf: gemessen 143ms gegen 123ms für dieselben 436 Bytes, verglichen mit einem Antrieb, der alle
// ~12µs pollt (SESSION.md 56). Mit dem Interrupt schiebt die Hardware sich selbst nach, sobald ein Platz
// frei wird, und der Durchsatz hängt nicht mehr am Tick-Intervall.
//
// DAS MACHT KEINEN PUFFER TIEFER. Der Ring bleibt TPUART_TX_INTERFACE_BUFFER, die Hardware 2 - die Rechnung
// zur Quittungsfrist (siehe Transmitter.h) ist unberührt. Wer hier später "wenn schon Interrupt, dann auch
// FIFO an" schlussfolgert, hebt genau diese Zusage auf.
//
// NEBENLÄUFIGKEIT: der Ring hat jetzt ZWEI Kontexte - der Erzeuger (Tick, ggf. auf Kern 1) füllt, die ISR
// (Kern 0) leert. Deshalb _txSection. Ein lockfreier Handshake über ein "ISR ist scharf"-Flag reicht hier
// nicht: beide Seiten müssen uart_is_writable() PRÜFEN und abhängig davon schreiben, und dafür hat der
// Cortex-M0+ kein CAS. Das unterscheidet den Fall von den drei SPSC-Warteschlangen der Library, wo jede
// Seite genau einen Index schreibt und keine eine Entscheidung der anderen braucht. Und es muss ein
// critical_section_t sein, kein save_and_disable_interrupts(): im Modus Loop1 läuft der Erzeuger auf dem
// anderen Kern, und PRIMASK wirkt nur auf den eigenen.
//
// Zur FIFO selbst: hier stand einmal, die RX-FIFO müsse aus bleiben, weil der DMA-Request sonst erst ab
// dem IFLS-Schwellwert auslöse und die letzten Frame-Bytes liegen blieben. Belegt ist das nicht - das
// Datenblatt beschreibt zwar beide Signale (Kap. 4.2.5, S. 424: uartrxdmasreq ab einem Zeichen,
// uartrxdmabreq ab der Wasserstandsmarke), sagt aber NICHT, welches als DREQ_UARTx_RX herausgeführt ist;
// im SDK steht dazu ebenfalls nichts. FIFO-Betrieb mit DMA ist ausdrücklich vorgesehen, ein Abschaltgebot
// gibt es nirgends. Umgestellt wird trotzdem nichts, solange das nicht an echter Hardware gegengeprüft ist
// - die FIFO-Abschaltung stammt aus einer Beobachtung am laufenden Bus. Der TX-Interrupt beantwortet diese
// Frage NICHT, er macht sie nur unwichtiger.

class RP2040 : public Abstract
{
  private:
    pin_size_t _rx, _tx;
    uart_inst_t *_uart;
    gpio_function_t _rxRestore, _txRestore;

    // volatile, weil die TX-ISR es als Erstes prüft: nach end() kann ein bereits gependeter Eintritt noch
    // kommen, und der darf keine Register eines deinitialisierten Blocks mehr anfassen.
    volatile bool _running = false;

    int _dmaChannel = -1;
    dma_channel_config _dmaConfig;
    volatile uint8_t __attribute__((aligned(TPUART_RP2040_RX_BUFFER_SIZE))) _dmaBuffer[TPUART_RP2040_RX_BUFFER_SIZE] = {};
    volatile uint32_t _dmaReaderCount = 0;
    volatile uint32_t _dmaTransferBase = 0;
    volatile bool _pendingRestart = false;

    // Gesetzt in read(), wenn die DMA den Leser überholt hat, ausgelesen und gelöscht von overflow().
    // Muss gelatcht werden: read() beseitigt bei der Korrektur genau die Zählerdifferenz, an der sich der
    // Überlauf erkennen ließe, und overflow() wird erst danach gefragt.
    bool _ringOverflow = false;

    uint8_t _txBuffer[TPUART_TX_INTERFACE_BUFFER] = {};
    volatile uint32_t _txHead = 0; // nächste Schreibposition - geschrieben NUR von write()
    volatile uint32_t _txTail = 0; // nächste zu sendende Position - geschrieben NUR von pumpTx()

    // Schützt _txBuffer, _txTail und den Zugriff auf Datenregister und Interruptmaske. Kein
    // save_and_disable_interrupts(): der Erzeuger kann auf dem anderen Kern laufen (siehe Klassenkommentar).
    critical_section_t _txSection;

    // Gesetzt, wenn der TX-Interrupt beansprucht werden konnte. Ist die Leitung fremdbelegt (der
    // arduino-pico-Core installiert in SerialUART::begin() einen exklusiven Handler auf dieselbe UART),
    // bleibt es false - dann leert nur noch der Tick den Ring, also genau das Verhalten von vorher.
    bool _txIrqInstalled = false;

    // 0 oder 1, je nach UART - die Zuordnung zu Instanzzeiger, Handler und IRQ-Nummer hängt daran. Als
    // Methode, damit die Ableitung nicht an zwei Stellen steht und auseinanderlaufen kann.
    uint uartIndex() const { return _uart == uart1 ? 1 : 0; }

    uint32_t dmaTransferCount();
    void checkRestart();
    void restartDma();

    // Schiebt so viele gepufferte Bytes ins Halteregister, wie die UART gerade annimmt, und setzt danach
    // die Interruptmaske passend. DIE EINZIGE STELLE, die das Datenregister beschreibt, _txTail bewegt und
    // TXIM anfasst. Vorbedingung: _txSection ist gehalten.
    void pumpTx();

    // Wird von der DMA-Completion-IRQ aufgerufen, wenn TPUART_RP2040_TRANSFER_COUNT erschöpft ist (Vorgabe:
    // rund 24 Minuten bei voller Buslast). Setzt nur ein Flag - keine Verarbeitung im ISR-Kontext.
    void onDmaComplete();

  public:
    // Aus dem UART-Interrupt gerufen. Öffentlich, weil der statische Handler daneben ihn erreichen muss -
    // eine friend-Deklaration für zwei Zeilen wäre mehr Aufwand als Nutzen.
    void onTxInterrupt();

  private:

  public:
    // DIE BEIDEN HINTEREN PARAMETER WERDEN IGNORIERT UND FALLEN IN DER FINALEN FASSUNG WEG. Sie stammen aus
    // der Zeit, als hier zwischen IRQ- und DMA-Betrieb zu wählen war, und beide Schalter haben heute keinen
    // sinnvollen Gegenwert mehr:
    //
    //   irq - wird IMMER gebraucht, inzwischen sogar zweifach: einmal, um die DMA nach Ablauf des
    //         Transferzählers wieder anzuwerfen, und einmal, um den Sendepuffer nachzufüllen.
    //   dma - soll IMMER benutzt werden. Ohne sie fiele der Empfang auf ein 1 Byte tiefes Halteregister
    //         zurück, und genau davor schützt sie (Flash-Schreibvorgänge, siehe Klassenkommentar).
    //
    // Sie stehen nur noch da, damit vorhandener Aufrufercode unverändert übersetzt.
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
