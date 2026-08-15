#include "TPUart/Interface/RP2040.h"
#ifdef ARDUINO_ARCH_RP2040
#include <functional>

// DMA_IRQ_0 wird von allen Channels gemeinsam genutzt - ein einziger Handler prüft, welcher Channel
// ausgelöst hat, und dispatcht an die jeweils registrierte RP2040-Instanz. Der Handler setzt bewusst
// nur ein Flag - die eigentliche Verarbeitung passiert nicht im ISR-Kontext.
// RP2040 hat nur 2 Hardware-UARTs, also auch maximal 2 RP2040-Instanzen gleichzeitig - die Registry
// muss nicht alle 12 möglichen DMA-Channel-Nummern abdecken, nur 2 Slots (Channel-Nummer selbst beliebig,
// da von dma_claim_unused_channel() vergeben).
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

// TX-INTERRUPT, und die Unterschiede zur DMA-Registry oben sind keine Willkür:
//
//   KEINE SUCHE. DMA_IRQ_0 ist EINE Leitung für alle zwölf Kanäle, der Handler muss also erst
//   herausfinden, wer ausgelöst hat. Die UART hat je Instanz ihre eigene Leitung (UART0_IRQ/UART1_IRQ) -
//   wer gemeint ist, steht damit schon fest, bevor der Handler läuft.
//
//   KEIN std::function. Der DMA-Handler feuert alle paar Tage (wenn der Transferzähler abläuft), dieser
//   bei 38400 Baud 3490-mal je Sekunde. Ein Indirektionsobjekt mit Heap-Möglichkeit hat in diesem Pfad
//   nichts zu suchen.
//
// Was NICHT unterschiedlich sein sollte, ist die Vorsicht beim Beanspruchen der Leitung: hier wird
// geprüft, ob sie schon jemandem gehört, beim DMA nicht (siehe Konstruktor).
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

// Die Modulo-Rechnung im Ring steht im ISR-Pfad. Bei einer Zweierpotenz ist das eine Maskierung, sonst
// eine Division - auf dem Cortex-M0+ ein Bibliotheksaufruf mitten im Interrupt. TPUART_TX_INTERFACE_BUFFER
// ergibt sich aus TPUART_CTRL_MAX_GROUP; wächst die je auf einen krummen Wert, soll der Build brechen und
// nicht die Zeitrechnung.
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
    // GETEILT UND NICHT EXKLUSIV: DMA_IRQ_0 ist EINE Leitung für alle zwölf Kanäle und wird deshalb
    // regelmäßig auch von anderen Bibliotheken benutzt. Hier stand irq_set_exclusive_handler(), und das
    // hätte bei jedem solchen Nachbarn eine Panic im Konstruktor ausgelöst - also vor setup() und ohne jede
    // Ausgabe. Der geteilte Handler koexistiert; unserer prüft ohnehin, ob einer SEINER Kanäle ausgelöst
    // hat, und geht sonst weiter.
    if (!__tpuartDmaIrqInstalled)
    {
        irq_add_shared_handler(DMA_IRQ_0, __tpuartDmaIrqHandler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(DMA_IRQ_0, true);
        __tpuartDmaIrqInstalled = true;
    }
    dma_channel_set_irq0_enabled(_dmaChannel, true);

    critical_section_init(&_txSection);

    // TX-Interrupt beanspruchen - im Konstruktor und nicht in begin(), aus demselben Grund wie beim DMA:
    // begin()/end() laufen bei jedem Baudratenkandidaten erneut, und die Vektortabelle gehört nicht in
    // einen Pfad, der mehrfach hintereinander durchlaufen wird.
    //
    // Auch hier geteilt, aus demselben Grund - und zusätzlich mit Prüfung: hat sich jemand die Leitung
    // EXKLUSIV genommen, panickt auch irq_add_shared_handler(). Der arduino-pico-Core tut genau das in
    // SerialUART::begin(), und uart0 ist dort Serial1. Statt zu panicken bleibt die Beschleunigung dann
    // einfach aus - der Ring wird wie früher allein vom Tick geleert (siehe availableForWrite).
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

    // War das die letzte Instanz, wird auch der geteilte Handler wieder abgemeldet - vorher blieb er für
    // immer stehen. Die LEITUNG bleibt dabei freigeschaltet: sie gehört uns nicht allein, und wer sie sonst
    // noch benutzt, braucht sie weiter.
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

        // Abschalten nur, wenn danach niemand mehr an dieser Leitung hängt - sonst nähme man einem
        // Nachbarn seine Interrupts weg.
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

    // Kanal vollständig neu konfigurieren, nicht nur Zähler und Zieladresse. Die Konfiguration stand früher
    // ausschließlich im Konstruktor - damit hing der Empfang daran, dass end() das CTRL-Register unangetastet
    // lässt. Genau das ist einmal schiefgegangen. begin() ist der Ort, an dem der Kanal betriebsbereit
    // gemacht wird, also gehört die Konfiguration auch hierher.
    dma_channel_set_read_addr(_dmaChannel, &uart_get_hw(_uart)->dr, false);
    dma_channel_set_config(_dmaChannel, &_dmaConfig, false);
    dma_channel_set_trans_count(_dmaChannel, TPUART_RP2040_TRANSFER_COUNT, false);
    _dmaReaderCount = 0;
    _dmaTransferBase = 0;
    _pendingRestart = false;
    _ringOverflow = false;
    _txHead = 0;
    _txTail = 0;

    // uart_init() nimmt den Block aus dem Reset, IMSC ist danach 0 - TXIM startet also von selbst
    // abgeschaltet. Das Löschen des Latches ist reine Vorsicht für den Fall, dass ein Aufrufer den Block
    // vorher selbst angefasst hat.
    hw_clear_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
    uart_get_hw(_uart)->icr = UART_UARTICR_TXIC_BITS;

    dma_channel_set_write_addr(_dmaChannel, _dmaBuffer, true);

    _running = true;
}

void RP2040::end()
{
    if (!_running) return;

    // REIHENFOLGE: erst die ISR stilllegen, dann den Block abschalten. _running zuerst, damit ein bereits
    // gependeter Eintritt umkehrt, ohne Register anzufassen; TXIM danach, damit keiner mehr entsteht. Beides
    // in der Sperre, weil sonst die ISR mitten in pumpTx() stehen könnte, während uart_deinit() läuft.
    critical_section_enter_blocking(&_txSection);
    _running = false;
    hw_clear_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
    uart_get_hw(_uart)->icr = UART_UARTICR_TXIC_BITS;
    critical_section_exit(&_txSection);

    // Was in Ring, Halte- und Schieberegister steht, geht dabei verloren. Gewartet wird ausdrücklich nicht
    // (Abstract.h: nichts blockiert), und für den Baudratenkandidaten ist das Verwerfen ohnehin richtig.

    // Abort plus ausdrückliches Quittieren des Completion-IRQ: ein Abort kann den IRQ-Status des Channels
    // stehen lassen, und bliebe er stehen, würde der Handler nach dem nächsten begin() _pendingRestart
    // spurios setzen und einen Neustart bei laufendem Zähler auslösen.
    //
    // Hier stand kurzzeitig dma_channel_cleanup(), was genau dafür gedacht ist - aber mehr anfasst als den
    // IRQ-Status (u.a. das CTRL-Register) und damit die Kanal-Konfiguration verwarf. Da end()/begin() bei
    // jedem Baudraten-Kandidaten läuft, war der Empfang ab dem zweiten Versuch tot und die
    // Verbindungsaufnahme kam nie zustande. begin() konfiguriert den Kanal inzwischen ohnehin komplett neu,
    // aber der schlankere Weg reicht hier und fasst nichts an, was er nicht muss.
    dma_channel_abort(_dmaChannel);
    dma_channel_acknowledge_irq0(_dmaChannel);
    uart_deinit(_uart);
}

// Absolut fortlaufende Anzahl empfangener Bytes - überlebt DMA-Neustarts durch _dmaTransferBase.
// Ein Überlauf von uint32_t ist unkritisch, da immer nur Differenzen zu _dmaReaderCount gebildet werden.
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
    // Der Lesezeiger wird bewusst NICHT zurückgesetzt: Basis-Offset und Schreibposition werden nahtlos
    // fortgeführt, damit bereits empfangene, aber noch ungelesene Bytes im Ring weiterhin gültig bleiben.
    //
    // Die Basis wird auf den AKTUELLEN Gesamtstand gesetzt, nicht um TPUART_RP2040_TRANSFER_COUNT erhöht.
    // Im Normalfall (Zähler auf 0 abgelaufen) ist beides identisch. Der Unterschied zählt, wenn der
    // Neustart bei noch laufendem Zähler ausgelöst wird - etwa durch einen nach einem Abort stehen
    // gebliebenen IRQ-Status: dann würde die Erhöhung den logischen Zählerstand vorspringen lassen,
    // während die physische Schreibposition bleibt. Ring-Index und Realität wären damit DAUERHAFT um
    // den Differenzbetrag verschoben und read() lieferte für immer Bytes von der falschen Stelle.
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

// Nichtblockierend: es wird nur nachgeschoben, was die UART sofort annimmt. VORBEDINGUNG: _txSection ist
// gehalten - hier wird _txTail bewegt, das Datenregister beschrieben und die Interruptmaske gesetzt.
//
// DAS SELBST-SCHREIBEN IST DER ANSTOSS, und deshalb steht die Schleife vor dem Setzen der Maske. Der
// TX-Interrupt des PL011 entsteht am ÜBERGANG auf "Halteregister leer", nicht aus dem Zustand: TXIM
// freizuschalten löst also nichts aus, wenn schon nichts drinsteht. Die Schleife umgeht das, statt es zu
// behandeln - ist das Halteregister voll, kommt der Übergang zwangsläufig, sobald das Byte nachrückt; ist
// es leer, erzeugt der eigene Schreibzugriff ihn. Ein Sonderweg "Ring leer, also direkt ins Register"
// braucht es damit nirgends, und den dürfte es auch nicht geben: er würde das Byte an allem vorbeischieben,
// was hier gerade noch aussteht.
void __time_critical_func(RP2040::pumpTx)()
{
    while (_txTail != _txHead && uart_is_writable(_uart))
        uart_get_hw(_uart)->dr = _txBuffer[_txTail++ % TPUART_TX_INTERFACE_BUFFER];

    // OHNE EIGENEN HANDLER WIRD TXIM NIE GESETZT. Gehört die Leitung jemand anderem (siehe
    // _txIrqInstalled), liefe unser Interrupt in dessen Handler, der ihn nicht quittiert - ein Sturm, der
    // das Gerät lahmlegt. Das Löschen bleibt trotzdem stehen: es ist in jedem Fall harmlos und richtig.
    if (_txTail == _txHead || !_txIrqInstalled)
        hw_clear_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
    else
        hw_set_bits(&uart_get_hw(_uart)->imsc, UART_UARTIMSC_TXIM_BITS);
}

// Der TX-Interrupt. Hält die Leitung zwischen zwei Ticks versorgt - ohne ihn überbrücken nur die 2 Bytes
// in der Hardware, und die reichen bei 38400 nicht (siehe Klassenkommentar).
void __time_critical_func(RP2040::onTxInterrupt)()
{
    // ALLES IN DER SPERRE, auch die _running-Prüfung. end() setzt das Flag und schaltet den Block ab -
    // beides ebenfalls in der Sperre, also kann dazwischen nichts passieren. Prüfte man davor, bliebe ein
    // Fenster: Flag gelesen, end() läuft durch, und der Registerzugriff hier träfe einen Block im Reset.
    // Heute käme das nicht vor, weil end() aus dem Hauptkontext desselben Kerns kommt - aber die Sperre
    // gibt es ja gerade, weil diese Zusage im Modus Loop1 nicht mehr für alles gilt.
    critical_section_enter_blocking(&_txSection);

    if (_running && (uart_get_hw(_uart)->mis & UART_UARTMIS_TXMIS_BITS))
    {
        // ZUERST quittieren, dann nachschieben. Andersherum löschte man den Übergang wieder, den die
        // eigenen frischen Schreibzugriffe gerade erzeugt haben, und die nächste Aufweckung bliebe aus. Nur
        // dieses eine Bit: beim RP2040 ist nur der KOMBINIERTE UART-Interrupt herausgeführt, ein pauschales
        // Löschen träfe auch fremde Quellen.
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

    // Mehr als eine Ringfüllung kann nicht mehr gelesen werden - alles darüber hinaus wurde bereits
    // überschrieben (das meldet overflow()). Gekappt, damit die Zahl der tatsächlich lesbaren Menge entspricht.
    uint32_t pending = dmaTransferCount() - _dmaReaderCount;
    return pending > TPUART_RP2040_RX_BUFFER_SIZE ? TPUART_RP2040_RX_BUFFER_SIZE : pending;
}

// Das Pumpen steht hier weiterhin drin, obwohl es seit dem TX-Interrupt meist nichts mehr zu tun gibt: es
// ist der Rückfallpfad. Konnte der Interrupt nicht beansprucht werden (siehe _txIrqInstalled), leert allein
// dieser Aufruf den Ring - dann verhält sich das Interface wie vor der Umstellung, ohne dass irgendwo eine
// Fallunterscheidung nötig wäre.
size_t RP2040::availableForWrite()
{
    if (!_running) return 0;

    critical_section_enter_blocking(&_txSection);
    pumpTx();
    uint32_t used = _txHead - _txTail;
    critical_section_exit(&_txSection);

    return TPUART_TX_INTERFACE_BUFFER - used;
}

// Legt das Byte nur in den Ring - kein Warten auf die Hardware. Der Ring erhält die Reihenfolge, sodass
// sich ein zwischendurch geschriebenes Acknowledge nicht zwischen zusammengehörige Bytes schieben kann.
bool RP2040::write(char value)
{
    if (!_running) return false;

    critical_section_enter_blocking(&_txSection);

    // Erst pumpen: was die Hardware jetzt schon annimmt, macht Platz für dieses Byte. Dann anhängen und
    // erneut pumpen - der zweite Aufruf bringt das eben angehängte Byte auf den Weg und setzt TXIM, falls
    // es dort warten muss. Beides in EINEM Abschnitt, damit Platzprüfung und Anhängen denselben Stand sehen.
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

    // Der Lesezeiger liegt mehr als eine Ringfüllung zurück - die DMA hat ungelesene Bytes überschrieben.
    // Zwei Dinge müssen hier passieren:
    //
    // 1. Der Verlust wird gemerkt. Er lässt sich NICHT nachträglich aus den Zählern ableiten, weil die
    //    Korrektur genau die Differenz beseitigt, an der man ihn erkennen würde - und overflow() wird erst
    //    NACH read() gefragt. Ohne dieses Latch bliebe ein Ringüberlauf komplett unsichtbar; das
    //    Hardware-OE greift nämlich nicht, da nicht die DMA hinterherhinkt, sondern unser Leser.
    // 2. Aufgesetzt wird am ÄLTESTEN noch gültigen Byte, nicht am neuesten. Gültig sind die letzten
    //    TPUART_RP2040_RX_BUFFER_SIZE Bytes; das +1 lässt den Slot aus, den die DMA als nächsten überschreibt.
    //    Ein Sprung auf dmaTransferCount() - 1 würde stattdessen eine ganze Ringfüllung weiterhin gültiger
    //    Bytes wegwerfen und den Verlust so um die Puffergröße verstärken.
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

    // Hardware-Overrun der UART: die DMA konnte das Datenregister nicht rechtzeitig leeren. Deckt vor
    // allem den kurzen Moment ab, in dem sie für den Neustart angehalten ist.
    if (uart_get_hw(_uart)->rsr & UART_UARTRSR_OE_BITS)
    {
        uart_get_hw(_uart)->rsr = 0; // Schreibzugriff löscht die Fehlerflags
        result = true;
    }

    // Ringüberlauf: die DMA hat den LESER überholt. Ein anderer Fehler als oben - die Hardware kommt dabei
    // problemlos mit, nur wir holen zu langsam ab, also bleibt OE sauber. Erkannt wird das in read(), das
    // dabei zwangsläufig auch die Zählerdifferenz einkassiert; hier deshalb das Latch statt einer erneuten
    // Berechnung, die immer false wäre.
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
