#pragma once
#include <stdint.h>

#if defined(ARDUINO_ARCH_RP2040)
    #include <pico/time.h>

// PRIORITÄT DES TICK-INTERRUPTS auf dem RP2040. Numerisch KLEINER heißt höher; das SDK setzt beim
// Hochlauf alles auf PICO_DEFAULT_IRQ_PRIORITY (0x80), und auf dem Cortex-M0+ zählen nur die oberen
// 2 Bit - es gibt also genau vier Stufen: 0x00, 0x40, 0x80, 0xC0.
//
// WARUM ÜBERHAUPT: ein Interrupt verdrängt auf Cortex-M sehr wohl einen anderen (dafür steht das N in
// NVIC), aber NUR bei echt höherer Priorität - GLEICHE Priorität verdrängt nicht. Solange wir auf 0x80
// mitschwimmen, wartet der Tick auf jeden anderen Handler, der gerade läuft, und das sind alle.
//
// WARUM 0x40 UND NICHT 0x00: im gesamten SDK-Quellbaum gibt es genau einen expliziten
// irq_set_priority()-Aufruf, und der setzt die Hintergrundarbeit des Netzwerkstacks nach UNTEN
// (async_context_threadsafe_background auf PICO_LOWEST_IRQ_PRIORITY). Über 0x80 sitzt damit niemand,
// 0x40 genügt also. 0x00 nähme nur USB und DMA die Luft, ohne etwas hinzuzugewinnen.
//
// WAS ES NICHT LÖST: PRIMASK. noInterrupts(), save_and_disable_interrupts() und
// critical_section_enter_blocking() sperren unabhängig von der Priorität. Wer eine solche Klammer lange
// hält, hält auch uns an - dagegen hilft nur ein Tick auf dem anderen Kern, denn PRIMASK ist pro Kern.
// Flash-Schreibvorgänge bleiben in jedem Fall ein Loch, weil sie zusätzlich den zweiten Kern parken.
    #ifndef TPUART_RP2040_TICK_IRQ_PRIORITY
        #define TPUART_RP2040_TICK_IRQ_PRIORITY 0x40
    #endif
#elif defined(ARDUINO_ARCH_ESP32)
    #include <esp_timer.h>
#endif

namespace TPUart
{

class DataLinkLayer;

// Der Takt, in dem tick() laufen soll. 500µs sind für ein normales Gerät reichlich: der Bus liefert
// höchstens alle 1,354ms ein Byte (9600 Baud, 13 Bitzeiten je TP1-Zeichen), empfangsseitig ist also
// Luft nach oben.
//
// WER VIEL SENDET, SETZT IHN HERUNTER - aber nur noch auf manchen Interfaces, und das ist gemessen, nicht
// geschätzt. Die Sendehardware fasst wenig: der RP2040-UART läuft ohne FIFO (die DMA soll jedes
// Empfangsbyte sofort bekommen) und hält deshalb nur zwei Bytes, Halte- plus Schieberegister, bei 38400
// Baud also 2 x 286µs Vorrat. Wird das ausschließlich aus dem Tick nachgelegt, steht die Leitung zwischen
// den Ticks zeitweise still: bei 500µs Takt kommen so 3 Bytes je 1000µs statt 3,5 durch, rund 14% Verlust.
// Gemessen an einem Router mit Telegrammen in Maximalgröße: 436 Hostbytes brauchten 143ms statt der 124ms,
// die die Leitung hergibt.
//
// DAS RP2040-INTERFACE IST DAVON SEIT DEM TX-INTERRUPT AUSGENOMMEN. Dort schiebt die Hardware sich selbst
// nach, sobald ein Platz frei wird (siehe RP2040.h), und der Durchsatz hängt nicht mehr am Takt.
//
// DIE REGEL GILT NUR FÜR ArduinoSerial, und der Grund ist nicht "nur der Tick füllt nach", sondern WIE
// VIEL er pro Aufruf hineinbekommt. Der RP2040-UART ohne FIFO hält zwei Bytes, und das Halteregister wird
// erst frei, wenn das Schieberegister das vorige übernommen hat - dort geht also nur EIN Byte je Tick
// hinein, egal was der Transmitter anbietet. Deshalb muss der Takt unter einer Zeichenzeit der
// HOSTLEITUNG liegen: 286µs bei 38400 Baud, 573µs bei 19200 - für einen Router dort also
// -D TPUART_TICK_INTERVAL_US=250, zum Preis von rund 1% Rechenzeit.
//
// DER ESP32 BRAUCHT DAS NICHT. Sein Treiber hat einen 512-Byte-Ring und nimmt beide Bytes eines Oktetts im
// selben Tick (Transmitter::process() verlangt 2 bzw. 3 Bytes und schreibt sie zusammen). Ein Tick liefert
// damit ein ganzes Oktett: bei 19200 belegt das die Leitung 1146µs, bei 38400 noch 572µs - der 500µs-Takt
// ist in beiden Fällen schneller als die Leitung, die Leitung bleibt der Engpass. Genau so soll es sein.
//
// Empfangsseitig ändert der Takt ohnehin nichts, dort sind 500µs auf jedem Interface reichlich.
#ifndef TPUART_TICK_INTERVAL_US
    #define TPUART_TICK_INTERVAL_US 500
#endif

// AB HIER IST DER ANTRIEB GRUNDSÄTZLICH ZU LANGSAM, und die Zahl kommt aus dem Bus statt aus der
// Konfiguration: ein Tick bewegt ein Byte je Richtung, ein TP1-Zeichen dauert 1,354ms (9600 Baud,
// 13 Bitzeiten). Liegt der mittlere Tickabstand darüber, holt die Schicht weniger Bytes ab als der Bus im
// Vollausbau liefert - dann füllen sich die Puffer, bis etwas überläuft, und keine Puffergröße hilft
// dagegen. Das ist deshalb keine Empfehlung, sondern eine Untergrenze.
//
// DataLinkLayer::checkTickRate() misst dagegen die ERREICHTE Rate und meldet, wenn sie darüber liegt. Der
// Wert ist bewusst nicht der eingestellte Takt: dass jemand 500µs wollte und 2200µs bekommt, ist die eine
// Frage - ob das Ergebnis überhaupt für den Bus reicht, die andere und wichtigere.
#ifndef TPUART_TICK_SLOW_US
    #define TPUART_TICK_SLOW_US 1354
#endif

// Fenster, über das gemittelt wird. Lang genug, dass eine einzelne lange Runde im Hauptloop nichts
// auslöst - der Ausreißer steht in getTickGapMaxUs(), hier geht es um den Dauerzustand.
#ifndef TPUART_TICK_RATE_WINDOW_MS
    #define TPUART_TICK_RATE_WINDOW_MS 2000
#endif

// EIGENER ANTRIEB FÜR tick(), von der Library selbst aufgesetzt.
//
// Das Warum steht im Klassenkommentar des DataLinkLayer: tick() bearbeitet pro Aufruf genau ein Byte je
// Richtung. Aus dem Hauptloop gerufen hängt der Durchsatz damit an dessen Frequenz, und die
// Quittungsfrist von 2,8ms hängt daran mit - eine einzelne langsame Runde in einem fremden Modul kostet
// die Quittung. Mit eigenem Takt ist beides von der Anwendung entkoppelt.
//
// DIE BEIDEN PLATTFORMEN LÖSEN DAS UNTERSCHIEDLICH, und der Unterschied ist keine Geschmacksfrage:
//
//   RP2040 - Hardware-Timer aus dem Default-Alarm-Pool, tick() läuft im INTERRUPT. Erlaubt ist das nur,
//            weil beide dort verwendbaren Interfaces ISR-tauglich schreiben: RP2040 über seinen
//            Software-TX-Ring, ArduinoSerial über uart_putc_raw().
//
//   ESP32  - esp_timer, tick() läuft im TASK-Kontext des Timer-Tasks. Ein echter Interrupt scheidet aus:
//            uart_write_bytes() im ESP32-Interface nimmt einen Mutex, den ein ISR nicht nehmen darf.
//            Bewusst NICHT als eigener Task mit delayMicroseconds(): der bräuchte einen ganzen Kern für
//            sich, und auf Kern 0 sitzen im echten Gerät WLAN und Bluetooth.
//
// Auf jeder anderen Plattform meldet supported() false und start() schlägt fehl - der Aufrufer bleibt
// dann beim Antrieb aus dem Hauptloop (DataLinkLayer::process()).
//
// NUR EIN ANTRIEB GLEICHZEITIG. Läuft dieser hier, darf tick() von nirgendwo sonst kommen: zwei
// Kontexte in derselben State-Machine sind kein Nachteil, sondern ein Defekt. process() prüft das
// selbst; wer tick() direkt ruft, muss den Antrieb vorher über setTickInterval(0) abbestellen.
class TickDriver
{
  private:
    DataLinkLayer *_dll = nullptr;

    // Vom Hauptkontext geschrieben, vom Timer gelesen (RP2040 zusätzlich aus dem Interrupt).
    volatile bool _running = false;

#if defined(ARDUINO_ARCH_RP2040)
    repeating_timer_t _timer;

    // EIGENER ALARM-POOL, statt des Default-Pools. Zwei Gründe, und beide zählen: die Callbacks EINES
    // Pools laufen aus einem gemeinsamen IRQ-Handler und damit serialisiert - wer sonst noch drinhängt,
    // verzögert uns um seine Laufzeit. Und nur ein eigener Pool hat einen eigenen Hardware-Alarm, dessen
    // Priorität sich anheben lässt (siehe TPUART_RP2040_TICK_IRQ_PRIORITY).
    //
    // Einmal angelegt und behalten: start() kann nach einem end()/begin() erneut laufen, und je Durchlauf
    // einen Pool zu erzeugen wäre ein Leck - alarm_pool_create() allokiert.
    alarm_pool_t *_pool = nullptr;

    // Ein einziger Timer je Pool, mehr wird hier nie eingehängt.
    static constexpr uint32_t POOL_MAX_TIMERS = 1;

    void claimOwnPool();
    static bool onTimer(repeating_timer_t *timer);
#elif defined(ARDUINO_ARCH_ESP32)
    esp_timer_handle_t _timer = nullptr;
    static void onTimer(void *argument);
#endif

  public:
    // Ob die Plattform überhaupt einen eigenen Antrieb hergibt. Compile-time-Eigenschaft, trotzdem als
    // Funktion: so muss der Aufrufer die #ifdef-Kette nicht nachbauen.
    static bool supported();

    // Startet den Takt. Läuft er schon, passiert nichts und die Rückgabe ist true.
    bool start(DataLinkLayer &dll, uint32_t intervalUs);

    // Hält den Takt an und kehrt erst zurück, wenn garantiert kein tick() mehr laufen kann. Muss VOR dem
    // Schließen des Interfaces geschehen.
    void stop();

    bool running() const;
};

} // namespace TPUart
