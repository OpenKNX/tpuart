#pragma once
#include <stdint.h>

#if defined(ARDUINO_ARCH_RP2040)
    #include <pico/time.h>

// Priorität des Timer-Interrupts auf dem RP2040 (kleiner = höher; Stufen 0x00/0x40/0x80/0xC0, Vorgabe 0x80).
// Gleiche Priorität verdrängt nicht, darum 0x40. PRIMASK-Sperren und Flash-Schreiben halten den Tick trotzdem an.
    #ifndef TPUART_RP2040_TIMER_IRQ_PRIORITY
        #define TPUART_RP2040_TIMER_IRQ_PRIORITY 0x40
    #endif
#elif defined(ARDUINO_ARCH_ESP32)
    #include <esp_timer.h>
#endif

namespace TPUart
{

class DataLinkLayer;

// Takt für tick(). Der Bus liefert höchstens alle 1,354ms ein Byte. Mit ArduinoSerial auf einem RP2040-UART
// ohne FIFO geht nur ein Byte je Tick hinaus - wer dort viel sendet, setzt den Takt unter die Zeichenzeit
// der Hostleitung (-D TPUART_TIMER_INTERVAL_US=250). RP2040- und ESP32-Interface brauchen das nicht.
#ifndef TPUART_TIMER_INTERVAL_US
    #define TPUART_TIMER_INTERVAL_US 500
#endif

// Untergrenze aus dem Bus: liegt der mittlere Tickabstand über einer Zeichenzeit (1,354ms), laufen die
// Puffer voll. checkTickRate() misst die erreichte Rate dagegen.
#ifndef TPUART_TICK_DEFERRED_US
    #define TPUART_TICK_DEFERRED_US 1354
#endif

// Fenster für die mittlere Taktrate - einzelne Verzögerungen zählt getTickDeferrals().
#ifndef TPUART_TICK_RATE_WINDOW_MS
    #define TPUART_TICK_RATE_WINDOW_MS 2000
#endif

// Wie viele DataLinkLayer der Timer treiben kann. Eine Instanz ohne Platz wird nicht getickt und meldet
// binnen zwei Sekunden "Tick stopped".
#ifndef TPUART_TIMER_MAX_CLIENTS
    #define TPUART_TIMER_MAX_CLIENTS 1
#endif

// Der Antrieb für tick(), ein Singleton für alle BCUs (der RP2040 hat nur vier Hardware-Alarme).
//   RP2040 - eigener Alarm-Pool, tick() im Interrupt
//   ESP32  - esp_timer, tick() im Task-Kontext (uart_write_bytes() nimmt einen Mutex)
// Ohne Timer-Plattform tickt nichts, bis jemand trigger() ruft - der Hauptloop tickt nie.
// Nur ein Antrieb je Instanz: wer selbst treibt, hält den Timer mit setInterval(0) an.
class Timer
{
  private:
    // Festes Array ohne Allokation; ein Platz ist atomar leer oder belegt, add() braucht keine Sperre.
    DataLinkLayer *volatile _clients[TPUART_TIMER_MAX_CLIENTS] = {};

    // Global für alle Instanzen.
    uint32_t _intervalUs = TPUART_TIMER_INTERVAL_US;

    // Vom Hauptkontext geschrieben, vom Timer gelesen (RP2040 zusätzlich aus dem Interrupt).
    volatile bool _running = false;

#if defined(ARDUINO_ARCH_RP2040)
    repeating_timer_t _timer;

    // Eigener Pool: eigene Priorität, keine fremden Callbacks davor. Einmal angelegt und behalten -
    // alarm_pool_create() allokiert.
    alarm_pool_t *_pool = nullptr;

    // Ein Timer je Pool.
    static constexpr uint32_t POOL_MAX_TIMERS = 1;

    void claimOwnPool();
    static bool onTimer(repeating_timer_t *timer);
#elif defined(ARDUINO_ARCH_ESP32)
    esp_timer_handle_t _timer = nullptr;
    static void onTimer(void *argument);
#endif

    // Ruft tick() für jeden eingetragenen DataLinkLayer, in Reihenfolge der Plätze.
    void fire();

    bool startTimer();
    void stopTimer();

    Timer() = default;

    // Alle Member müssen trivial zerstörbar bleiben: ~DataLinkLayer() eines globalen Objekts kann nach
    // diesem Objekt laufen und greift trotzdem auf instance().
    static Timer _instance;

  public:
    // Klassenmember statt funktionslokalem static - ohne Guard-Aufruf, aus dem Callback erreichbar.
    static Timer &instance();

    static bool supported();

    // Trägt ein und startet den Timer. false: keine Timer-Plattform, Intervall 0 oder kein Platz frei.
    bool add(DataLinkLayer &dll);

    // Trägt aus. Danach läuft garantiert kein tick() mehr für diese Instanz.
    void remove(DataLinkLayer &dll);

    // RP2040: add() und remove() - also begin(), end() und der Destruktor - müssen vom selben Kern kommen,
    // sonst gilt die Zusage von remove() nicht. Wird nicht geprüft.

    bool contains(const DataLinkLayer &dll) const;

    // Global für alle Instanzen. 0 schaltet den Timer ab.
    void setInterval(uint32_t intervalUs);
    uint32_t interval() const;

    bool running() const;
    uint8_t clients() const;

    // Von Hand treiben: ein tick() je eingetragener Instanz. Neben einem laufenden Timer ein Defekt. Auf dem
    // ESP32 nicht aus einem ISR.
    void trigger();
};

} // namespace TPUart
