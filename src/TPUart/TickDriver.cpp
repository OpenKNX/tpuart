#include "TPUart/TickDriver.h"

#include "TPUart/DataLinkLayer.h"

#if defined(ARDUINO_ARCH_RP2040)
    #include <hardware/irq.h>
    #include <hardware/timer.h>
#endif

namespace TPUart
{

#if defined(ARDUINO_ARCH_RP2040)

bool TickDriver::supported()
{
    return true;
}

// Läuft im Interrupt. Der Rückgabewert hält den Timer am Leben.
bool TickDriver::onTimer(repeating_timer_t *timer)
{
    TickDriver *self = (TickDriver *)timer->user_data;
    if (self != nullptr && self->_running) self->_dll->tick();

    return true;
}

// Besorgt einen eigenen Hardware-Alarm samt Pool und hebt dessen IRQ-Priorität an. Schlägt das fehl,
// bleibt _pool auf nullptr und start() nimmt den Default-Pool - dann verhält sich alles wie vorher.
//
// DIE REIHENFOLGE IST HIER NICHT VERHANDELBAR, denn beide bequemen SDK-Funktionen sind für eine Library
// unbrauchbar: alarm_pool_create() ruft intern ta_hardware_alarm_claim() und macht ein HARD ASSERT, wenn
// der Alarm schon vergeben ist; alarm_pool_create_with_unused_hardware_alarm() claimt mit required=true
// und assertet, wenn gar keiner mehr frei ist. Beides wäre ein Absturz des Geräts statt eines
// Fehlschlags, den wir melden könnten. hardware_alarm_claim_unused(false) ist der einzige Weg, der einen
// Misserfolg ZURÜCKGIBT - deshalb wird damit geprüft, sofort wieder freigegeben und dann erst erzeugt.
//
// Das Fenster zwischen unclaim und create ist ungefährlich: das hier läuft aus begin(), im Hauptkontext,
// und niemand sonst greift zu diesem Zeitpunkt nach Hardware-Alarmen.
void TickDriver::claimOwnPool()
{
    int alarmNum = hardware_alarm_claim_unused(false);
    if (alarmNum < 0) return; // alle vier Alarme vergeben - Rückfall auf den Default-Pool

    hardware_alarm_unclaim((uint)alarmNum);

    _pool = alarm_pool_create((uint)alarmNum, POOL_MAX_TIMERS);
    if (_pool == nullptr) return; // malloc fehlgeschlagen - ebenfalls Rückfall, der Alarm ist wieder frei

    // Über das Makro und nicht als TIMER_IRQ_0 + n: auf dem RP2350 heißt die erste Alarm-IRQ anders, und
    // arduino-pico baut beide Chips unter ARDUINO_ARCH_RP2040.
    irq_set_priority(TIMER_ALARM_IRQ_NUM(alarm_pool_get_default_timer(), (uint)alarmNum), TPUART_RP2040_TICK_IRQ_PRIORITY);
}

bool TickDriver::start(DataLinkLayer &dll, uint32_t intervalUs)
{
    if (_running) return true;
    if (intervalUs == 0) return false;

    _dll = &dll;

    if (_pool == nullptr) claimOwnPool();

    _running = true; // muss stehen, bevor der erste Interrupt kommen kann

    // NEGATIVES Intervall: der Abstand gilt zwischen den STARTS zweier Aufrufe, nicht zwischen dem Ende
    // des einen und dem Start des nächsten - nur so ist der Takt von der Laufzeit eines Ticks unabhängig.
    bool started = _pool != nullptr
                       ? alarm_pool_add_repeating_timer_us(_pool, -(int64_t)intervalUs, onTimer, this, &_timer)
                       : add_repeating_timer_us(-(int64_t)intervalUs, onTimer, this, &_timer);

    if (!started)
    {
        _running = false;
        return false;
    }

    return true;
}

void TickDriver::stop()
{
    if (!_running) return;

    // Erst abbestellen, dann das Flag löschen: cancel_repeating_timer() kehrt zurück, wenn kein Aufruf
    // mehr ansteht, ein bereits laufender läuft aber zu Ende. Das Flag davor zu löschen brächte nichts -
    // dieser eine Aufruf ist ohnehin schon durch die Prüfung hindurch.
    cancel_repeating_timer(&_timer);
    _running = false;
}

#elif defined(ARDUINO_ARCH_ESP32)

bool TickDriver::supported()
{
    return true;
}

// Läuft im Timer-TASK, nicht im Interrupt - siehe Header. Damit ist alles erlaubt, was auch aus dem
// Hauptloop erlaubt wäre, insbesondere der Mutex des UART-Treibers.
void TickDriver::onTimer(void *argument)
{
    TickDriver *self = (TickDriver *)argument;
    if (self != nullptr && self->_running) self->_dll->tick();
}

bool TickDriver::start(DataLinkLayer &dll, uint32_t intervalUs)
{
    if (_running) return true;
    if (intervalUs == 0) return false;

    _dll = &dll;

    // Alle Felder ausdrücklich gesetzt - ohne die hinteren warnt der Compiler bei -Wextra.
    esp_timer_create_args_t args = {
        .callback = onTimer,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tpuart_tick",
        .skip_unhandled_events = true, // lieber einen Takt auslassen als Aufrufe nachholen
    };

    if (esp_timer_create(&args, &_timer) != ESP_OK)
    {
        _timer = nullptr;
        return false;
    }

    _running = true; // muss stehen, bevor der erste Aufruf kommen kann

    if (esp_timer_start_periodic(_timer, intervalUs) != ESP_OK)
    {
        _running = false;
        esp_timer_delete(_timer);
        _timer = nullptr;
        return false;
    }

    return true;
}

void TickDriver::stop()
{
    if (!_running) return;

    // esp_timer_stop() wartet auf einen gerade laufenden Callback, esp_timer_delete() ebenfalls - danach
    // ist sicher keiner mehr unterwegs.
    esp_timer_stop(_timer);
    _running = false;

    esp_timer_delete(_timer);
    _timer = nullptr;
}

#else

bool TickDriver::supported()
{
    return false;
}

bool TickDriver::start(DataLinkLayer &dll, uint32_t intervalUs)
{
    (void)dll;
    (void)intervalUs;
    return false;
}

void TickDriver::stop() {}

#endif

bool TickDriver::running() const
{
    return _running;
}

} // namespace TPUart
