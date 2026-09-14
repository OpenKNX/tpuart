#include "TPUart/Timer.h"

#include "TPUart/DataLinkLayer.h"

#if defined(ARDUINO_ARCH_RP2040)
    #include <hardware/irq.h>
    #include <hardware/timer.h>
#endif

namespace TPUart
{

Timer Timer::_instance;

Timer &Timer::instance()
{
    return _instance;
}

#if defined(ARDUINO_ARCH_RP2040)

bool Timer::supported()
{
    return true;
}

// Läuft im Interrupt. Der Rückgabewert hält den Timer am Leben.
bool Timer::onTimer(repeating_timer_t *timer)
{
    Timer *self = (Timer *)timer->user_data;
    if (self != nullptr) self->fire();

    return true;
}

// Holt einen eigenen Hardware-Alarm samt Pool und hebt dessen Priorität. Fehlschlag: Default-Pool.
// Erst mit hardware_alarm_claim_unused(false) prüfen und wieder freigeben - alarm_pool_create() und
// ..._with_unused_hardware_alarm() asserten sonst hart.
void Timer::claimOwnPool()
{
    int alarmNum = hardware_alarm_claim_unused(false);
    if (alarmNum < 0) return; // alle vier Alarme vergeben - Rückfall auf den Default-Pool

    hardware_alarm_unclaim((uint)alarmNum);

    _pool = alarm_pool_create((uint)alarmNum, POOL_MAX_TIMERS);
    if (_pool == nullptr) return; // malloc fehlgeschlagen - ebenfalls Rückfall, der Alarm ist wieder frei

    // Über das Makro - auf dem RP2350 heißt die Alarm-IRQ anders.
    irq_set_priority(TIMER_ALARM_IRQ_NUM(alarm_pool_get_default_timer(), (uint)alarmNum), TPUART_RP2040_TIMER_IRQ_PRIORITY);
}

bool Timer::startTimer()
{
    if (_running) return true;
    if (_intervalUs == 0) return false;

    if (_pool == nullptr) claimOwnPool();

    _running = true; // muss stehen, bevor der erste Interrupt kommen kann

    // Negatives Intervall: Abstand zwischen den Starts, unabhängig von der Laufzeit eines Ticks.
    bool started = _pool != nullptr
                       ? alarm_pool_add_repeating_timer_us(_pool, -(int64_t)_intervalUs, onTimer, this, &_timer)
                       : add_repeating_timer_us(-(int64_t)_intervalUs, onTimer, this, &_timer);

    if (!started)
    {
        _running = false;
        return false;
    }

    return true;
}

void Timer::stopTimer()
{
    if (!_running) return;

    // Der Callback läuft im Interrupt desselben Kerns - nach der Rückkehr ist kein tick() mehr unterwegs.
    cancel_repeating_timer(&_timer);
    _running = false;
}

#elif defined(ARDUINO_ARCH_ESP32)

bool Timer::supported()
{
    return true;
}

// Läuft im Timer-Task, nicht im Interrupt.
void Timer::onTimer(void *argument)
{
    Timer *self = (Timer *)argument;
    if (self != nullptr) self->fire();
}

bool Timer::startTimer()
{
    if (_running) return true;
    if (_intervalUs == 0) return false;

    // Alle Felder ausdrücklich gesetzt - ohne die hinteren warnt der Compiler bei -Wextra.
    esp_timer_create_args_t args = {
        .callback = onTimer,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tpuart_timer",
        .skip_unhandled_events = true, // lieber einen Takt auslassen als Aufrufe nachholen
    };

    if (esp_timer_create(&args, &_timer) != ESP_OK)
    {
        _timer = nullptr;
        return false;
    }

    _running = true; // muss stehen, bevor der erste Aufruf kommen kann

    if (esp_timer_start_periodic(_timer, _intervalUs) != ESP_OK)
    {
        _running = false;
        esp_timer_delete(_timer);
        _timer = nullptr;
        return false;
    }

    return true;
}

void Timer::stopTimer()
{
    if (!_running) return;

    // esp_timer_stop() wartet auf einen laufenden Callback - nötig, der Timer-Task läuft parallel.
    esp_timer_stop(_timer);
    _running = false;

    esp_timer_delete(_timer);
    _timer = nullptr;
}

#else

bool Timer::supported()
{
    return false;
}

bool Timer::startTimer()
{
    return false;
}

void Timer::stopTimer() {}

#endif

// Aus Interrupt (RP2040) oder Timer-Task (ESP32) - ruft nur weiter.
void Timer::fire()
{
    for (uint8_t i = 0; i < TPUART_TIMER_MAX_CLIENTS; i++)
    {
        DataLinkLayer *client = _clients[i];
        if (client != nullptr) client->tick();
    }
}

// Ohne Sperre: ein Zeiger wird atomar geschrieben. Eingetragen wird auch ohne Timer-Plattform - der
// Rückgabewert sagt, ob getickt wird, contains() ob eingetragen.
bool Timer::add(DataLinkLayer &dll)
{
    if (!contains(dll))
    {
        bool placed = false;

        for (uint8_t i = 0; i < TPUART_TIMER_MAX_CLIENTS; i++)
        {
            if (_clients[i] != nullptr) continue;

            _clients[i] = &dll;
            placed = true;
            break;
        }

        if (!placed) return false; // alle Plätze belegt - siehe TPUART_TIMER_MAX_CLIENTS
    }

    return startTimer();
}

// Anhalten, austragen, wieder anlaufen: erst das Anhalten sichert, dass kein tick() mehr für diese Instanz
// läuft (auf dem ESP32 läuft der Timer-Task parallel). Die übrigen Instanzen verlieren dabei einen Takt.
void Timer::remove(DataLinkLayer &dll)
{
    if (!contains(dll)) return;

    stopTimer();

    for (uint8_t i = 0; i < TPUART_TIMER_MAX_CLIENTS; i++)
        if (_clients[i] == &dll) _clients[i] = nullptr;

    if (clients() > 0) startTimer();
}

bool Timer::contains(const DataLinkLayer &dll) const
{
    for (uint8_t i = 0; i < TPUART_TIMER_MAX_CLIENTS; i++)
        if (_clients[i] == &dll) return true;

    return false;
}

// Setzt einen laufenden Timer mit dem neuen Wert neu auf; von 0 aus startet er, wenn Instanzen eingetragen sind.
void Timer::setInterval(uint32_t intervalUs)
{
    if (intervalUs == _intervalUs) return;

    _intervalUs = intervalUs;

    if (_running) stopTimer();

    if (_intervalUs > 0 && clients() > 0) startTimer();
}

uint32_t Timer::interval() const
{
    return _intervalUs;
}

bool Timer::running() const
{
    return _running;
}

void Timer::trigger()
{
    fire();
}

uint8_t Timer::clients() const
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < TPUART_TIMER_MAX_CLIENTS; i++)
        if (_clients[i] != nullptr) count++;

    return count;
}

} // namespace TPUart
