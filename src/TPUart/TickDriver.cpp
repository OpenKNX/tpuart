#include "TPUart/TickDriver.h"

#include "TPUart/DataLinkLayer.h"

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

bool TickDriver::start(DataLinkLayer &dll, uint32_t intervalUs)
{
    if (_running) return true;
    if (intervalUs == 0) return false;

    _dll = &dll;
    _running = true; // muss stehen, bevor der erste Interrupt kommen kann

    // NEGATIVES Intervall: der Abstand gilt zwischen den STARTS zweier Aufrufe, nicht zwischen dem Ende
    // des einen und dem Start des nächsten - nur so ist der Takt von der Laufzeit eines Ticks unabhängig.
    if (!add_repeating_timer_us(-(int64_t)intervalUs, onTimer, this, &_timer))
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
