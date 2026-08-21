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

// Besorgt einen eigenen Hardware-Alarm samt Pool und hebt dessen IRQ-Priorität an. Schlägt das fehl,
// bleibt _pool auf nullptr und startTimer() nimmt den Default-Pool - dann verhält sich alles wie vorher.
//
// DASS DAS HIER NUR EINMAL PASSIERT, ist der eigentliche Gewinn des Singletons: der RP2040 hat vier
// Hardware-Alarme, einer gehört dem Default-Pool, es sind also drei zu holen. Mit einem Pool je BCU war
// die vierte Instanz zwangsläufig ohne - jetzt reicht einer für beliebig viele.
//
// DIE REIHENFOLGE IST NICHT VERHANDELBAR, denn beide bequemen SDK-Funktionen sind für eine Library
// unbrauchbar: alarm_pool_create() ruft intern ta_hardware_alarm_claim() und macht ein HARD ASSERT, wenn
// der Alarm schon vergeben ist; alarm_pool_create_with_unused_hardware_alarm() claimt mit required=true
// und assertet, wenn gar keiner mehr frei ist. Beides wäre ein Absturz des Geräts statt eines
// Fehlschlags, den wir melden könnten. hardware_alarm_claim_unused(false) ist der einzige Weg, der einen
// Misserfolg ZURÜCKGIBT - deshalb wird damit geprüft, sofort wieder freigegeben und dann erst erzeugt.
//
// Das Fenster zwischen unclaim und create ist ungefährlich: das hier läuft aus begin(), im Hauptkontext,
// und niemand sonst greift zu diesem Zeitpunkt nach Hardware-Alarmen. Der Default-Pool ist zu dem
// Zeitpunkt längst versorgt - er holt seinen Alarm im Runtime-Init, also vor main().
void Timer::claimOwnPool()
{
    int alarmNum = hardware_alarm_claim_unused(false);
    if (alarmNum < 0) return; // alle vier Alarme vergeben - Rückfall auf den Default-Pool

    hardware_alarm_unclaim((uint)alarmNum);

    _pool = alarm_pool_create((uint)alarmNum, POOL_MAX_TIMERS);
    if (_pool == nullptr) return; // malloc fehlgeschlagen - ebenfalls Rückfall, der Alarm ist wieder frei

    // Über das Makro und nicht als TIMER_IRQ_0 + n: auf dem RP2350 heißt die erste Alarm-IRQ anders, und
    // arduino-pico baut beide Chips unter ARDUINO_ARCH_RP2040.
    irq_set_priority(TIMER_ALARM_IRQ_NUM(alarm_pool_get_default_timer(), (uint)alarmNum), TPUART_RP2040_TIMER_IRQ_PRIORITY);
}

bool Timer::startTimer()
{
    if (_running) return true;
    if (_intervalUs == 0) return false;

    if (_pool == nullptr) claimOwnPool();

    _running = true; // muss stehen, bevor der erste Interrupt kommen kann

    // NEGATIVES Intervall: der Abstand gilt zwischen den STARTS zweier Aufrufe, nicht zwischen dem Ende
    // des einen und dem Start des nächsten - nur so ist der Takt von der Laufzeit eines Ticks unabhängig.
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

    // Erst abbestellen, dann das Flag löschen: cancel_repeating_timer() kehrt zurück, wenn kein Aufruf
    // mehr ansteht. Ein bereits laufender läuft zu Ende - was hier aber nicht passieren kann, denn der
    // Callback läuft im Interrupt DESSELBEN Kerns, der diese Funktion aufruft, und ein Interrupt wird vom
    // Hauptkontext nicht unterbrochen. Nach der Rückkehr ist also sicher kein tick() mehr unterwegs.
    //
    // Die Einschränkung, die daran hängt: begin()/end() aller Instanzen müssen vom SELBEN Kern kommen -
    // der Pool gehört dem Kern, der ihn erzeugt hat.
    cancel_repeating_timer(&_timer);
    _running = false;
}

#elif defined(ARDUINO_ARCH_ESP32)

bool Timer::supported()
{
    return true;
}

// Läuft im Timer-TASK, nicht im Interrupt - siehe Header. Damit ist alles erlaubt, was auch aus dem
// Hauptloop erlaubt wäre, insbesondere der Mutex des UART-Treibers.
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

    // esp_timer_stop() wartet auf einen gerade laufenden Callback, esp_timer_delete() ebenfalls - danach
    // ist sicher keiner mehr unterwegs. Hier ist das keine Formalie wie auf dem RP2040, sondern nötig:
    // der Timer-Task läuft echt parallel zum Hauptkontext.
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

// Der eine Aufruf, der zählt. Er läuft je Plattform in einem anderen Kontext (RP2040: Interrupt,
// ESP32: Timer-Task) und darf deshalb nichts tun, was in einem ISR verboten wäre - er ruft nur weiter.
//
// KEINE PRÜFUNG AUF _running: ein Callback, der bereits unterwegs ist, während stopTimer() läuft, kann
// auf beiden Plattformen nicht auftreten (siehe die Begründung dort). Der Zeigerabgleich reicht.
void Timer::fire()
{
    for (uint8_t i = 0; i < TPUART_TIMER_MAX_CLIENTS; i++)
    {
        DataLinkLayer *client = _clients[i];
        if (client != nullptr) client->tick();
    }
}

// OHNE SPERRE, und das trägt: ein Zeiger wird atomar geschrieben, der Callback sieht den Platz also
// entweder leer oder vollständig belegt. Dass er die Instanz sofort tickt, ist unbedenklich - sie ist
// fertig konstruiert, und tick() kehrt bis zur ersten Verbindung ohnehin gleich wieder um.
// DAS VERZEICHNIS IST PLATTFORMUNABHÄNGIG, nur der Timer ist es nicht - deshalb wird hier auch dann
// eingetragen, wenn die Plattform keinen Timer hergibt oder das Intervall 0 ist. Der Rückgabewert sagt, ob
// tatsächlich getickt wird; wer nur wissen will, ob er eingetragen ist, fragt contains().
//
// Das ist nicht bloß Symmetrie: ohne diese Trennung ließe sich die Buchführung im nativen Testlauf gar
// nicht prüfen, weil supported() dort false ist und add() vor dem Eintragen umkehrte.
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

// ANHALTEN, AUSTRAGEN, WIEDER ANLAUFEN - und in genau dieser Reihenfolge. Der Timer wird angehalten,
// BEVOR der Platz frei wird, weil nur das Anhalten die Zusage gibt, dass danach kein tick() mehr für diese
// Instanz läuft; end() schließt gleich danach das Interface, ein verspäteter Tick griffe also ins Leere.
//
// Den Zeiger einfach auf nullptr zu setzen genügte auf dem RP2040 (Interrupt desselben Kerns), aber NICHT
// auf dem ESP32: dort läuft der Timer-Task echt parallel und könnte mitten im tick() stehen. Ein
// Generationszähler mit Warteschleife wäre die Alternative - das dokumentierte Warten von
// esp_timer_stop() tut dasselbe, ohne eigene Schleife.
//
// Die kurze Unterbrechung trifft die übrigen Instanzen mit: sie verlieren einen Takt. Das passiert nur in
// end(), also nicht im Betrieb.
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

// Ein laufender Timer wird mit dem neuen Wert neu aufgesetzt - anders als früher, wo eine Zahlenänderung
// den Antrieb nur anhalten konnte. Das ist jetzt gefahrlos, weil der Kontext derselbe bleibt: es wechselt
// nur die Periode, nicht wer tickt.
void Timer::setInterval(uint32_t intervalUs)
{
    if (intervalUs == _intervalUs) return;

    _intervalUs = intervalUs;

    // Auch der umgekehrte Weg zählt: von 0 auf einen Wert zu wechseln startet den Timer, wenn schon
    // Instanzen eingetragen sind. Sonst hinge das Anlaufen an einem begin(), das längst gelaufen ist.
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
