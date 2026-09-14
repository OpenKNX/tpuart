# Timer.md - Tick-Antrieb (Timer)

Teil von `AGENTS.md`.

- **Die Library treibt `tick()` selbst an** (`Timer.{h,cpp}`, eingetragen aus `begin()`, ausgetragen
  in `end()`, Vorgabe 500µs). Das gibt es, weil `tick()` **ein Byte je Richtung und
  Aufruf** bewegt: aus einem Host-`loop()` getrieben hängen Durchsatz und die 2,8ms-Quittungsfrist
  beide an der Loopfrequenz der Anwendung. In der echten Firmware gemessen, wo der knx-Wrapper
  `process()` einmal je Durchlauf ruft, reichte das nicht.
  **`TPUart::Timer` ist ein SINGLETON - ein Timer für alle BCUs**, und das ist keine Stilfrage. Die
  Library unterstützt mehrere BCUs an einem Gerät, und ein Timer je Instanz trägt das nicht: der RP2040
  hat genau **vier** Hardware-Alarme (`NUM_ALARMS`), von denen der Default-Pool einen belegt - noch im
  Runtime-Init, also vor `main()`, weshalb unser `claimOwnPool()` ihm nichts wegnehmen kann und nichts
  abstürzt. Frei sind damit drei, und die vierte Instanz fiel **stillschweigend** auf den Default-Pool
  zurück: Priorität 0x80 statt 0x40, serialisiert mit fremden Callbacks - genau der Zustand, dessen
  Beseitigung den eigenen Pool überhaupt gerechtfertigt hat, und nichts meldete es.
  Getrennte Timer brächten auch nichts ein: mehrere Alarme auf 0x40 verdrängen sich **nicht** gegenseitig
  (gleiche Priorität verdrängt nicht), sie serialisieren genauso wie ein gemeinsamer Timer - nur
  unvorhersehbar statt in fester Reihenfolge.
  Der Preis ist, dass die Ticks aller Instanzen hintereinander in **einem** Aufruf laufen. Gemessener
  Höchstwert eines einzelnen Ticks: 660µs, davon 269µs im Quittungs-Callback des Aufrufers; Median 1-2µs.
  Drei Instanzen im schlimmsten Fall wären also rund 2ms gegen ein Quittungsfenster von 2,8ms - praktisch
  unkritisch, aber `getTickDurationMaxUs()` gibt es weiterhin **je `DataLinkLayer`**, damit es sichtbar
  bleibt.
  **`TPUART_TIMER_MAX_CLIENTS` ist 1**, weil eine BCU der Normalfall ist; zwei oder drei kosten ein `-D`.
  Der Grund ist **nicht** die Laufzeit - hier stand einmal, ein größeres Array belaste den heißesten Code,
  und das war überzogen: vier Plätze sind grob 25 Takte von den 66.500, die bei 133MHz in einem 500µs-Tick
  stecken, also 0,04%. Der Grund ist, dass eine Grenze sichtbar sein soll: wer eine zweite BCU anschließt,
  schreibt das einmal ausdrücklich hin.
  Ist kein Platz frei, wird die Instanz **abgewiesen** und die bereits eingetragene läuft weiter.
  **Der Fehlfall ist laut**, und das trägt die Entscheidung: die abgewiesene Instanz wird gar nicht
  getickt, `checkTickRate()` sieht in ihrem `loop()` null Ticks im Fenster und meldet
  `Tick stopped - no tick in ... ms`; die Konsole zeigt zusätzlich `(no slot)`. Ein vergessenes `-D` fällt
  damit binnen zwei Sekunden auf. Dass dieser Zweig existiert, war übrigens Glück und nicht Absicht - er ist
  da, weil `delta == 0` sonst eine Division durch null wäre.
  **Das Verzeichnis ist plattformunabhängig, nur der Timer ist es nicht**: `add()` trägt auch dort ein, wo
  `supported()` falsch ist, und nur der Rückgabewert sagt, ob wirklich getickt wird. Ohne diese Trennung
  wäre die Buchführung im nativen Testlauf gar nicht prüfbar.
  **Der Destruktor des `DataLinkLayer` trägt aus**, und das ist kein Schmuck: der Timer hält einen
  *Zeiger*, und der überlebte die Instanz sonst - der nächste Tick liefe in freigegebenen Speicher, auf dem
  RP2040 aus einem Interrupt. `end()` zu verlangen genügt nicht, weil niemand verpflichtet ist, es vor dem
  Wegwerfen zu rufen. `test_timer_slot_is_released_on_destruction` hält das fest (mutationsgeprüft).
  - **Die Vorgabe ist 500µs, und auf manchen Interfaces sollte ein Gerät, das viel sendet, sie
    senken** (`-D TPUART_TIMER_INTERVAL_US=…`). Zum Empfangen ist 500µs bequem (der Bus liefert
    höchstens alle 1,354ms ein Byte), aber **Senden kann die Leitung aushungern**, und das ist
    gemessen, nicht geraten. Die sendende Hardware ist flach: der RP2040-UART läuft mit
    abgeschaltetem FIFO (die DMA will jedes empfangene Byte sofort), er hält also genau zwei Bytes -
    Halte- plus Schieberegister, 572µs bei 38400 Baud. Werden die nur aus dem Tick nachgefüllt,
    **kommt je Tick höchstens ein Byte hinein**, weil das Halteregister erst frei wird, wenn das
    Schieberegister das vorige übernommen hat. Das pendelt sich auf 3 Byte je 1000µs statt 3,5 ein -
    14% Verlust, und es deckt sich mit der Messung: 436 Host-Bytes einzuspeisen dauerte 143ms gegen
    die 125ms, die die Leitung zulässt (`loop1` mit ~12µs Pollrate maß 123ms). Der Software-TX-Ring
    deckte das *nicht* ab, weil er sich nur zum Tick in die Hardware entleerte.
    **Das `RP2040`-Interface ist ausgenommen, seit es einen TX-Interrupt hat**: die Hardware füllt
    sich selbst nach, sobald ein Platz frei wird, ihr Durchsatz hängt also gar nicht mehr am
    Tick-Intervall (siehe `RP2040.h`). **Die Regel gilt nur für `ArduinoSerial`**, und was sie
    auslöst, ist nicht "nur der Tick füllt nach", sondern *wie viel* je Aufruf hineinkommt: mit der
    oben beschriebenen Zwei-Byte-Hardware genau eines, was der Transmitter auch anbietet - daher
    **muss das Intervall unter einer Zeichenzeit der Host-Strecke bleiben** (286µs bei 38400, 573µs
    bei 19200), ein Router dort will also 250µs, zum Preis von rund 1% CPU.
    **`ESP32` braucht das nicht.** Sein Treiberring nimmt beide Bytes eines Oktetts im selben Tick
    (`Transmitter::process()` fragt nach 2, mit Offsetbyte nach 3, und schreibt sie zusammen), ein
    Tick liefert also ein ganzes Oktett - 1146µs Leitungszeit bei 19200, 572µs bei 38400, beides
    länger als der 500µs-Tick. Die Leitung bleibt der Engpass, und das ist der Sinn.
  - **RP2040**: Hardware-Timer, `tick()` läuft **im Interrupt**. Nur
    zulässig, weil beide brauchbaren Interfaces ISR-sicher schreiben (`RP2040` über seinen
    Software-TX-Ring, `ArduinoSerial` über `uart_putc_raw()`). Flash-Schreibvorgänge sind keine
    Gefahr: `rp2040_arduino_platform.cpp` klammert sie in `noInterrupts()`/`idleOtherCore()`, der
    Tick pausiert also, statt aus abgeschaltetem XIP zu laufen.
    **Er hängt in einem EIGENEN Alarmpool, nicht im Vorgabepool**, und das aus zwei Gründen, die beide
    gemessen sind: die Callbacks *eines* Pools laufen aus einem gemeinsamen IRQ-Handler und damit
    serialisiert, und nur ein eigener Pool hat einen eigenen Hardware-Alarm, dessen Priorität sich
    anheben lässt. Sie steht auf `TPUART_RP2040_TIMER_IRQ_PRIORITY` (0x40) statt auf den 0x80, die das SDK
    beim Hochlauf jedem IRQ gibt.
    **Der Grund dafür ist eine Feinheit des Cortex-M, die leicht falsch erinnert wird**: ein Interrupt
    verdrängt dort sehr wohl einen anderen (dafür steht das N in NVIC), aber **nur bei echt höherer
    Priorität - gleiche Priorität verdrängt nicht**. Solange der Tick auf 0x80 mitschwimmt, wartet er auf
    jeden anderen laufenden Handler, und das sind alle. 0x40 genügt und 0x00 wäre falsch: im gesamten
    SDK-Quellbaum gibt es genau *einen* expliziten `irq_set_priority()`-Aufruf, und der setzt die
    Hintergrundarbeit des Netzwerkstacks nach **unten** (`async_context_threadsafe_background` auf
    `PICO_LOWEST_IRQ_PRIORITY`). Über 0x80 sitzt also niemand; 0x00 nähme nur USB und DMA die Luft.
    Beachte, dass daraus zugleich folgt, dass **der Netzwerkstack den Tick gar nicht blockieren kann** -
    eine Vermutung, die hier ausführlich verfolgt und am Quelltext widerlegt wurde.
    **Keine der bequemen SDK-Funktionen ist für eine Library benutzbar**: `alarm_pool_create()` macht ein
    *hard assert*, wenn der Alarm schon vergeben ist, und
    `alarm_pool_create_with_unused_hardware_alarm()` ebenso, wenn gar keiner frei ist - ein Absturz statt
    eines meldbaren Fehlschlags. `hardware_alarm_claim_unused(false)` ist der einzige Weg, der einen
    Misserfolg zurückgibt, deshalb wird damit geprüft, sofort wieder freigegeben und erst dann erzeugt
    (`claimOwnPool()`). Schlägt irgendetwas davon fehl, bleibt `_pool` auf `nullptr` und es läuft wie
    vorher über den Vorgabepool - der Umbau kann also nicht schlechter sein als sein Vorgänger.
    **Was Priorität NICHT löst, ist `PRIMASK`**: `noInterrupts()`, `save_and_disable_interrupts()` und
    `critical_section_enter_blocking()` sperren unabhängig davon. Dagegen hülfe nur ein Tick auf dem
    zweiten Kern, denn PRIMASK ist pro Kern - und selbst der nicht gegen `idleOtherCore()`, das der
    Flash-Pfad zusätzlich benutzt.
  - **ESP32**: `esp_timer`, `tick()` läuft im **Timer-Task**. Ein echter Interrupt scheidet aus -
    `uart_write_bytes()` nimmt einen Treiber-Mutex. Bewusst auch kein festgenagelter Task mit
    `delayMicroseconds()` (so machte es das frühere Diagnosewerkzeug): das kostet einen ganzen Kern,
    und auf einem echten Gerät trägt Kern 0 WLAN und Bluetooth.
  - **Sonst überall**: `Timer::supported()` ist falsch und der Timer startet nicht. Dann tickt **nichts**,
    bis der Aufrufer `Timer::trigger()` aus einem eigenen Kontext ruft - einen Rückfall gibt es nicht, siehe
    den nächsten Punkt.
  - **DER HAUPTLOOP TICKT NIE.** `process()` ruft ausschließlich `loop()`. Das ist eine ausdrückliche
    Festlegung des Anwenders und keine Auslegungsfrage - und sie ist die Lehre aus dem Fall, der diese
    ganze Messung ausgelöst hat: aus dem Hauptloop getrieben lief der Router mit 457 Ticks/s statt 2000 und
    verlor 12% der Quittungen. Ein Rückfall auf den Hauptloop macht genau diesen Zustand zum **stillen
    Normalfall** - er sieht aus wie "es läuft", und niemand erfährt, dass der Antrieb fehlt. Fehlt der
    Timer, tickt lieber nichts: das verlangt eine Entscheidung des Entwicklers statt einer Notlösung.
  - **KEINE STELLSCHRAUBE JE INSTANZ.** Der Timer wird immer benutzt, `begin()` trägt die Instanz ohne
    Nachfrage ein, `end()` und der Destruktor tragen sie aus. Es gab dafür zwischenzeitlich ein
    `setUseTimer(bool)`, und es ist bewusst wieder weg: ein Schalter mit der Bedeutung "ich mache es
    selbst" ist genau die Zuständigkeitsübergabe, die niemand nachlesen kann - und was `process()` dann
    tun soll, war für jeden Leser eine andere Antwort. Ebenso weg ist `setTickInterval()` an der Instanz:
    das Intervall ist global, und eine Methode am Objekt, die alle Instanzen umstellt, wäre eine Falle.
    **Was bleibt, ist genau zweierlei**: `Timer::setInterval(0)` hält den Takt an, und `Timer::trigger()`
    treibt ihn von Hand. Wer beides kombiniert - Timer läuft UND jemand ruft `trigger()` -, hat zwei
    Tick-Kontexte auf einer Schnittstelle. Im IP-Router ausprobiert (Kern 0 gegen Kern 1), und der Chip
    meldete es sofort: `Unknown control byte` für die gestohlenen Empfangsbytes, `PE` (Protokollfehler) in
    `U_State.ind` für die halbierten Sequenzen.
    `usesTimer()` meldet, ob **diese** Instanz vom Timer getickt wird, **dynamisch** und nicht bei
    `begin()` festgeschrieben (der Timer kann später anlaufen, wenn eine andere Instanz ihren Platz
    freigibt). Es ist der einzige Diagnosewert zum Antrieb, und er ist wichtiger als er aussieht: liefert
    er falsch, ohne dass jemand `trigger()` ruft, steht die Schicht still - und das sieht von außen aus wie
    "die BCU antwortet nicht".
  - **`Timer::trigger()` treibt den Takt von Hand** - ein `tick()` je eingetragener Instanz, genau das,
    was der plattformeigene Callback tut. Das ist der **einzige** Weg neben dem Timer, und damit die
    Antwort auf "Plattform ohne Timer-Unterstützung": kein Rückfall, sondern ein ausdrücklicher Aufruf aus
    einem Kontext, den der Entwickler selbst stellt. Es gilt dieselbe Anforderung wie an den
    Callback: der Kontext muss dürfen, was `tick()` tut - auf dem ESP32 also kein ISR, weil
    `uart_write_bytes()` einen Mutex nimmt.
  - **Der Timer-Antrieb gibt eine Zusage, die zwei Kerne nicht geben.** Ein Interrupt läuft immer zu
    Ende und wird nie vom Hauptloop verdrängt, `tick()` und `loop()` konnten sich also nie
    verschränken. Mit `tick()` auf Kern 1 ist das weg. Die SPSC-Ringe überstehen es (der Kopf wird
    erst veröffentlicht, wenn der Eintrag vollständig geschrieben ist), und die Sendewarteschlange
    berührt es gar nicht mehr - sie gehört seit dem Umbau dem Hauptkontext allein, der Tick sieht nur
    die Vorlage. Jedes ab hier neu hinzugefügte geteilte Feld muss trotzdem gegen echte Parallelität
    geprüft werden, nicht nur gegen Verdrängung.
  - **Folge für Callbacks**: auf dem RP2040 läuft der Quittungs-Callback
    (`registerCheckAcknowledge`) jetzt im Interrupt-Kontext. Er musste ohnehin kurz und
    allokationsfrei sein; das ist nun keine Empfehlung mehr.
