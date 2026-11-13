# Build.md - Build-Umgebungen und Tests

Teil von `AGENTS.md`.

## Build-Umgebungen

`platformio.ini` hat **drei Envs, und alle drei sind Testumgebungen**: `pico_test`, `esp32_test` und
`native_test`. Eine Produktions-Env gibt es bewusst nicht - dieses Repository ist eine Library, kein
Programm. Gebaut und geprüft wird ausschließlich über `pio test`:

```
pio test -e pico_test
pio test -e esp32_test
pio test -e native_test                                       (auf dem PC, ohne Gerät)
pio test -e pico_test --without-uploading --without-testing   (nur übersetzen, ohne Gerät)
```

**`native_test` ist eine Ergänzung, kein Ersatz.** Es fährt dieselben Fälle gegen dieselbe Library,
aber mit **gestellter Uhr**: `pump()` dreht die Zeit in 50µs-Schritten weiter, statt zu warten. Der
ganze Durchlauf kostet damit rund zwei Sekunden statt fünfundzwanzig, und die beiden
Fünf-Sekunden-Fälle kosten gar nichts mehr.
Was dabei **wegfällt, ist das Zeitverhalten selbst** - und daran ist hier schon ein Fehler
aufgefallen, den eine gestellte Uhr wegdefiniert (siehe die Zustellkosten der Testvorrichtung weiter
unten). Die Schrittweite *ist* die Tickrate; 50µs liegen unter dem 500µs-Vorgabetakt und weit unter
der Bytezeit des Busses, die Pausenerkennung sieht nativ also feiner auf als auf jeder echten
Plattform. Ein Fall, der nativ besteht, kann auf Hardware knapp scheitern. **Vor einer Freigabe zählen
die Hardwareläufe.**
Möglich ist das **ohne eine Zeile Änderung in `src/`**: die Library benutzt aus Arduino nur `millis()`
und `micros()`, und die plattformgebundenen Teile schalten sich über ihre eigenen
`ARDUINO_ARCH_*`-Klammern selbst ab (`Interface/RP2040.cpp`, `ESP32.cpp`, und `Timer.cpp` hat
einen `#else`-Zweig, der `supported()` auf `false` setzt). Den Ersatz liefert
`test/test_tpuart/native/Arduino.h`; nur diese Env nimmt den Ordner über `-I` in den Suchpfad, in den
Hardware-Envs liegt das echte `Arduino.h` davor.
Der Schim ist **header-only, und das ist Bedingung**: eine `.cpp` dort würde PlatformIO in *jeder* Env
mitübersetzen (der Testordner wird rekursiv gebaut) und den Hardware-Envs ein zweites `millis()`
verpassen. Der Zählerstand steckt deshalb in einem funktionslokalen `static`.
Was nativ nicht mitläuft, ist `interface_check.cpp` - es instanziiert `ArduinoSerial<decltype(Serial1)>`
und die Plattform-Interfaces, und beides gibt es dort nicht. Die Env braucht einen **Host-Compiler im
`PATH`** (`g++` oder `clang++`); fehlt er, scheitert sie beim Übersetzen, ohne die anderen beiden zu
berühren.

`pio run` ist **kein Einstiegspunkt** und scheitert im Linker mit `undefined reference to 'setup'`:
`setup()` und `loop()` kommen aus `test/test_tpuart/test_main.cpp`, und dieser Ordner wird nur bei
einem Testlauf mitübersetzt. Zum Aufräumen taugt `pio run` trotzdem, das Ziel kommt vor dem Linker:
`pio run -t clean -e pico_test`.

**Dass jedes Interface sich überhaupt übersetzen und linken lässt, prüft
`test/test_tpuart/interface_check.cpp`**, und das läuft in beiden Testumgebungen mit. Dort wird
`ArduinoSerial<decltype(Serial1)>` ausdrücklich instanziiert - als Template entsteht ohne Aufrufer
kein einziger Rumpf, ein Fehler darin fiele sonst erst im Fremdprojekt auf -, und je Plattform werden
`RP2040` bzw. `ESP32` mit einem `DataLinkLayer` verdrahtet. Die Funktion dort wird **nie aufgerufen**:
der Compiler übersetzt sie vollständig und der Linker löst jedes Symbol auf, aber kein Konstruktor
läuft und kein Interface fasst Hardware an. Ein globales Objekt stattdessen beanspruchte auf dem
RP2040 beim Start einen DMA-Kanal, den die echten Tests dann nicht mehr bekämen.

Beide Envs bauen immer mit `framework = arduino` - einen nativen Host-Bau gibt es nicht.


### Tests

`pio test -e pico_test` und `-e esp32_test` fahren die Sammlung in `test/test_tpuart/test_main.cpp`
**auf dem Zielgerät**, gegen das `Dummy`-Interface, das im selben Ordner liegt. Unity ist das
Framework (PlatformIOs Vorgabe, automatisch geholt - im Projekt landet keine Abhängigkeit).

`test_build_src = yes` ist tragend: ein Testbau übersetzt `src/` sonst gar nicht und die Library
fehlte schlicht.

**`test/test_tpuart/unity_config.h` gibt es für genau eine Zeile.** PlatformIO erzeugt diese Datei
sonst selbst und beendet sie mit `void unityOutputComplete(void) { Serial.end(); }`. Auf einem Board,
dessen `Serial` das USB-CDC *ist*, meldet das das Gerät mitten in `UNITY_END()` vom Bus ab: dem
Testläufer fehlt die Schlusszeile (`ClearCommError failed` nach 23 von 24 Fällen), und der nächste
Upload findet überhaupt keinen Port mehr (`Please specify upload_port`), bis das Board neu gesteckt
wird. Unsere Fassung leert stattdessen den Puffer. Die Datei bereitzustellen heißt zugleich, die vier
Ausgabefunktionen bereitzustellen - sie stehen am Kopf von `test_main.cpp`.
`setup()` wartet vor `UNITY_BEGIN()` darauf, dass der Host den Port öffnet (mit einer Frist, damit die
Sammlung auch unbeaufsichtigt läuft); ohne das fehlt der erste Fall regelmäßig im Bericht. Nach dem
Lauf hält `loop()` die Verbindung offen und nimmt zwei Tasten entgegen: `r` fährt die Sammlung erneut,
`b` setzt den RP2040 in den Bootloader.

**Alle drei Umgebungen sind verifiziert**, nicht nur gebaut: 96 von 96 auf `pico_test` und dieselben
96 auf `esp32_test` (~39s), beide auf echter Hardware, dazu `native_test` (~1,5s). Da die
Fälle echte Fristen ausmessen, ist gerade die Übereinstimmung über zwei sehr verschiedene Uhren und
Treiberpuffer hinweg das Interessante am Ergebnis - besonders bei den Buslast-Fällen, deren Schranken
an einer Messspanne von rund 1000ms hängen: nativ ist die deterministisch, auf Hardware nicht.
**WAS DIE SAMMLUNG NICHT PRÜFT, ist der Antrieb selbst.** Die Vorrichtung hält `Timer::setInterval(0)`,
getickt wird ausschließlich aus `pump()`/`tickOnly()`. Geprüft ist damit die Buchführung des Timers
(Eintragen, Austragen, Plätze, Intervall) - dass der Hardware-Timer wirklich mit dem eingestellten Takt
feuert, belegt nur der Betrieb auf einem Gerät. Für das Singleton steht dieser Nachweis noch aus; die
Messungen mit 0 Verzögerungen über 26000 Ticks stammen vom vorherigen, instanzeigenen Antrieb.
**Der native Lauf ersetzt die beiden Hardwareläufe trotzdem nicht**, und dafür gibt es jetzt einen
Beleg statt einer Vermutung: er war grün, als `pico_test` rot war, und konnte den Fehler prinzipiell
nicht sehen - dort hängt die Uhr an den Schleifendurchläufen und nicht an der Wanduhr, eine teure
Testvorrichtung kostet also nichts (siehe die Zustellkosten weiter unten).

**Die Testvorrichtung muss billig zustellen, sonst verfälscht sie Zeitmessungen.** Mit
angehaltenem Timer laufen Tick und Loop serialisiert: was der Telegramm-Callback kostet, liegt
zwischen dem Tick, der das letzte Byte einer Sequenz verbraucht, und dem nächsten, der die Leere
bemerkt und `_emptySince` setzt - die Zustellkosten verschieben also die Pausenerkennung nach hinten.
`_frames` wuchs während eines Falls um und kopierte dabei jeden Eintrag samt Datenvektor neu;
`test_confirmation_after_broken_echo_arrives_after_pause` fiel dadurch auf dem RP2040 in etwa der
Hälfte der Läufe durch - die Pause wurde erst erkannt, nachdem das `L_Data.con` bereits eingetroffen
und im `Resync` verworfen war. Behoben mit `reserve()` und `push_back(std::move(...))`; auf dem ESP32
war die Reserve zufällig groß genug, dort fiel es nie auf.

**Derselbe Fehler ein zweites Mal, an anderer Stelle: `Dummy::available()` war quadratisch.** Es lief
bei jedem Aufruf von `_pos` bis zum Ende der Warteschlange, und `read()` ruft es gleich noch einmal -
bei einem ohne Pause eingespeisten Block sind damit alle Bytes sofort verfügbar und jeder Aufruf zählt
den ganzen Rest erneut ab. `test_rx_queue_overflow_is_counted` speist 900 Bytes ein, das sind rund
810.000 Schleifendurchläufe je Richtung: auf dem PC unsichtbar, auf dem RP2040 über 70ms gegen ein
80ms-Budget. Der Fall verlor seine Ticks also an die Vorrichtung und meldete 0 Ringüberläufe - **nativ
grün, auf Hardware rot**. Behoben, indem der Fahrplan fortgeschrieben statt neu durchgerechnet wird
(`_availableCount`/`_scanArrivesAt`); tragend ist dabei, dass `read()` `_nextAvailableAt` um genau die
Pause weiterschiebt, um die `_pos` vorrückt - die absoluten Ankunftszeiten der verbliebenen Bytes
ändern sich dadurch nicht, ein einmal verfügbares Byte bleibt also verfügbar.
Die Lehre ist dieselbe wie beim `_frames`-Fall und inzwischen zweimal belegt: **Kosten in der
Vorrichtung sind nicht neutral, sie gehen dem Prüfling vom Zeitbudget ab** - und der native Lauf kann
das nicht zeigen, weil dort die Uhr an den Schleifendurchläufen hängt und nicht an der Wanduhr. Genau
deshalb zählen vor einer Freigabe die Hardwareläufe.

**Die neun Busmonitor-Fälle sind gegen Mutation geprüft**, und das war nötig: `abort()`
auszukommentieren ließ zunächst nur *einen* der beiden dafür gedachten Fälle scheitern.
`test_monitor_aborts_running_transmission` bestand weiter, weil schon der Wächter in
`Transmitter::process()` verhindert, dass noch Bytes hinausgehen - der Fall prüfte also den Wächter
und nicht den Abbruch. Erst die zusätzliche Zusicherung auf `TxState::Idle` bindet ihn an
`abort()`. Wer hier einen Fall ergänzt: prüfe, ob er ohne die zugehörige Änderung wirklich scheitert.

`monitor_speed = 115200` steht in `[env]` und muss zum `Serial.begin(115200)` in `test_main.cpp`
passen; PlatformIOs Vorgabe wäre 9600. Das fiel nur auf dem ESP32 je ins Gewicht, wo `Serial` ein
echter UART hinter dem Brückenchip ist - auf dem RP2040 ist `Serial` das USB-CDC und die Baudrate
Zierde, weshalb die fehlende Zeile bis zur ersten ESP32-Monitorsitzung unbemerkt blieb, die dann nur
Buchstabensalat zeigte. Der Testläufer selbst war nie betroffen: er benutzt `test_speed`, dessen
Vorgabe bereits 115200 ist.

**Auf der Hardware läuft alles in Echtzeit - dort gibt es keine stellbare Uhr** (die hat nur
`native_test`, siehe oben). Ein Fall kostet, was die Sache kostet:
2,6ms für die Pausenerkennung, 5s für einen Verbindungsverlust. Das ist der Preis dafür, die echten
Fristen zu prüfen statt heruntergedrehter, und der ganze Lauf bleibt trotzdem im Sekundenbereich.
Wird das je untragbar, ist der Hebel eine Indirektion `TPUart::nowMs()`/`nowUs()` - 14 direkte
`millis()`/`micros()`-Aufrufstellen in 7 Dateien, was zugleich einen nativen Bau möglich machte.
`TPUART_TX_CONFIRM_TIMEOUT_MS` wird in `[env]` von 10s auf 1s gesenkt. Zwei Fälle hängen daran - der
Wachhund, der bei ausbleibendem `L_Data.con` einen Reset auslöst, und die Auffrischung der Frist durch
jedes Echo. Beide brauchen echte Wartezeit; mit dem Vorgabewert dauerte allein der Wachhund-Fall zehn
Sekunden. Geprüft wird damit der Mechanismus, nicht die Zahl.

Jeder Fall bekommt eine **frische Vorrichtung** (`Dummy` + `DataLinkLayer`, auf dem Heap in
`setUp()`/`tearDown()`), weil `RxState`/`TxState` von außen nicht zurücksetzbar sind - ein
liegengebliebener Resync sickerte sonst in den nächsten Fall. Die Vorrichtung setzt
`Timer::setInterval(0)`, damit kein Timer nebenher läuft; getickt wird ausschließlich aus `pump()` bzw.
`tickOnly()`. Einen Fall
hinzuzufügen heißt, eine parameterlose Funktion zu schreiben und sie mit `RUN_TEST()` in `setup()`
aufzuführen - diese Liste ist die einzige Stelle, die von ihm weiß.

Zum RP2040: hier stand zwischenzeitlich eine zweite Env, die über J-Link flashen sollte, weil der
USB-Port am Ende jedes Testlaufs verschwand. Die Ursache lag aber nicht am USB, sondern an der
Unity-Anbindung von PlatformIO (siehe `unity_config.h` oben). Mit eigener `unity_config.h` läuft der
Upload ganz normal über USB, auch mehrfach hintereinander. Die J-Link-Env ist damit weg.

### `library.json`

Version **2.0.0**. `frameworks`/`platforms` sind gesetzt, damit der LDF keine fremden Plattformen
versucht. Einen `srcFilter` braucht es nicht: `src/` enthält ausschließlich Library-Code.
**Legst du eine Datei in `src/`, die nicht Teil der Library ist, brauchst du den Filter wieder.**
Testcode braucht hier keinen Eintrag - er liegt in `test/`, was ein Verbraucher nie sieht.
