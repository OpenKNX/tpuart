# Interface.md - UART-Interfaces

Teil von `AGENTS.md`.

## Aktueller Stand (Interface-Schicht)

`src/TPUart/Interface/` enthält ein abstraktes UART-Interface und vier
Implementierungen, alle bisher ausgebaut:

- **`Abstract.h`** - die gemeinsame Schnittstelle. Rein pollend
  (`begin/end/flush/available/availableForWrite/read/write/overflow`).
  **`available()` liefert eine Anzahl, keinen Wahrheitswert** - die Schicht darüber muss
  wissen, wie weit sie hinter dem Bus liegt (siehe die Quittungsunterdrückung weiter
  unten), nicht nur, ob überhaupt etwas da ist.
  **Auf der Leseseite darf ein Interface nicht STAPELN.** Die Schicht darüber misst Pausen
  daran, wann ein Byte *sichtbar* wird, nicht daran, wann es auf der Leitung war - ein
  zurückgehaltenes Byte lügt also über seine Ankunftszeit, und die Pausenerkennung ist das,
  was ein Telegramm vom nächsten trennt. Hardware-FIFOs sind die Falle: der UART-Treiber der
  ESP-IDF lässt Empfangenes im 128-Byte-Hardware-FIFO liegen, bis 120 Bytes beisammen sind
  oder die Leitung 10 Symbolzeiten still war (5,7ms bei 19200 - mehr als die 2,6ms-Schwelle
  fürs Frame-Ende), was ein 263-Byte-Echo bei genau 120 Bytes abschnitt. `ESP32::begin()`
  setzt deshalb `uart_set_rx_full_threshold(1)` und `uart_set_rx_timeout(1)`. Auf dem RP2040
  ergibt sich das von selbst, weil DMA jedes Byte sofort ablegt - weshalb die Annahme so
  lange unbemerkt blieb. **Für `ArduinoSerial` ist das nie geprüft worden** - ob der
  eingepackte Kern beim Empfang stapelt, ist offen, und die Pausenerkennung hängt daran.
  Das Fehlerbild ist wissenswert, denn es meldet sich nicht selbst: die Pausenerkennung ist
  keine Prüfung, sondern das *Axiom* - sie legt überhaupt erst fest, wo ein Frame endet, und
  dahinter gibt es kein zweites, unabhängiges Signal, das einer falschen Grenze widersprechen
  könnte. Wo Redundanz vorhanden war, hat sie gehalten (jedes Phantomtelegramm kam als
  `INVALID` heraus, die Prüfsumme hat sie alle erwischt), aber **Steuerbytes haben keine
  Prüfsumme** - ein einzelnes Byte ist seine eigene Nachricht. Durchgekommen sind deshalb
  erfundene Chip-Zustände und, der scharfe Fall, ein erfundenes positives `L_Data.con`
  (`0x8B & 0x7F == 0x0B`), das `Transmitter::confirmed()` aufrief und den Sendeweg für einen
  Versand freigab, der keine Bestätigung hatte. Nachrüsten lässt sich das weiter unten nicht:
  nach einem abgeschnittenen Frame in den Resync statt in den Leerlauf zu gehen würde genau
  den Fall zerstören, für den die Schwelle auf 2600 gesenkt wurde (das `L_Data.con`, das nach
  einem verstümmelten Echo hinter einer Pause eintrifft, abgesichert durch zwei Testfälle).
  Der Schutz gehört dorthin, wo er jetzt liegt - das Interface darf nicht lügen.
  Beachte die Asymmetrie zu *unserer eigenen* Verspätung, denn sie erklärt, warum nur eines von
  beidem zerstörerisch ist. Auf der Leseseite in Rückstand zu geraten ist überhaupt erst
  einmal schwierig: was der Chip weiterreicht, kommt im Bustakt, ein Oktett je 1,354ms bei
  9600 Baud, gegen einen 500µs-Tick - und das schließt das Echo unseres eigenen Versands ein
  (263 Oktetts gemessen als ~355ms, also Busgeschwindigkeit, nicht Host-Geschwindigkeit). Nur
  Steuerbytes kommen im Takt der Host-Strecke, und sie kommen zu zweit, nicht zu Hunderten.
  Die Zeichenzeit des Hosts ist die Grenze der *Schreib*seite (siehe `Timer`). Und selbst
  wenn der Tick zurückfällt, meldet `available()` weiterhin den Rückstand, es wird also keine
  Stille vorgetäuscht und kein Frame zerschnitten: die Quittung kommt lediglich zu spät und
  wird unterdrückt - eingeschränkte Funktion, ehrliche Daten. Ein stapelndes Interface
  erfindet dagegen Stille, und das macht aus Nutzlast Steuerbytes.
  **`availableForWrite()` liefert eine Anzahl, keinen Wahrheitswert**, und `write()` darf
  nicht blockieren - der Aufrufer fragt vorher nach Platz. Die Anzahl gibt es, damit mehrere
  zusammengehörende Bytes reserviert und unmittelbar hintereinander abgesetzt werden können,
  ohne dass eine Quittung dazwischenrutscht; jede Implementierung muss deshalb die
  Schreibreihenfolge wahren.
  **Die Zahl muss in BEIDE Richtungen stimmen**, und beide Fehler sind hier gemacht worden.
  Die Zahl, die sie definiert, ist **`TPUART_TX_INTERFACE_BUFFER` (4)**, und sie steht in
  `Transmitter.h` - sie ist eine Anforderung des Protokolls, keine Eigenschaft irgendeiner
  Hardware, und sie ist aus den beiden Konstanten abgeleitet (`max`), aus denen sie folgt
  (`TPUART_CTRL_MAX_GROUP`, `TPUART_TX_ATOMIC_BYTES`). Die Interfaces leiten ihre Puffergröße
  davon ab, sie kann also nicht an einer Stelle wachsen und an der anderen zurückbleiben.
  - **Zu klein**: 0 oder 1 zu melden bricht eine 4-Byte-Steuersequenz *dauerhaft*, nicht nur
    verlangsamt - sie darf nicht zerteilt werden, bleibt also am Kopf der Warteschlange stehen
    und blockiert alles dahinter. Siehe `ArduinoSerial`.
  - **Zu groß**: ein Interface darf nie mehr als so viele Bytes *unter sich* zulassen, welche
    Puffer es auch hat. Was in einem tiefen FIFO liegt, verzögert alles Folgende, und das
    nächste Byte ist womöglich ein `U_Ackn.req` mit 2,8ms Frist (Siemens TP-UART 2+ S. 25).
    32 Byte in einem TX-FIFO sind 18ms bei 19200 - die Quittung käme nicht nur zu spät, der
    Chip hängte sie ans *nächste* Telegramm. Tiefer zu puffern bringt ohnehin nichts: der Bus
    nimmt ein Byte je 1,354ms, und was länger wartet, ist zu spät.
  Zwei Wege erfüllen beides, beide sind im Einsatz: ein kleiner Ring plus ein exaktes
  Hardware-Signal (`RP2040` - FIFO aus, das Halteregister ist damit ein Byte tief und
  `uart_is_writable` ist genau; Tiefe beweisbar <= 5), oder **Buchführung über die Leitung**
  (`ArduinoSerial`, `ESP32` - Bytezeit aus der Baudrate, 8E1 = 11 Bit, jedes übergebene Byte
  schreibt den Fahrplan fort, und was noch aussteht, folgt aus der Uhr). Das zweite braucht
  man, wenn die Schicht darunter ihren Füllstand verbirgt, was beide tun. Eine
  Callback-/Benachrichtigungs-API gibt es bewusst **nicht**: Interrupts können global
  vorübergehend gesperrt werden, und eine IRQ-Zustellung genau einmal je Byte ist nicht
  garantiert (besonders mit DMA), der Datenpfad muss also immer pollend sein, nie schiebend.
  Interrupts werden ausschließlich als interne Implementierungssache innerhalb eines einzelnen
  Interfaces benutzt (siehe `RP2040`), nie um Verarbeitung in höheren Schichten anzustoßen.
- **`RP2040`** - Hardware-UART mit DMA-getriebenem Empfang in einen Ringpuffer, dessen Größe
  `TPUART_RP2040_RX_BUFFER_EXP` bestimmt (Vorgabe 8 = 256 Byte). Ein Gerät, dessen Hauptloop
  eine ganze Sekunde stehenbleiben kann, will mehr: der Bus liefert höchstens 738 Byte/s, mit
  `-D TPUART_RP2040_RX_BUFFER_EXP=11` (2048) ist das mit Reserve abgedeckt. Die frühere
  Diagnose-Env stellte genau das ein. Achtung: die Zahl, auf der diese Auslegung ursprünglich
  beruhte (~1745 Byte/s, die gesättigte 19200er Host-Strecke), ist der falsche Bezug - zitiere
  sie nicht als Anforderung.
  DMA ist der *einzige* Empfangspfad (kein reiner IRQ-Modus).
  **Zwei verschiedene Arten von Überlauf, und sie brauchen verschiedene Erkennung.** Ein
  Hardware-Overrun des UART (`UART_UARTRSR_OE_BITS`) heißt, dass DMA das Datenregister nicht
  rechtzeitig geleert hat. Ein *Ring*-Überlauf heißt, dass DMA unseren Leser überholt hat - die
  Hardware kam dort bestens mit, `OE` bleibt also sauber und taugt dafür nicht.
  Der Ringfall wird in `read()` gerastet (`_ringOverflow`) und von `overflow()` abgeholt. Die
  Rastung trägt: `read()` muss `_dmaReaderCount` korrigieren, wenn es das Überholen bemerkt,
  und zerstört damit genau die Zählerdifferenz, die die Bedingung geprüft hat - und `processRx()`
  fragt `overflow()` erst *nach* `read()`. Dort neu zu rechnen ergibt deshalb immer falsch, und
  genau so blieben Ringüberläufe früher vollständig unberichtet.
  Wenn `read()` korrigiert, setzt es beim **ältesten noch gültigen** Byte wieder auf
  (`dmaTransferCount() - BUFFER_SIZE + 1`), nicht beim neuesten. Zum neuesten zu springen wirft
  einen ganzen Ring voll noch lesbarer Bytes weg und vervielfacht den Verlust um die Puffergröße.
  Der DMA-Transferzähler (`TPUART_RP2040_TRANSFER_COUNT`) läuft ab, und die Maschine bliebe
  danach für immer stehen, er muss also neu gestartet werden. Früher war er der größte Wert, den
  die Hardware nimmt (`UINT32_MAX >> 1`, über einen Monat Dauerlast) - **bewusst klein gemacht**:
  `TPUART_RP2040_TRANSFER_COUNT_EXP` steht auf 20, also rund 24 Minuten bei voller Buslast, und
  `-D …_EXP=12` provoziert einen Neustart binnen Sekunden. Ein Pfad, der einmal im Monat läuft,
  ist ein Pfad, den nie jemand ausprobiert, und dieser muss Schreibposition und Zähler über den
  Neustart hinweg stimmig halten.
  Das ist gefahrlos, weil das Neustartfenster kurz ist: die DMA steht zwischen dem Ablauf des
  Zählers und dem nächsten `checkRestart()`, also höchstens einen Tick (500µs), und in der Zeit
  liefert der Bus höchstens **ein** Byte (ein TP1-Zeichen dauert 1,354ms) - genau das, was das
  Halteregister des UART puffert. Für einen Overrun bräuchte es zwei Bytes im Fenster.
  Ablauf: der DMA-Fertig-Interrupt (`onDmaComplete()`) setzt nur ein Flag - keine Verarbeitung im
  ISR-Kontext - und der eigentliche Neustart (`restartDma()`) passiert aus dem normalen Pollpfad
  über `checkRestart()` in `available()`.
  Der Neustart ist **zeigererhaltend**: `_dmaReaderCount` wird *nicht* zurückgesetzt; stattdessen
  wird `_dmaTransferBase` auf die aktuelle absolute Summe gesetzt, damit die Zähler stimmig
  bleiben, und `TPUART_RP2040_TRANSFER_COUNT` wird auf ein Vielfaches der Puffergröße abgerundet,
  damit die Schreibposition im Ring nahtlos weiterläuft. Das ist wesentlich - den Lesezeiger
  zurückzusetzen würde stillschweigend jedes empfangene, aber noch ungelesene Byte im Ring
  verwerfen.
  Beachte `_dmaTransferBase = total` statt `+= TRANSFER_COUNT`: im Normalfall identisch (Zähler
  bei 0 abgelaufen), aber es übersteht auch einen Neustart, der ausgelöst wird, während der
  Zähler noch *läuft*. Die Additionsform würde den logischen Zähler vorausspringen lassen,
  während die physische Schreibposition stehen bleibt, und damit den Ringindex dauerhaft gegen
  die Wirklichkeit verschieben, sodass `read()` für immer Bytes von der falschen Stelle lieferte.
  `end()` benutzt aus demselben Grund `dma_channel_cleanup()` statt eines nackten
  `dma_channel_abort()` - ein Abort kann den Fertig-IRQ-Status des Kanals gesetzt lassen, was
  nach dem nächsten `begin()` einen unechten Neustart auslöste. Dieser Pfad läuft bei jedem
  Baudratenkandidaten.
  **Der Sendeweg läuft über einen Software-Ring** (`TPUART_RP2040_TX_BUFFER_SIZE`), nicht direkt
  in die Hardware. Zwei Gründe: `write()` rief früher `uart_tx_wait_blocking()`, was bis zu eine
  Bytezeit blockiert und in einem ISR nicht hinnehmbar ist; und die Hardware kann den mehrbytigen
  Vorlauf nicht anbieten, den `availableForWrite()` melden muss. Am PL011 schaltet
  `uart_set_fifo_enabled()` RX- und TX-FIFO gemeinsam, und der FIFO ist hier aus, das
  TX-Halteregister ist also ein Byte tief - daher der Software-Ring.
  Er ist **fest auf 4 Byte und bewusst nicht einstellbar**: mehr kann ein einzelner Tick nicht
  erzeugen - drei für ein Telegrammoktett (Offset, Position, Daten) plus eines für die Quittung
  zum gerade eintreffenden Telegramm. Alles darüber läge nur herum; die Hardware nimmt bei 19200
  Baud ohnehin ein Byte je ~0,52ms, und ein Byte, das länger im Puffer wartet, erreicht den Bus
  zu spät. Ein größerer Ring würde nur den Rückstand verstecken, den `availableForWrite()` melden
  soll.
  Das ist es, was die Vorfahrt der Steuercodes tragend macht statt kosmetisch: die längste
  Steuersequenz ist 4 Byte (`TPUART_CTRL_MAX_GROUP`), sie braucht also den Ring vollständig leer -
  und der Telegrammpfad greift sich drei Bytes, sobald drei frei sind, womit in einem 4-Byte-Ring
  nie vier frei würden. `processCtrlQueue()` liefert deshalb wahr ("der Sendeweg gehört mir in
  diesem Tick") **auch dann, wenn die Gruppe noch nicht passt**, damit der Telegrammpfad
  zurücksteht und der Ring leerläuft. Vorfahrt heißt hier Platz schaffen, nicht bloß zuerst
  dranzukommen.
  **Geleert wird der Ring von einem TX-Interrupt** (`onTxInterrupt()`), und deshalb gibt es ihn
  in dieser Form. Früher lief `pumpTx()` (davor `drainTx()`) nur aus `write()`/
  `availableForWrite()`, also nur zum Tick, und zwischen den Ticks hielten nur die zwei Bytes in
  der Hardware die Leitung beschäftigt - und je Tick passt nur *eines* nach, weil das
  Halteregister erst frei wird, wenn das Schieberegister das vorige übernommen hat. Die Messung
  dazu steht im Abschnitt zum Tick-Intervall. **Vertieft wird dadurch nichts**: der Ring bleibt
  4, die Hardware bleibt 2, das Quittungsbudget in `Transmitter.h` bleibt unangetastet. Wer
  daraus "Interrupt, also machen wir auch den FIFO an" folgert, bricht diese Zusage.
  Nebenläufigkeit: der Ring hat jetzt **zwei** Kontexte - der Tick füllt, der ISR leert - daher
  `critical_section_t _txSection`. Ein sperrfreies "ISR scharf"-Flag geht hier nicht: beide Seiten
  müssen `uart_is_writable()` *prüfen* und abhängig vom Ergebnis schreiben, und der M0+ hat dafür
  kein CAS. Es muss eine Critical Section sein und nicht `save_and_disable_interrupts()`, weil im
  `Loop1`-Modus der Erzeuger auf dem anderen Kern läuft. Ist die IRQ-Leitung schon belegt
  (arduino-pico installiert in `SerialUART::begin()` einen exklusiven Handler auf demselben UART),
  wird die Beschleunigung stillschweigend übersprungen und `TXIM` nie gesetzt - das Interface
  verhält sich dann genau wie vorher.
  **Eine Korrektur an der Begründung, die hier früher stand**: behauptet wurde, der RX-FIFO
  *müsse* aus bleiben, weil die DMA-Anforderung sonst erst an der IFLS-Schwelle käme und die
  letzten Bytes eines Frames liegen blieben. Das ist nicht belegt - das Datenblatt beschreibt
  beide Signale (§4.2.5 S. 424: `uartrxdmasreq` ab einem Zeichen, `uartrxdmabreq` ab der
  Wassermarke), sagt aber nirgends, welches als `DREQ_UARTx_RX` nach außen geführt ist, und das
  SDK schweigt ebenfalls. FIFO-Betrieb mit DMA ist ausdrücklich vorgesehen; nirgends steht eine
  Anweisung, ihn abzuschalten. Geändert wurde trotzdem nichts: die Entscheidung fürs Abschalten
  stammt aus einer Beobachtung am laufenden Bus, und sie ohne erneute Prüfung an echter Hardware
  umzudrehen ist das Risiko nicht wert. Der TX-Interrupt **beantwortet** diese Frage nicht, er
  macht sie nur weniger dringend.
- **`ESP32`** - kein DMA. Recherchiert und bestätigt: UHCI (der einzige DMA-Weg für UART auf dem
  ESP32) hat auf dem klassischen ESP32 keine offizielle ESP-IDF-Treiberunterstützung (nur
  C3/C6/S3/P4). Benutzt das übliche `uart_driver_install`; die Ereigniswarteschlange des Treibers
  wird beim Pollen nicht-blockierend geleert, um FIFO-/Pufferüberläufe zu erkennen.
  `availableForWrite()` **meldete früher `uart_get_tx_buffer_free_size()` - bis zu 512 Byte**, der
  Fehler "zu groß" in Reinform: der Treiber hätte ein ganzes Telegramm an Host-Bytes geschluckt,
  und jede Quittung dahinter wäre hoffnungslos zu spät gekommen. Der Sendepuffer des Treibers
  lässt sich nicht auf 4 verkleinern (`uart_driver_install` erlaubt nur 0 oder mehr als eine
  FIFO-Länge), er wird deshalb nicht als Puffer, sondern als Durchlauf benutzt: dieselbe
  Buchführung über die Leitung wie im `ArduinoSerial` begrenzt, was übergeben wird, und der freie
  Platz des Treibers geht als zweite Schranke ein, damit `uart_write_bytes()` trotzdem nie wartet.
  **ISR-tauglich ist es weiterhin nicht**: `uart_write_bytes()` nimmt den TX-Mutex des Treibers.
  Heute unbestritten (ein einziger Schreiber), aber `tick()` auf dem ESP32 aus einem Interrupt zu
  treiben bräuchte einen eigenen Ringpuffer plus einen FreeRTOS-Task.
  **Die Pins sind hier die Falle, und sie haben einen Abend gekostet.** `uart_set_pin()` prüft
  nicht, ob ein Pin frei ist - es legt die Pin-Matrix um, und wenn dort der eingebaute Flash oder
  PSRAM hängt (ESP32-PICO-V3-02: GPIO 6-11 **sowie 16 und 17**; WROVER: 16/17), findet der nächste
  Cache-Miss keinen Flash mehr. Der Kern bleibt stehen, und da der Panic-Handler selbst erst aus
  dem Flash gelesen werden müsste, **kommt kein einziges Zeichen heraus**: nach 300ms setzt der
  Interrupt-Watchdog stillschweigend zurück (`rst:0x8 TG1WDT_SYS_RESET`). Ein Bootloop ganz ohne
  Ausgabe auf dem ESP32 heißt "zuerst die Pins prüfen" - im Quelltext ist nichts Falsches zu
  finden. Erschwerend kommt hinzu, dass der hängende Aufruf der unverdächtige ist:
  `uart_param_config()` kehrt zurück, `uart_driver_install()` meldet brav `rc=0`, und
  **`uart_set_pin()` kommt nie wieder**. Das frühere Diagnosewerkzeug hatte 16/17 fest verdrahtet,
  weil das die Arduino-Vorgabepins für `Serial2` sind (auf einem WROOM-32 auch richtig); die
  Verdrahtung des Entwicklungsboards ist **RX 37 / TX 5**. Die beiden Rollen sind nicht tauschbar:
  GPIO 34-39 sind beim klassischen ESP32 reine Eingänge, 37 kann also nur RX sein - vertauscht
  bleibt der Sendeweg stumm, ohne dass irgendwo ein Fehler auftaucht. Ein Aufrufer muss also Pins
  übergeben, die auf seinem Modul tatsächlich frei sind; der Konstruktor nimmt sie entgegen, und
  der Kommentar an `uart_set_pin()` beschreibt die Falle, weil die Quelle selbst nichts zeigt.
  Was daran über den Einzelfall hinausgeht: jene Env hatte monatelang warnungsfrei gebaut, ohne je
  auf echter Hardware zu laufen, und der Fehler war durch Lesen nicht zu finden - im Quelltext
  steht nichts Falsches, die Zahl 16 ist erst auf diesem Modul verkehrt. Genau dafür ist die Zeile
  "nur kompiliert, nie gelaufen" in den offenen Punkten da.
- **`ArduinoSerial<T>`** - allgemeiner Wrapper um beliebige `Stream`-artige Arduino-Klassen (z.B.
  `HardwareSerial`). Ein Template, damit es auch mit Kernen funktioniert, bei denen `Serial1`
  nicht wörtlich `HardwareSerial` heißt (z.B. arduino-pico) - daher das Muster
  `ArduinoSerial<decltype(Serial1)>`, das `test/test_tpuart/interface_check.cpp` auch ausdrücklich
  instanziiert.
  **`availableForWrite()` ist der schwierige Teil dieses Adapters, und er hat beide Fehler
  gemacht.** Zuerst zu klein: durchgereicht, was die Serial-Klasse meldet.
  `Print::availableForWrite()` liefert als Vorgabe 0, und `SerialUART` von arduino-pico liefert
  `(uart_is_writable(_uart)) ? 1 : 0`, also höchstens 1. Eine 4-Byte-Steuersequenz bekam damit nie
  ihren Platz, blieb am Kopf der Steuerwarteschlange stehen und blockierte den Telegrammpfad
  gleich mit. Beobachtet auf dem RP2040 als **Dauerlauf von `CTRL OVERFLOW`**, sobald eine eigene
  Adresse gesetzt war - `U_SetAddress.req` ist die erste Sequenz, die mehr als ein Byte braucht,
  weshalb dieser Adapter bis dahin unauffällig aussah.
  Zu groß ist der unsichtbare Gegenfehler: `SerialUART::write()` ruft `uart_putc_raw()` direkt in
  die Hardware, und `uart_init()` des SDK lässt den **TX-FIFO an - 32 Byte tief**,
  `uart_is_writable` bleibt also wahr, bis 32 Bytes drinliegen.
  Beides löst dieselbe Buchführung: Bytezeit aus der Baudrate, jedes übergebene Byte schreibt
  `_lineBusyUntil` fort, und was noch aussteht, folgt aus der Uhr. Gemeldet wird der Platz bis zur
  erlaubten Tiefe.
  **Dazu ein vier Byte tiefer eigener Sendepuffer**, und hier stand einmal das Gegenteil ("kein eigener
  Ring - die FIFO darunter ist der Ring"). Das widersprach der Anforderung in `Transmitter.h`, die einen
  solchen Puffer ausdrücklich verlangt, und es ging schief: `availableForWrite()` ist eine RESERVIERUNG,
  deren Einhaltung ohne Puffer an der eingepackten Klasse hängt - und die kann ablehnen, ohne dass diese
  Ebene den Grund kennt. `SerialPIO` tut es, wenn seine `CoreMutex` belegt ist (Reentranz zwischen Tick und
  Hauptkontext). Weil der Aufrufer Gruppen unteilbar absetzt, lag dann eine halbe Sequenz auf der
  Hostleitung: am Bus gemessen meldete der Chip `PE` in Serie und liess etwa jedes zwanzigste Telegramm
  unbestaetigt, jedes davon 10s Wachhund plus BCU-Reset. Gezaehlt hat es nichts - kein Überlauf, kein
  Verlust, keine Meldung. Der Puffer nimmt jetzt an, was zugesagt war, und schiebt es nach; die erlaubte
  Tiefe bleibt dieselbe, weil "noch bei uns" und "geschätzt noch unter uns" zusammen gezählt werden. Vor
  einer zugelassenen Quittung liegen damit nie mehr als drei Bytes, die 2,8ms-Frist bleibt also gedeckt.
  **Der Sendeweg dieses Adapters ist damit erstmals an echter Hardware gelaufen** - über `SerialPIO` in
  `tpbridge`; empfangen hatte er dort vorher schon fehlerfrei. Ob die eingepackte Klasse überhaupt freien Sendeplatz meldet, wird **in
  `begin()`** festgestellt (unmittelbar danach ist der Sendepuffer leer, eine funktionierende
  Implementierung muss dort also mehr als 0 melden); tut sie es nicht, bremst allein die Uhr.
- **`Dummy`** - simuliertes Interface für Tests, ohne echte Hardware (aber trotzdem über das
  Arduino-Framework gebaut, nicht nativ - siehe "Build-Umgebungen").
  **Es liegt nicht in `src/`, sondern in `test/test_tpuart/`**, neben den Tests, die sein einziger
  Nutzer sind. Ein Testdoppel in der Library wäre totes Gewicht in jeder Produktionsfirmware und
  bräuchte eine `#ifdef`-Klammer, die jemand vergessen kann; aus dem Testordner heraus ist es
  zugleich der Beleg, dass `Interface::Abstract` von außen implementierbar ist, genau wie ein
  Aufrufer es täte.
  Empfangene Bytes werden einzeln über `addByte(char data, uint32_t pauseUs)` eingereiht. Die
  Pause ist in **Mikrosekunden**, nicht Millisekunden, und sie ist selbst bedeutungstragende
  Angabe - der Abstand zwischen Bytes wird von der Protokollschicht ausgewertet (Zeitüberschreitung,
  Frame-Ende-Erkennung), Tests müssen ihn also je Byte genau setzen können. `overflow()` ist
  einmalig; Bytes, die die Schicht darüber schreibt (z.B. `U_Ackn.req`), werden mitgeschrieben und
  sind über `writtenBytes()` einsehbar. `setWriteCapacity()` täuscht einen beengten Sendepuffer
  vor, damit Tests den Fall "nicht genug Platz für eine zusammengehörende Bytegruppe" abdecken
  können - und die Kapazität *schrumpft* tatsächlich mit jedem `write()`, freigegeben wird sie
  wieder von `drainWritten()`. Ohne das könnte ein Aufrufer, der mehr Bytes schreibt als er
  reserviert hat, in keinem Test scheitern. `available()` zählt *jedes* Byte, dessen Ankunftszeit
  bereits vorbei ist, nicht nur das nächste - ohne das ließe sich "die Verarbeitung ist in
  Rückstand geraten und das Telegramm liegt schon vollständig im Puffer" in keinem Test
  nachstellen, und genau das treibt die Quittungsunterdrückung.
  **`begin()` spult das Skript bewusst nicht zurück.** Echte Interfaces verwerfen über ein
  `end()`/`begin()`-Paar hinweg alles Gepufferte. `_pos` zurückzusetzen würde die Warteschlange ab
  Byte 0 erneut abspielen - und da `searchBaudRate()` je Baudratenkandidat ein `end()`/`begin()`
  macht, sähe die Erkennungslogik bei jedem Versuch dasselbe `U_Reset.ind` wieder, eines, das auf
  echter Hardware längst weg wäre. Das ist falsches Zutrauen für genau den Teil, den der Anwender
  als den heiklen benannt hat. Zum absichtlichen Neuscharfmachen eines Skripts gibt es
  `clearData()`.
