# Statistics.md - Statistik, Taktmessung, Buslast

Teil von `AGENTS.md`.

- **`Statistics`** (`Statistics.{h,cpp}`, erreichbar über `getStatistics()`). Die Zähler werden aus
  `tick()` hochgezählt und aus dem Hauptkontext gelesen, daher
  `volatile` und einfache Inkremente - ein Schreiber je Zähler. `reset()` aus dem Hauptkontext kann
  mit einem Inkrement kollidieren; für eine Statistik wird das hingenommen statt gesperrt.
  **Das Namensschema ist verbindlich**: Richtungspräfix immer (`getRx…`/`getTx…`), die Einheit steht
  im Namen (`…Frames` zählt Telegramme, `…Bytes` zählt Bytes, `…Overflows`/`…Losses` zählen
  Ereignisse), nichts wird abgekürzt. Wer einen Zähler ergänzt, hält sich daran - sonst ist in einem
  Jahr wieder unklar, ob ein `getInterfaceOverflows` den Empfang oder den Versand meint (es war der
  Empfang, daher heißt er heute `getRxInterfaceOverflows`).
  **Eine Zahl, ein Name.** `getRxFrameBytes()` und `getRxBusBytes()` lieferten dasselbe Feld unter
  zwei Namen - genau die Doppelung, die diese Klasse an ihrem Vorgänger kritisiert. `getRxFrameBytes()`
  ist die echte Methode, `getRxBusBytes()` nur noch KOMPAT.
  **Der KOMPAT-Block enthält ausschließlich Namen, die es in v1 schon gab und die ein Verbraucher
  aufruft.** Neue Namen bekommen dort *nichts*: solange 2.0 in Arbeit ist, kann sich auf sie noch
  niemand stützen, sie dürfen also direkt heißen, wie sie heißen sollen. Nicht nachgerüstet werden die
  drei ungenutzten Dubletten aus v1 (`getRxOverflowInterface`, `getRxOverflowFrameBuffer`,
  `getRxOverflowSearchBuffer`) - dort gab es jedes dieser Ereignisse unter zwei Namen, und die
  Verbraucher rufen durchweg nur die eine Schreibweise.
  **Gesendete Telegrammbytes gibt es bewusst nicht als eigenen Zähler**, sie sind ableitbar - aber
  nicht so einfach, wie es aussieht: die Quittung geht ebenfalls über `Transmitter::writeByte()` und
  steckt damit in `getTxBytes()`. Richtig ist
  `getTxBytes() - getTxControlBytes() - getTxAcknowledges()`, denn eine Quittung ist genau ein Byte.
  **Ereignis und Umfang sind zwei Fragen.** `getRxResyncs()` zählt, wie oft die Position im Bytestrom
  verloren ging, `getRxDroppedBytes()` was es gekostet hat - drei Resyncs zu je 5 Byte und einer zu 15
  ergeben dieselbe Byte-Zahl bei völlig verschiedenem Befund. Gezählt wird an der einen Stelle, an der
  alle Wege in den Resync zusammenlaufen (`resetSequence()`); doppelt zählt dabei nichts, weil
  `forceResync()` bei bereits laufendem Resync früh umkehrt.
  **Die Fehler, die der Chip selbst meldet, werden einzeln gezählt** - `getChipSlaveCollisions()`,
  `getChipReceiveErrors()`, `getChipTransmitErrors()`, `getChipProtocolErrors()`,
  `getChipTemperatureWarnings()`. Nötig ist das, weil `_stateErrors` die Bits aus `U_State.ind` nur
  ODER-akkumuliert und beim Ausgeben löscht: der Chip meldet jedes Ereignis genau einmal, "einmal vor
  Stunden" und "dauernd" sähen dort gleich aus. Einzeln statt als Summe, weil es fünf verschiedene
  Diagnosen sind - eine stehende Übertemperaturwarnung ist etwas anderes als gelegentliche Kollisionen.
  Ein gemeinsamer Zähler über alle fünf fehlt **bewusst** - er sagte nichts, was diese hier nicht
  besser sagen, und wäre nachträglich auch nicht zu bilden, weil ein `U_State.ind` mehrere Bits zugleich
  tragen kann. Die Zuordnung Bit → Zähler steht im `DataLinkLayer`, nicht in `Statistics` - dafür
  braucht es die Protokollkonstanten, und die Zählerklasse soll keine kennen. Kein `Rx`/`Tx`-Präfix:
  das sind Zustände des Chips, keine Richtung von uns aus (TE und SC entstehen beim Senden auf dem Bus,
  RE beim Empfangen - ein gemeinsames Präfix wäre in jedem Fall falsch).
  **Höchststände statt nur Überläufe.** `getRxQueuePeakBytes()`, `getTxControlQueuePeakBytes()` und
  `getTxQueuePeakBytes()` beantworten die Auslegungsfrage, *bevor* etwas überläuft - ein
  Überlaufzähler sagt nur, dass es zu spät war. Sie hängen sich an einen Füllstand an, der an der
  jeweiligen Stelle ohnehin ausgerechnet wird, es kommt also nur ein Vergleich dazu. Alle drei zählen
  **Bytes**; die Sendequeue tat das früher in Telegrammen, seit sie ein Bytepuffer ist, misst sie sich
  wie die beiden anderen. Zu lesen sind sie gegen `TPUART_RX_QUEUE_SIZE`, `TPUART_CTRL_QUEUE_SIZE` und
  `TPUART_TX_BUFFER_SIZE`.
  **Gezählt werden Probleme, nicht Vorgänge.** Deshalb gibt es `getConnectionLosses()` und
  `getTxConfirmTimeouts()`, aber keinen Zähler für Resets und keinen für negative Bestätigungen: ein
  negatives `L_Data.con` sagt nur, dass auf dem Bus niemand quittiert hat - eine Aussage über den Bus,
  kein Fehler der Strecke. `getTxConfirmTimeouts()` zählt dagegen den Fall, in dem **überhaupt keine**
  Bestätigung kam und der Wachhund die BCU zurücksetzen musste; das zeigt auf den Chip oder die
  Verkabelung. Resets zu zählen wurde verworfen, weil allein die Baudratenerkennung mehrere schickt -
  ohne Ursache ist die Zahl wertlos.
  **Der Unterschied, auf den es ankommt: kaputte Telegramme und verworfene Bytes sind nicht
  dasselbe** und werden getrennt gezählt. Ein kaputtes Telegramm *wurde gemeldet* (mit
  `TP_FRAME_FLAG_INVALID` - CRC-Fehler, von einer Pause abgeschnitten, beschädigtes Längenoktett), der
  Verbraucher hat es also gesehen. Verworfene Bytes hat nie jemand gesehen, und dafür gibt es drei
  Quellen: alles, was während `Resync` verbraucht wurde; die Reste einer von einem Moduswechsel
  abgebrochenen Sequenz; und ein fertiger Eintrag, für den im RX-Ring kein Platz mehr war. Ein
  gemeinsamer Zähler "verworfen" für kaputte Telegramme und verlorene Bytes wäre unbrauchbar - führe
  ihn nicht ein.
  **Die Kategorien sind keine Zerlegung**: ein mangels Ringplatz verworfenes Telegramm steht mit
  seinen Bytes auch in `getRxFrameBytes()`, denn über den Bus kam es, und daran hängt die Buslast.
  `getTxAcknowledgesSuppressed()` lohnt die Beobachtung: er zählt den Fall "wir liegen hinter dem Bus",
  was korrektes Verhalten ist und kein Fehler - ein steigender Wert heißt aber, dass der Tick nicht oft
  genug drankommt. **Er ist dabei nicht durch Fremdverkehr aufgebläht**: `sendAcknowledge()` fragt den
  Quittungs-Callback *vor* der Rückstandsprüfung, ein abgelehntes Telegramm kommt also gar nicht bis
  dorthin. Ohne diese Reihenfolge zeigte ein Gerät ohne eigene Adressen Rückstände in Höhe des gesamten
  Busverkehrs an, und ein echter Rückstand wäre darin nicht mehr zu finden - festgehalten in
  `test_unaddressed_frame_is_not_acknowledged`, das den Fall bewusst *mit* vollem Rückstand fährt.
- **Der Takt misst sich selbst** (`getTicks()`, `getTickDeferrals()`, `getTickLastDeferredUs()`,
  `getRxInterfacePeakBytes()`), und
  das ist die Antwort auf ein wiederkehrendes Problem: der Antrieb ist die Voraussetzung für alles in
  dieser Schicht, war aber der einzige Wert, den man **nicht ablesen konnte**. Sichtbar waren nur seine
  Folgen - unterdrückte Quittungen, Interface-Überläufe -, und die Ursache musste geraten werden.
  Die Werte trennen verschiedene Krankheitsbilder, und erst zusammen ergeben sie eine Diagnose:
  - `getTicks()` gegen die Laufzeit ist die **mittlere** Rate. Deutlich unter dem eingestellten Soll
    heißt: der Timer treibt diese Instanz gar nicht, der Hauptloop tickt - dasselbe sagt `usesTimer()`.
  - `getTickDeferrals()` sagt, **wie oft** der Tick aufgehalten wurde (Schwelle
    `TPUART_TICK_DEFERRED_US`, die Zeichenzeit des Busses), `getTickLastDeferredUs()`, **wie lang die
    letzte** dauerte. Ein guter Mittelwert bei vorhandenen Verzögerungen heißt: der Antrieb stimmt, wird
    aber zwischendurch blockiert.
    **Bewusst die letzte und nicht die schlimmste.** Ein Höchstwert SÄTTIGT - nach einem einzigen
    Flash-Schreibvorgang stünde dort für immer eine fünfstellige Zahl, und er sagte danach nichts mehr
    über den aktuellen Zustand. Es gab dafür einmal `getTickGapMaxUs()`; der Wert ist aus genau diesem
    Grund entfernt worden. Wer wissen will, wann es passiert, liest den Zähler vor und nach der
    verdächtigen Aktion ab.
  - `getTickDurationMaxUs()` und `getCheckAcknowledgeMaxUs()` beantworten die **umgekehrte** Frage: nicht,
    wie oft der Tick aufgehalten wurde, sondern wie lange er selbst braucht. Daran hängt die
    Prioritätswahl - wer andere Interrupts verdrängen will, muss belegen können, dass er sie nur kurz
    aufhält. Gemessen wird nur der volle Durchlauf; die frühen Abbrüche sind ein paar Vergleiche und
    würden das Bild beschönigen. Die zweite Zahl ist der Anteil des **Aufrufers**: der Quittungs-Callback
    ist der einzige unbegrenzte Teil des Ticks (alles andere ist ein Byte je Richtung), und liegen beide
    dicht beieinander, gehört die Laufzeit ihm und nicht dieser Schicht. Beides sind Höchstwerte und
    sättigen deshalb - anders als bei `getTickLastDeferredUs()` ist das hier richtig, weil die Frage
    "wie schlimm kann es werden" lautet und nicht "wie steht es gerade".
    **"Wie lange er selbst braucht" ist dabei nicht dasselbe wie "wie viel er tut"**: ein Stillstand, der
    einen bereits laufenden Tick erwischt, verlängert dessen Dauer, ohne dass eine einzige zusätzliche
    Anweisung liefe. Am Gerät gemessen: 46184µs bei 378µs Callback-Anteil. Der Mechanismus steht beim
    Flash-Schreibvorgang weiter unten; hier zählt die Lesart - ein großer Wert ist erst dann teure Arbeit
    dieser Schicht, wenn `getTickDeferrals()` daneben bei 0 steht **und** kein Stillstand in Frage kommt.
  - `getRxInterfacePeakBytes()` ist derselbe Befund in der Einheit, in der er entsteht: 0-1 ist gesund
    (der Bus liefert höchstens alle 1,354ms ein Byte, der Tick holt alle 500µs eines ab), ab 2 wird bei
    Byte 6 die Quittung unterdrückt. Kostenlos erhoben, weil `Receiver::process()` `available()` ohnehin
    fragt.
  Die Grenze, die zählt, ist **~2700µs**: darüber ist ein Standardtelegramm vollständig eingetroffen,
  bevor der Tick bei Byte 6 die Entscheidung trifft.
  `_tickLastUs` gehört dem Tick und wird in `begin()` zurückgesetzt - die Pause zwischen `end()` und
  `begin()` ist kein Aussetzer des Antriebs, und ohne das Zurücksetzen stünde sie für immer als
  Höchstwert da, ausgerechnet auf einem Gerät, das die BCU neu verbindet (`test_tick_gap_survives_restart`).
  `DataLinkLayer::tickInterval()` liefert dazu das **eingestellte** Soll. Die drei Auskünfte sind
  bewusst getrennt: `usesTimer()` sagt, *wer* tickt, `Timer::interval()`, wie schnell es *gedacht* war,
  und `getTicks()`, was daraus *geworden* ist.
- **Und der Antrieb meldet sich selbst, wenn er die Untergrenze reißt** (`checkTickRate()`, aus `loop()`,
  Fenster `TPUART_TICK_RATE_WINDOW_MS` = 2000). Das ist die Lehre aus dem Fall, der diese ganze Messung
  ausgelöst hat: im Router lag die Rate bei 457/s statt 2000/s, und **sichtbar war davon ausschließlich der
  Folgeschaden** - 12% unterdrückte Quittungen und ein Rückstand von 7 Byte. Die Ursache stand nirgends;
  sie war nur auf ausdrückliche Nachfrage über `usesTimer()` zu erfahren, und danach fragt im Betrieb
  niemand. Nach dem Umschalten auf den eigenen Timer: 0 unterdrückte Quittungen, unverändert am
  Protokollpfad.
  Zwei Entwurfsentscheidungen daran sind tragend:
  - **Gemessen wird die ERREICHTE Rate, nicht die eingestellte.** Ein Wächter auf `usesTimer()` hätte
    genau den Fall verfehlt, in dem jemand absichtlich selbst tickt (`Timer::trigger()`, eigener Task) und dabei zu langsam
    ist - und das war hier fast der Fall.
  - **Die Schwelle kommt aus dem Bus, nicht aus der Konfiguration**: `TPUART_TICK_DEFERRED_US` ist 1354, die
    Zeichenzeit auf TP1. Ein Tick bewegt ein Byte je Richtung; liegt der mittlere Abstand darüber, holt die
    Schicht weniger Bytes ab, als der Bus im Vollausbau liefert (738 Byte/s), und **keine Puffergröße hilft
    dagegen**. Unterhalb ist alles gut, oberhalb ist es grundsätzlich kaputt - das ist keine Empfehlung,
    sondern eine Untergrenze.
  Gemeldet wird **einmal je Störung**, nicht je Fenster; erholt sich die Rate, wird der Melder wieder
  scharf. Der Fall "gar kein Tick im Fenster" ist ausdrücklich behandelt, er wäre sonst eine Division durch
  null. Zwei Testfälle hängen daran, und der zweite ist der wichtigere:
  `test_deferred_tick_is_reported` und `test_healthy_tick_is_not_reported` - ein Wächter, der grundsätzlich
  meldet, wäre ohne den zweiten genauso "grün" wie der richtige und im Betrieb reines Rauschen.
  **Ein guter Mittelwert schließt das Problem nicht aus**, und im Router steckten dahinter *zwei*
  verschiedene Ursachen, die sich im blossen Höchstwert nicht unterscheiden ließen - erst
  `getTickDeferrals()` hat sie getrennt:
  - **Dauerhaft, 2-4 mal je Sekunde**: ein gleichrangiger Interrupt auf 0x80. Behoben durch den eigenen
    Alarmpool mit angehobener Priorität (siehe `Timer`), Ergebnis 0 Verzögerungen über 26000 Ticks bei einem
    Höchstwert von 520µs gegen 500µs Intervall.
  - **Einmalig und riesig**: ein Flash-Schreibvorgang. **Auf BEIDEN Plattformen gemessen** - RP2040
    53176µs, ESP32 40033µs -, es ist also keine Eigenheit des einen Ports. Der Mechanismus unterscheidet
    sich, die Wirkung nicht: der RP2040 sperrt über `noInterrupts()`/`idleOtherCore()` das XIP (PRIMASK
    wirkt unabhängig von jeder IRQ-Priorität, und `idleOtherCore()` parkt Kern 1 gleich mit - ein Tick
    dort wäre also ebenfalls betroffen), beim ESP32 wird der Instruction-Cache abgeschaltet und der
    `esp_timer`-Task kann nicht aus dem Flash laufen. Aus der Library heraus ist beides nicht behebbar.
    **Die Dauer, die die Anwendung für den Speichervorgang meldet, ist NICHT der Stillstand** - sie
    beantwortet eine andere Frage, und beide Zahlen sind für sich richtig. Gemeldet wird die Dauer des
    integritätskritischen Fensters: so lange muss die Stromversorgung bei einem Ausfall überbrücken,
    damit die Daten vollständig geschrieben sind. Auf dem RP2040 wird der Schreibbereich VORAB gelöscht
    und der Erase des alten Blocks läuft danach - das kritische Fenster ist damit kurz ("2ms"), der
    Tick-Stillstand umfasst aber auch den nachgelagerten Erase (53176µs). Auf dem ESP32 fallen beide
    zusammen ("47ms" gegen 40033µs), weil der Erase dort im Schreibvorgang steckt.
    Für die Auslegung von Fristen dieser Schicht zählt der Stillstand, nicht die gemeldete Schreibdauer.
    **ER ERSCHEINT IN ZWEI VERSCHIEDENEN ZAHLEN, je nachdem, wann er zuschlägt**, und hier stand lange nur
    die eine. Fällt er zwischen zwei Ticks, ist es eine Verzögerung und zählt in `getTickDeferrals()` -
    das ist der oben beschriebene Fall. Fällt er, während ein Tick schon LÄUFT, verlängert er diesen Tick
    und landet in `getTickDurationMaxUs()`: `noInterrupts()` hält einen bereits laufenden Handler nicht an,
    aber beim nächsten Befehlsabruf steht er, bis XIP zurück ist. Am Gerät gemessen: **46184µs Tickdauer**
    bei unveränderten 378µs im Quittungs-Callback, also nichts davon eigene Arbeit. Wer `Max run` liest,
    muss das wissen - ein vierstelliger Wert dort ist nicht automatisch teurer Code in dieser Schicht, und
    weil es ein Höchstwert ist, bleibt er danach für immer stehen.
  **Was ein solcher Stillstand kostet, ist für Puffer und Quittung VERSCHIEDEN**, und die Messung oben
  zeigte nur deshalb keinen Schaden, weil der Bus fast leer war - das ist kein Freibrief:
  - **Der Empfangspuffer hält, und das ist rechenbar.** Die DMA ist eigenständig und füllt den Ring
    weiter, während die CPU steht; sie braucht keine Interrupts. 52ms bei voller Buslast (738 Byte/s)
    sind 38 Byte gegen einen 256-Byte-Ring - überlaufen würde er erst bei rund 350ms.
  - **Die Quittung hält nicht - und mehr als sie kann auch nicht verlorengehen.** Jedes Telegramm, dessen
    Byte 6 in das Fenster fällt, verliert seine Quittung; bei voller Last sind das drei bis vier je
    Stillstand. Dass in der Messung keines betroffen war, lag daran, dass nur 3% des Verkehrs an dieses
    Gerät gerichtet waren und der Bus ruhig lag - Glück, kein Verdienst.
    **Das Telegramm selbst geht dabei nicht verloren**: es liegt im Ring, wird nach dem Stillstand
    zerlegt und ausgeliefert, und die Wiederholungen des Absenders kommen ebenfalls an (als `FILTERED`
    markiert). Die Kosten liegen beim ABSENDER - drei Wiederholungen Busbandbreite und eine Übertragung,
    die er für gescheitert hält, obwohl sie ankam. Empfangsverlust setzt einen Ringüberlauf voraus, und
    der bräuchte die oben gerechneten ~350ms.
  Zugleich ist es der Beleg dafür, dass die Pausenerkennung richtig aufgehängt ist: `_emptySince` misst ab
  der ersten Beobachtung von Stille, ein 52ms-Stillstand des Ticks wird also nicht als Buspause
  missdeutet.
  Der Wächter schweigt zu beidem zu Recht, sobald der Mittelwert stimmt - dafür gibt es
  `getTickDeferrals()` und `getTickLastDeferredUs()` daneben.
  `getBusLoad()` hat die Einheit Byte je Sekunde und beschreibt **genau ein Messintervall** von
  `BUS_LOAD_INTERVAL_MS` (1000) - einen gleitenden Mittelwert gibt es bewusst **nicht** (Entscheidung
  des Anwenders): der Wert soll die letzte Sekunde beschreiben und nicht ein Mittel über mehrere. Die
  Konstante ist ein festes `static constexpr`-Member, kein überschreibbares Makro - sie beschreibt eine
  Anzeige, keine Betriebsart. Der Preis ist bekannt und hingenommen: ein einzelnes 263-Oktett-Telegramm
  belegt den Bus 356ms, die Anzeige springt dadurch stärker als der Bus sich ändert. Wer glätten will,
  mittelt über mehrere dieser fertigen Werte, statt das Fenster zu verbreitern.
  **Gerechnet wird beim MESSEN, nicht beim Ablesen**: `sampleBusLoad()` läuft aus `loop()`, hält
  Zählerstände plus Zeitstempel als Bezugspunkt und schließt die Sekunde ab, sobald das Intervall voll
  ist; `getBusLoad()` liefert das fertige Ergebnis, beliebig oft und ohne es zu verändern. Ein
  Bezugspunkt, der bei jedem Lesen mitzöge, ergäbe Spannen von Millisekunden - ein einzelnes Telegramm
  darin sähe wie ein völlig überlasteter Bus aus. Geteilt wird durch die *gemessene* Spanne, nie durch
  die angenommenen 1000ms, ein verspäteter Hauptloop verfälscht also nichts.
  Was hineinfließt, ist bewusst eng gefasst: **nur Telegrammbytes, Polls eingeschlossen**. Steuerbytes
  kommen von der BCU, nicht vom Bus, und gehören zur Auslastung der Host-Leitung; im Resync verworfene
  Bytes lassen sich nichts zuordnen.
  **`getBusLoadPercent()` ist NICHT dieselbe Zahl in anderer Einheit**, und der Unterschied ist der
  eigentliche Punkt: es ist eine **Zeitrechnung**, nicht ein Verhältnis zur Bytekapazität.
  ```
  belegt = Oktetts × BUS_OCTET_TIME_US (1354)
         + Telegramme × (BUS_FRAME_GAP_US (5208) + BUS_ACK_SLOT_US (2708))
  ```
  **Beide Zuschläge sind FESTE SLOTS**, und daran hängt, dass die Rechnung ohne Kenntnis des Verkehrs
  auskommt. Beim Quittungsslot ist das der entscheidende Punkt: er ist reserviert, nicht bedingt - ob
  jemand quittiert oder nicht, die Zeit vergeht und niemand sonst kann senden. Es muss deshalb *nicht*
  bekannt sein, welche Telegramme quittiert wurden, was der Host außerhalb des Busmonitors auch gar
  nicht wissen kann. 15 Bitzeiten Abstand plus das Quittungsoktett (11 Bit) sind 26 Bitzeiten = 2708µs;
  bleibt die Quittung aus, wartet der Sender die dokumentierten 30 Bitzeiten ab (S. 38) - 417µs mehr,
  was keine Fallunterscheidung wert ist.
  Hier stand zwischenzeitlich, die Quittung sei "nicht zählbar" und der Wert unterschätze die Belegung
  deshalb um rund 13%. Das war falsch, und der Fehler war die Frage: gefragt war nicht, *ob* quittiert
  wurde, sondern nur, *wie lang der Slot ist*.
  **UNABHÄNGIG BESTÄTIGT durch die geläufige Kennzahl "der Bus schafft rund 50 Telegramme je Sekunde".**
  Sie fällt aus dieser Rechnung genau heraus - und nur, wenn alle drei Anteile drinstehen:
  ```
  9 × 1354µs (Oktetts) + 5208µs (Pause) + 2708µs (Quittungsslot) = 20102µs  ->  49,7 Telegramme/s
  ```
  Ohne den Quittungsslot kämen 57,5/s heraus, ohne die Pause 67/s - beides passt nicht. Das ist der
  einzige Prüfstein von außen, den diese Rechnung hat, und er trifft. Zugleich ist die Anzeige damit
  kalibriert: 50 minimale Telegramme je Sekunde sind genau 100%, und bei kleinen Telegrammen liest sich
  der Prozentwert direkt als "Telegramme je Sekunde geteilt durch 50".
  Deshalb steht in der Messprobe neben dem Byte- auch der **Telegrammzähler** (heile plus kaputte - der
  Bus war in beiden Fällen belegt). Aus den Oktetts allein ginge es nicht: 270 Oktetts sind ein großes
  Telegramm oder dreißig kleine, und die dreißig belegen den Bus 156ms länger, weil vor jedem Telegramm
  50 Bitzeiten frei sein müssen.
  Das ist auch der Grund, warum der Nenner **nicht** die rohe Kanalkapazität ist (738 Oktetts/s). Gegen
  die gerechnet wäre 100% unerreichbar, und zwar unterschiedlich weit: ~61% bei 9-Oktett-Telegrammen,
  ~98% bei maximalen - eine Zahl, deren Obergrenze vom Verkehr abhängt, taugt nicht als Füllstand. Über
  die Zeit gerechnet heißt 100% dagegen wirklich "hier passt kein Telegramm mehr hinein".
  **Abgeschnittene Telegramme gehen vollwertig ein**, und das ist kein Zufall: gezählt werden die Bytes,
  die tatsächlich ankamen (`length` am Sequenzende, nicht die Sollgröße), und da ein Fragment als
  `INVALID` gemeldet wird, zählt es auch bei den Telegrammen mit. Ein nach drei Oktetts abgebrochenes
  Telegramm bekommt also 3 × 1354µs plus einmal die Pause - der Bus war für beides belegt.
  **Bytes aus einem Resync fehlen dagegen**, sie laufen nach `incrementRxDroppedBytes()` und tauchen in
  `getRxFrameBytes()` nie auf. Bewusst nicht nachgerüstet (Entscheidung des Anwenders): ein Resync ist so
  selten, dass es die Zahl nicht bewegt, und wenn er häufig wird, zeigt `getRxResyncs()` das deutlicher
  an. **Den Dropped-Zähler dafür einfach zu addieren wäre falsch** - er fasst drei Quellen zusammen, und
  eine davon ist bereits gezählt: ein fertiges Telegramm ohne Platz im RX-Ring erhöht *beide* Zähler.
  Wer den Resync-Anteil doch will, braucht dafür einen eigenen Zähler.
  **Über 100 wird bewusst NICHT gedeckelt** (Entscheidung des Anwenders), und der Rückgabetyp ist deshalb
  16 Bit - ein `uint8_t` liefe bei 256% still über und meldete 0. Der Bus kann nicht voller als voll sein,
  ein Wert über 100 ist also ein Befund und soll sichtbar sein statt weggerundet.
  `test_bus_load_percent_is_not_capped` hält das fest, damit es niemand als offensichtlichen Fehler
  "repariert".
  **Damit das trägt, werden die Telegrammbytes JE BYTE gezählt** (`processFrameByte()` /
  `processPollByte()` / `processControlByte()`), nicht mehr am Sequenzende in einem Rutsch. Der alte Weg
  hatte einen echten Zuordnungsfehler: ein laufendes Telegramm trug 0 bei und brachte beim Abschluss seine
  gesamte Busbelegung mit, auch den Teil, der vor dem Fenster lag - bei einem maximalen Telegramm 356ms,
  also über ein Drittel der Messsekunde. Das hätte ein "über 100" erzeugt, das nichts bedeutet, und damit
  genau die Aussage zerstört, für die der Deckel weggelassen wurde. Die Kategorie steht dabei in jedem Pfad
  schon mit Byte 0 fest, und der Zähler hat weiterhin genau einen Schreiber (den Tick) - es braucht also
  weder Klassifikation im Nachhinein noch Atomarität. Achte auf die Reihenfolge in
  `processControlByte()`: der Zähler steht **hinter** dem Poll-Zweig, weil der sein Byte selbst als
  Telegrammbyte zählt.
  Als Rest bleibt der Zuschlag **je Telegramm**, der weiterhin erst beim Abschluss fällig wird - vorher
  steht nicht fest, ob das Telegramm heil ist. Für ein Telegramm an der Fenstergrenze sind das 7916µs,
  bei 1s also 0,8%.
  Früher legte der Leser selbst das Fenster fest: das Intervall war "Zeit seit dem letzten Aufruf", was
  von der Konsole kommt und damit unbegrenzt ist - die erste Anzeige nach Stunden Laufzeit mittelte
  über die gesamte Laufzeit, und der Zwischenwert (Bytes × 1000) lief über 32 Bit über. Eine Messung
  nach einer Lücke von mehr als zehn Intervallen wird verworfen statt benutzt, da sie nichts über die
  aktuelle Last aussagt.
