# DataLinkLayer.md - DataLinkLayer: Aufbau, Tick/Loop, Zustände, Nebenläufigkeit

Teil von `AGENTS.md`.

## Aktueller Stand (DataLinkLayer)

`src/TPUart/Types.h` + `DataLinkLayer.{h,cpp}` + `Receiver.{h,cpp}` +
`Transmitter.{h,cpp}` setzen die Zustandsmaschine auf Byteebene um, aufgesetzt auf
`Interface::Abstract`.

**Die Teilung in `Receiver`/`Transmitter`** trägt sich über die Art, wie die beiden verbunden
sind: jeder hält eine `DataLinkLayer &` und ist dessen `friend`, damit Interface,
Statistik und Chip-Zustand an einer Stelle liegen statt in beiden Hälften doppelt. Die einzige
umgekehrte Freundschaft ist `Receiver` innerhalb von `Transmitter` (Quittung und Echovergleich);
der `DataLinkLayer` kommt mit deren öffentlichen Methoden aus.
**Genau je eine Referenz, `_dll`** - die Hälften halten *keine* eigenen
`Interface::Abstract &` / `Statistics &` vor. Das sparte eine Indirektion im Bytepfad, erkaufte
aber eine Zusage über die Reihenfolge der Memberdeklarationen im `DataLinkLayer` (die
zwischengespeicherten Referenzen mussten aus bereits konstruierten Membern initialisiert werden),
und für den Transmitter war diese Zusage ohnehin nicht haltbar, weil die beiden Hälften einander
brauchen. Gegen einen 2-3µs-Tick ist die Indirektion nicht messbar. Die Trennlinie ist
**Richtung, nicht Kontext**: alles Lesende liegt im `Receiver`, alles Schreibende im
`Transmitter`, und beide haben eine Tick-Hälfte und eine Loop-Hälfte.
Der eine Fall, der quer liegt, ist die **Quittung**, und sie ist an der Naht geteilt: die
*Entscheidung* liegt in `Receiver::sendAcknowledge()` (sie fällt bei Byte 6 eines noch
eintreffenden Frames, und nur der Empfänger weiß, wie viel davon noch aussteht), das *Byte* geht
über `Transmitter::sendAcknowledge()` hinaus.
Gewachsen ist das aus einer flachen Klasse, die selbst einmal `Receiver` hieß und umbenannt wurde,
als `tick()` auch das Senden treiben musste. Die Namen kamen zurück, als der echte Transmitter
(Sendequeue, `U_L_DataStart/Cont/End`, Wachhund) die Klasse groß genug machte, um sie zu
zerteilen.

**Jede Klasse wird in einer `.h` deklariert und in einer `.cpp` definiert** - einschließlich der
trivialen Getter. Die eine Ausnahme ist `Interface::ArduinoSerial`, und
sie ist unvermeidlich: ein Template, das mit dem Serial-Typ des Aufrufers instanziiert wird,
braucht seine Rümpfe an der Instanziierungsstelle. `Interface::Abstract` hat eine `.cpp` für genau
zwei Funktionen (Destruktor und die Vorgabe für `overflow()`), damit seine vtable einmal statt in
jeder Übersetzungseinheit entsteht. Die `Statistics`-Inkremente je Byte liegen inzwischen außerhalb
der Header; bei 2-3µs je Tick gegen ein 500µs-Intervall ist das nicht messbar.

- **Zeitvergleiche über die Tick-/Loop-Grenze hinweg brauchen eine Momentaufnahme *und* einen
  vorzeichenbehafteten Vergleich.** `_lastReceivedAt` wird vom Tick geschrieben (einem Interrupt)
  und von `loop()` gelesen. Schreibt man `now = millis(); if ((uint32_t)(now - _lastReceivedAt) >=
  TIMEOUT)`, macht ein Byte, das zwischen den beiden Lesevorgängen eintrifft, den Zeitstempel
  *neuer* als das bereits eingefrorene `now`; die vorzeichenlose Differenz läuft auf ~4,29
  Milliarden über und die Frist schlägt sofort zu. Das erzeugte im IP-Router eine Flut unechter
  `BCU disconnected`-Meldungen, nur unter Last (bei 38400 Baud landet alle ~286µs ein Byte in
  diesem Fenster) und unmöglich vor dem `Timer`, als beide Hälften in einem Kontext liefen.
  Die Regel: **erst den Zeitstempel lesen, dann die Uhr**, und trotzdem als `int32_t` vergleichen -
  der Tick kann gleich danach erneut schreiben, und ein leicht vorauseilender Zeitstempel bedeutet
  "gerade eben", nicht "vor Ewigkeiten". Jeder andere Zeitvergleich der Library bleibt innerhalb
  eines Kontexts (`_awaitSince`, `_emptySince`, `_detectRequestSentAt` im Tick;
  `_lastStateRequestAt`, `_lastBusLoadTime`, der Wiederholungsfilter im Hauptkontext); die beiden
  `_detectNextAttemptAt`-Prüfungen waren bereits vorzeichenbehaftet.

- **`tick()` und `loop()` sind zwei bewusst getrennte Hälften, mit dem RX-Ringpuffer als
  Trennlinie.** `tick()` ist die zeitkritische Seite (ein Byte je Aufruf, blockiert nie, dafür
  bestimmt, aus einem Timer-ISR zu laufen): sie zerlegt, sendet die Quittung und schiebt alles
  Fertige in `_rxQueue`. `loop()` ist die entspannte Seite, aus dem Hauptloop gerufen: sie leert die
  Warteschlange, reicht **Telegramme** an einen registrierten Callback und verarbeitet
  **Steuerbytes** intern. Ohne diese Teilung liefe Anwendungscode im Interrupt-Kontext und könnte
  das schmale Quittungsfenster blockieren.
  Für empfangene Daten gibt es keine Poll-API - kein `hasFrame()`/`popFrame()`, kein `rxBuffer()`.
  Telegramme kommen ausschließlich über `registerFrameCallback(std::function<void(Frame &frame)>)`,
  gerufen aus `loop()`. Das `Frame` ist eine Sicht auf einen internen Puffer und nur für die Dauer
  des Aufrufs gültig; es ist nicht `const`, damit der Verbraucher eigene Flags setzen kann (z.B.
  `setFiltered()`).
  **Aus der `tick()`-Hälfte darf nichts Beobachtbares heraussickern.** Ein ISR darf nicht nach
  `Serial` schreiben und nicht allokieren, es gibt also keinen Diagnosehaken je Byte. Ein früheres
  Paar `lastByte()`/`ackSent()` existierte genau für eine bytweise Konsolenspur und wurde aus
  diesem Grund entfernt - diese Form nicht wieder einführen.
  Bekannter Preis, vom Anwender akzeptiert: der Zeitpunkt, zu dem das `U_Ackn.req` abging (früher
  das cyanfarbene `*` hinter Byte 6), ist nicht mehr beobachtbar, weil er nicht Teil des Telegramms
  und damit nicht Teil des Warteschlangeneintrags ist. Ein Flag dafür kann mit der nächsten Stufe
  der Frame-Flags kommen.
- **Politik bei voller Warteschlange: den NEUEN Eintrag verwerfen, das bereits Angenommene
  behalten.** `pushRxEntry()` verweigert, wenn der Eintrag nicht passt, und setzt ein einmaliges
  Flag, lesbar über `queueOverflow()`; die Bytes werden stillschweigend verworfen. Ausdrückliche
  Entscheidung des Anwenders - ändere das nicht "hilfsbereit" in Überschreiben des Ältesten.
- **Aufbau des Ringpuffers**: `[len_lo][len_hi][flags][data...]`. Die Länge braucht zwei Bytes, weil
  263 nicht in eines passt. Die Flags stehen **vor** den Daten, damit ein Leser entscheiden kann, ob
  ihn der Eintrag überhaupt interessiert, bevor er ihn anfasst. Ein Erzeuger, ein Verbraucher: `_rxQueueHead` schreibt nur `tick()`, `_rxQueueTail` nur
  `loop()`, und der Kopf rückt erst weiter, *nachdem* der Eintrag vollständig geschrieben ist, ein
  halb geschriebener Eintrag wird also nie sichtbar. Kopf und Ende laufen monoton mit
  Modulo-Indizierung (derselbe Kniff wie beim TX-Ring des RP2040), damit voll und leer nicht
  verwechselbar sind.
  Telegramm oder Steuerbyte steht **nicht** in einem Flag - es wird aus `data[0]` neu abgeleitet,
  mit derselben Prüfung, die der Parser benutzt hat, also genau mit dem Byte, das die Entscheidung
  ursprünglich getroffen hat.
  `loop()` **prüft die rekonstruierte Länge gegen `TPUART_BUFFER_SIZE`**, bevor es kopiert. In
  geordnetem Betrieb unerreichbar (der Erzeuger schreibt nie mehr als 263), aber das Längenfeld ist
  16 Bit breit, während `_deliverBuffer` ein 263-Byte-*Member* ist - ein Wert außerhalb des Bereichs
  überschriebe die Zustände, den Sendepuffer und die `std::function`-Callbacks. Ein einziger
  beschädigter Eintrag darf nicht das ganze Objekt zerlegen können, es wird also der Ring verworfen
  und weitergemacht.
- **`TP_FRAME_FLAG_*`** - das Flag-Byte ist die API, die der Rest des KNX-Stacks spricht, **die
  Bitbelegung liegt damit fest und darf nicht umsortiert werden**: ein Verbraucher, der das Byte
  numerisch liest statt über die Zugriffsmethoden von `Frame`, bekäme sonst stillschweigend etwas
  anderes. Bit 4 war lange als `ECHO` reserviert und unbenutzt, dort sitzt jetzt `INVALID`.
  **Das Byte ist damit voll** -
  ein weiteres Flag bräuchte ein zweites Byte im Warteschlangeneintrag. Beachte, dass
  `Repeated`/`Extended`/`GroupAddress` keines brauchen: `Frame` leitet sie direkt aus den Daten ab.
  | Bit | Flag | heute gesetzt? |
  |---|---|---|
  | 7 | `TX` | ja - das Echo eines von uns gesendeten Telegramms |
  | 6 | `DATA_CON` | ja - eine Bestätigung für unser eigenes Telegramm kam an |
  | 5 | `FILTERED` | ja - der Wiederholungsfilter hat es als Dublette markiert |
  | 4 | `INVALID` | ja - CRC-Fehler, abgeschnitten, kaputte Länge |
  | 3 | `ADDRESSED` | ja - wir sind für dieses Telegramm zuständig (der Quittungs-Callback sagte ja) |
  | 2 | `ACK_BUSY` | ja - nur im Busmonitor |
  | 1 | `ACK_NACK` | ja - nur im Busmonitor |
  | 0 | `ACK` | ja - nur im Busmonitor |
- **`ACK` heißt "dieses Telegramm wurde quittiert"** - von wem auch immer. Bekannt sein kann das in
  drei Lagen: wir haben selbst quittiert, wir haben die Quittung im Busmonitor auf dem Bus gesehen,
  oder sie kam als Antwort auf ein Telegramm, das *wir* gesendet haben.
  **`ADDRESSED` heißt "wir sind für dieses Telegramm zuständig"** - und das ist ausdrücklich NICHT
  dasselbe wie "wir haben quittiert". Es wird gesetzt, sobald der Quittungs-Callback ja sagt, und
  *bevor* die Rückstandsprüfung greift: ein Telegramm, dessen Quittung unterdrückt wurde, ist trotzdem
  an uns gerichtet, und der Aufrufer soll es als solches erkennen. Umgekehrt trägt unser eigenes Echo
  auch mit Bestätigung **kein** `ADDRESSED` - für ein Telegramm, das wir selbst gesendet haben, sind wir
  der Absender und nicht der zuständige Empfänger. Genau daran fallen die beiden Flags auseinander, und
  `test_own_echo_with_confirmation_is_acked_but_not_addressed` nagelt es fest.
  `setAcknowledge(AcknowledgeType)` setzt entsprechend `ADDRESSED | ACK`. **Verenge `ACK` nicht auf
  "bei uns ist eine Quittung angekommen"**. Diese Lesart wurde ausprobiert und zurückgenommen: sie
  wirkt einleuchtend,
  weil der Chip die sofortige Quittung selbst erzeugt und nie zurückspiegelt, aber sie bricht die
  eine einheitliche Bedeutung, die das Flag über alle drei Fälle hat, und lässt keinen Weg, ein
  eigenes NACK oder BUSY festzuhalten.
- **Zwei Bytepuffer neben den Warteschlangen**: `_rxBuffer` (die gerade eintreffende Sequenz - ein
  Telegramm *oder* eine Steuerbytefolge, nie beides zugleich, ein Puffer deckt also beides ab) und
  `_txBuffer` (das zu sendende Telegramm samt Prüfsumme). Beide `TPUART_BUFFER_SIZE` (263) Byte.
  Steuercodes laufen ausdrücklich *nicht* über `_txBuffer` (siehe die Steuercode-Warteschlange
  unten). Dazu `_deliverBuffer` (263), eine zusammenhängende Kopie für den Callback, weil ein
  Ringeintrag umbrechen kann.
- **Steuercodes haben ihre eigene Warteschlange** (`_ctrlQueue`, `TPUART_CTRL_QUEUE_EXP`, 32 Byte),
  Eintragsformat `[len][bytes...]`. Das ist der zweite SPSC-Ring dieser Klasse, Spiegelbild des
  RX-Rings in die Gegenrichtung: Erzeuger ist der Hauptkontext, Verbraucher ist `tick()`, der Kopf
  wird zuletzt veröffentlicht.
  **Warum eine Warteschlange und nicht Besitz über `TxState`**: Steuercodes müssen absetzbar sein,
  während ein Telegramm gesendet wird. Sie sind keine Telegrammübertragung - genau dasselbe Argument
  galt bereits für die Quittung. Der frühere Entwurf ließ `queueControl()` *verweigern*, sobald
  `TxState != Idle`, was zudem der Bedeutung des Rückgabewerts widerspricht: das `bool` jeder
  Steuerbytemethode heißt "Vorbedingung erfüllt" (nicht initialisiert / falscher Chiptyp), nie
  "gerade beschäftigt".
  Eine Gruppe geht **ganz oder gar nicht** hinaus: `processCtrlQueue()` prüft zuerst, ob das
  Interface Platz für die vollständige Gruppe hat. Eine zu zerteilen ließe die BCU den Rest als
  eigenen Befehl lesen. `TPUART_CTRL_MAX_GROUP` ist 4, die längste Sequenz aus Tabelle 12 des
  Datenblatts (`U_SetAddress.req`, `U_SetRepetition.req`, `U_PollingState.req`) - beachte, dass das
  `TPUART_TX_ATOMIC_BYTES` (3) übersteigt, was nur den Telegrammpfad abdeckt.
  Steuercodes haben in `processTx()` **Vorfahrt** vor Telegrammbytes, und je Tick geht höchstens
  eine Gruppe hinaus. Ein Überlauf wird einmalig über `controlOverflow()` gemeldet; der Code wird
  verworfen, nichts bereits Eingereihtes wird gestört.
  **Ein Einplatz-Zwischenspeicher für die Quittung wäre der falsche Weg** und ist keine Vereinfachung,
  die noch aussteht: einer, der nur aus dem Telegramm-Sendepfad geleert wird, lässt eine Quittung
  beliebig lange liegen und setzt sie dann weit außerhalb ihres ~1,7ms-Fensters ab. Eine Quittung ist
  entweder rechtzeitig oder sie gehört unterdrückt.
- **`RxState` beschreibt nur, was gerade *eintrifft***: `Idle` / `Frame` / `FrameAck` / `Control` /
  `Resync`. Es gibt bewusst **keine "fertig"-Zustände** - sobald etwas fertig ist, geht es in den
  Ringpuffer und der Zustand rückt sofort weiter. Frühere Entwürfe hatten
  `FrameComplete`/`ControlComplete`/`Invalid` als Übergangszustände, die innerhalb eines `tick()`
  gelesen werden mussten; der Ringpuffer macht das alles überflüssig, und `releaseBuffer()`,
  `isCompleteState()` und das aufgeschobene `_resyncAfterRelease`-Flag sind damit entfallen.
  Bewusst *nicht* als Zustände modelliert: "Ziel bekannt" und "Größe bekannt"
  ("Ziel bekannt", "Größe bekannt") - Positionen innerhalb eines Telegramms
  müssen nicht unterschieden werden.
- **`Invalid` ist ein Auslöser, kein Zustand** (ausdrückliche Formulierung des Anwenders und der
  Grund, dass das alte `RxState::Invalid` weg ist). Ein kaputtes Telegramm wird immer gleich
  gemeldet: als normaler Warteschlangeneintrag mit `TP_FRAME_FLAG_INVALID`. Unterschiedlich ist nur,
  was *danach* passiert:
  | Ursache | gemeldet | danach |
  |---|---|---|
  | CRC stimmt nicht | kaputtes Telegramm | `Resync` - dem Frame-Ende ist nicht zu trauen, das Längenoktett kann selbst beschädigt sein |
  | Pause mitten im Telegramm (Abschnitt) | kaputtes Telegramm, so weit es kam | `Idle` - **die Pause ist bereits der Synchronisationspunkt**, den ein Resync suchen ginge |
  | Längenoktett beschädigt (LG=255) | kaputtes Telegramm, so viel wie empfangen | `Resync` - Frame-Ende unbekannt, weiterzusammeln wäre Raten |
  `Resync` bedeutet damit genau eines: alles ab hier ist bedeutungslos bis zu einer bestätigten
  Pause. Bytes werden währenddessen nicht einmal gepuffert.
- **`TxState`**: `Idle` / `Transmit` / `Await` - es geht um das Senden eines **Telegramms** und um
  nichts sonst. Er dient zugleich als Besitzmarke für `_txBuffer`: der Hauptkontext füllt ihn nur
  bei `Idle` und veröffentlicht `Transmit` zuletzt.
  **Weder die Quittung noch ein Steuercode sind ein `TxState`** - beides sind Bytegruppen, die
  zwischendurch hinausgehen, ohne eine Telegrammübertragung anzufassen. Für Steuercodes gilt das
  erst, seit sie ihre eigene Warteschlange haben; davor belegte ein anstehender Steuercode diesen
  Zustand und blockierte alles andere.
  RX und TX laufen nie gleichzeitig, die beiden Zustandsmaschinen können also nicht kollidieren.

- **Die Verlustmelder liegen ebenfalls am `DataLinkLayer`**, aus demselben Grund: sie steuern nichts.
  `_interfaceOverflow`, `_rxQueueOverflow`, `_ctrlQueueOverflow` sind Information für die Schicht
  darüber, kein Zustand, auf den die Hälften reagieren - in ihnen waren sie Zustand, der nichts tat.
  Die Hälften rufen aus dem Tick `reportInterfaceOverflow()` / `reportRxQueueOverflow()` /
  `reportControlOverflow()`, und jede davon tut **beides**, was früher an jeder Aufrufstelle
  nebeneinanderstand: das einmalige Flag setzen *und* den Zähler hochzählen ("ist gerade etwas
  passiert" gegenüber "wie oft insgesamt").
- **Alle Callbacks liegen am `DataLinkLayer`, nicht in den Hälften** - Telegramm, Quittung und
  Meldung (`_callbacksReceivedFrame`, `_callbackCheckAcknowledge`,
  `_callbackMessage`). Er ist die Schnittstelle nach außen und die Stelle, an der sie registriert
  werden, also ist er auch die Stelle, die sie aufruft. Der `Receiver` baut das Telegramm und fragt:
  `_dll.checkAcknowledge()` danach, ob und was zu quittieren ist, `_dll.deliverFrame()` für die
  Übergabe. "Kein registrierter Callback heißt gar keine Quittung" sitzt deshalb ebenfalls dort, wie
  das alte `checkAcknowledge()`.
- **Die Pause wird nur gemessen, solange eine Sequenz offen ist.** `checkPause()` kehrt bei
  `RxState::Idle` sofort zurück - eine Pause kann immer nur etwas *beenden* (ein begonnenes
  Telegramm, einen Resync, eine ausbleibende Antwort), und zwischen zwei Telegrammen gibt es nichts
  zu beenden; die Länge kommt aus dem Längenoktett, nicht aus der Zeit. Das ist zugleich der
  billigste verfügbare Leerlaufpfad: auf einem ruhigen Bus ist `Idle` der Normalzustand, ein Tick
  spart dort den `micros()`-Aufruf der Pausenmessung und kostet stattdessen einen Vergleich.
  (Die Abkürzung spart nicht mehr *jeden* Zeitzugriff: seit der Taktmessung liest `tick()` selbst
  einmal `micros()` - siehe den Leerlaufpfad weiter unten.)
  Zwei Felder bleiben: `_emptySince` + `_emptyStarted` - seit wann das Interface nichts mehr liefert,
  und ob diese Messung überhaupt läuft. Das zweite trägt: der Bezugspunkt muss die *erste*
  Beobachtung von Stille sein, nicht das zuletzt gelesene Byte, sonst hielte ein stehengebliebener
  Tick seine eigene Lücke für eine Buspause. Ein drittes Flag "schon ausgelöst" wurde probiert und
  verworfen - die `Idle`-Abkürzung verhindert den Wiedereintritt bereits, weil jeder Zweig von
  `handleVerifiedPause()` in `Idle` endet. Einen Zweig dort einzubauen, der *nicht* in `Idle` endet,
  bricht diese Zusicherung.
- **Zwei Empfangspuffer plus das Telegramm, drei Aufgaben** - die immer wiederkehrende Frage "warum
  nicht einer?" beantworten ihre Besitzer. `_buffer` (263, **Tick**) ist die eintreffende Sequenz:
  zusammenhängend, weil das Zerlegen hineinindiziert und der Echovergleich `memcmp` benutzt - direkt
  in den Ring zu zerlegen hieße also Modulo-Indizierung im zeitkritischsten Pfad. `_queue` (1024,
  **Tick -> Loop**) ist die ISR-Grenze *und der Rückstau* - rund 80 Telegramme, was ihre eigentliche
  Aufgabe ist: ein Stillstand von einer Sekunde lässt den Bus ~740 Byte nachliefern. Das **`Frame`
  selbst** ist die Übergabe: es besitzt seine Bytes (siehe `Frame.h`) und wird je Eintrag auf dem
  **Stack von `processQueue()`** gebaut, der Ringplatz ist also freigegeben, bevor fremder Code
  läuft. Früher war es ein `_deliverBuffer`-Member des `Receiver` - eine Kratzfläche im C-Stil,
  hinter einem kurzlebigen Objekt; der Speicher gehört dem Typ, der die Daten darstellt. Kein Paar
  lässt sich zusammenlegen: `_buffer` gehört dem Tick und das Telegramm der Loop (dieselben Bytes
  hieße, der Tick überschreibt, was der Callback liest), und der Ring ersetzt das Telegramm nicht
  (er bricht um) und wird nicht von ihm ersetzt (er hält ein einzelnes Telegramm).
  **`Frame` besitzt ohne Heap**: ein festes `TPUART_BUFFER_SIZE`-Array, `Frame(length, flags)` und
  dann `data()` zum Füllen. Ein `malloc` je Telegramm im Empfangspfad würde für die Lebensdauer des
  Geräts im Bustakt allokieren und freigeben, neben den längerlebigen Blöcken der Sendeseite. Der
  Preis ist, dass ein Telegramm immer so groß ist wie das größtmögliche - auf dem Stack belanglos,
  wissenswert, wenn ein Aufrufer eines aufbewahrt.
- **Eine Stelle je Zustandsübergang.** `Receiver::resetSequence(nextState)` ist der einzige Code, der
  die Empfangsfelder löscht (Pufferlänge, Telegrammgröße, CRC, Quittung, Adressiertheit) - früher
  stand er am Sequenzbeginn, in `forceResync()` und in `completeSequence()` ausgeschrieben.
  `Transmitter::beginTransmission()` ist derselbe Gedanke für die Sendeseite (geholtes Telegramm und
  Neustart nach einem Reset). Es geht dabei nicht in erster Linie um Länge: ein Feld, das in einer
  von drei Kopien ergänzt und in den anderen vergessen wird, zeigt sich als Zustand, der ins
  *nächste* Telegramm durchsickert, und das ist das übelste Fehlerbild dieser Klasse.
- **Keine bequemen Weiterleitungen am `DataLinkLayer` für das, was die Hälften schon anbieten.**
  `rxState()`, `txState()`, `isTransmitting()`, `transmitQueueUsed/Size()` gab es und sind entfernt -
  `getReceiver()`/`getTransmitter()` erreichen dieselben Werte unmittelbar. Die Überlaufmelder sahen
  ebenfalls nach Weiterleitungen aus; sie sind keine, denn die Flags selbst sind hierher gewandert
  (siehe oben), es gibt sie also genau an einer Stelle und genau einen Weg, sie zu lesen.
- **`tick()` blockiert nie und verweilt nie** - es ist auf den späteren ISR-Einsatz gebaut, jeder
  Schritt steigt also so früh und so billig wie möglich aus. Es ist `processRx()` + `processTx()`,
  und beide haben ihre Wächter vorn: nichts empfangen -> RX übersprungen (nur der Pausenzähler
  läuft); auf der TX-Seite wird zuerst die Steuerwarteschlange geprüft (leer -> ein Indexvergleich),
  dann `_txState == Idle` -> Telegrammpfad ganz übersprungen; nicht genug Platz im Interface für
  `TPUART_TX_ATOMIC_BYTES` (3) -> auf einen späteren Tick verschoben.
  Die 3-Byte-Reservierung gibt es, weil ein Telegrammbyte von bis zu zwei Positionsbytes begleitet
  sein kann, die unmittelbar hintereinander hinausmüssen - vorab nach dem Platz zu fragen ist das,
  was eine gleichzeitig fällige Quittung daran hindert, dazwischenzurutschen. Es funktioniert, weil
  jedes Interface die Schreibreihenfolge wahrt. Steuergruppen reservieren stattdessen ihre eigene
  Länge (bis zu `TPUART_CTRL_MAX_GROUP` = 4).
- **Ein `tick()` = ein Byte je Richtung** (vom Anwender gesetzte Invariante, beibehalten): es wird
  höchstens ein empfangenes Byte zerlegt und höchstens ein *Telegramm*byte abgesetzt. "Ein Byte" auf
  der TX-Seite heißt ein Byte **des Telegramms** - die bis zu zwei Positionsbytes, die es begleiten
  können (`U_L_DataOffset.req` / `U_L_DataCont.req`), gehören dazu und gehen im selben Tick hinaus,
  wofür `TPUART_TX_ATOMIC_BYTES` (3) genau den Platz reserviert. Das Interface wird nie in einer
  Schleife leergezogen. Das macht einen Tick zu einem **Durchlauf mit fester Kostenobergrenze**,
  worauf der ISR-Plan beruht.
- **Zieltakt: alle ~100µs** (0,1ms). **Maßgeblich ist der Bus, nicht die Host-Strecke**: KNX TP1
  läuft immer mit 9600 Baud, und ein Zeichen belegt 13 Bitzeiten (Start, 8 Daten, Parität, Stop, dazu
  2 Bit Abstand), es trifft also höchstens alle **1,354ms** ein Byte ein - 738 Byte/s. Die 19200
  (oder 38400) zwischen Host und BCU sind mehr als das Doppelte; der Überschuss trägt unsere Sendungen
  und die Steuerbytes, er beschleunigt den Empfang nicht. Bei 100µs sind das ~13 Ticks je Byte, und
  ein Byte je Tick hat reichlich Reserve.
- **Der Leerlaufpfad muss so kurz wie möglich bleiben**, denn bei 100µs ist er der mit Abstand am
  häufigsten ausgeführte Code hier. Heutige Gestalt, gemessen auf dem RP2040: zwei bis drei
  MMIO-Lesevorgänge (`micros()` für die Taktmessung, der DMA-Transferzähler in `available()`, und
  `micros()` in `checkPause()`, solange ein Pausenfenster läuft) plus etwa ein Dutzend Vergleiche -
  deutlich unter 100 von den ~13.300 Takten, die bei 133MHz je Tick zur Verfügung stehen, also rund
  0,5% CPU. Die Reihenfolge der Wächter ist bewusst gewählt: `_bcuState` zuerst,
  dann `_interface.available()`, und auf der TX-Seite `_txState == Idle` vor allem anderen. Muss das
  je billiger werden, sind die zwei naheliegenden Kandidaten, den `micros()`-Aufruf in `checkPause()`
  durch einen Tickzähler zu ersetzen und dem Leerlaufpfad eine Ja/Nein-Abfrage statt der vollen
  Anzahl von `available()` zu geben.
  **Der erste `micros()`-Aufruf ist eine bewusste Abkehr von "so wenig wie möglich"**, und er steht
  ausdrücklich *vor* allen weiteren Abbrüchen in `tick()`: gemessen werden soll der **Antrieb**, also
  wie oft wir gerufen werden, nicht wie oft wir etwas zu tun hatten. Er kostet rund 0,1% CPU und
  bezahlt damit den einzigen Wert dieser Schicht, der sich vorher überhaupt nicht ablesen ließ. Wie
  teuer das Raten stattdessen war, steht bei `getTicks()` in `Statistics.h`.
- **Das Interface gehört `tick()` allein.** Sobald der Timer-Antrieb läuft, darf nichts von außen
  `Interface::Abstract` anfassen - das wäre ein zweiter Zugriffskontext auf dieselben Zähler und
  Hardwareregister, und `RP2040::overflow()` löscht als Nebenwirkung sogar das Overrun-Flag des UART.
  Deshalb läuft die Überlaufmeldung über die Schicht: `tick()` rastet sie in `_interfaceOverflow`
  (nur im Bytepfad, denn ein Überlauf kann nur entstehen, während Daten fließen - der Leerlaufpfad
  bleibt frei von Hardwarezugriff), und der Hauptkontext liest sie über das einmalige
  `interfaceOverflow()`. Ein Aufrufer ruft folglich nichts mehr am `interface` selbst auf.
- **Pollend, nicht rückrufend**, im Einklang mit der Interface-Schicht: `tick()` ist dafür gedacht,
  wiederholt gerufen zu werden - entweder aus `loop()` oder aus einem Hardwaretimer. Immer nur eines
  von beiden darf es treiben.
- **Geteilter Zustand wird über Warteschlangen und Reihenfolge behandelt, nicht über Sperren.**
  - **Steuerwarteschlange**: ein Erzeuger (Hauptkontext), ein Verbraucher (`tick()`), der Kopf wird
    **nach** dem vollständigen Schreiben des Eintrags veröffentlicht. Das *ersetzte* ein Besitzschema,
    in dem `TxState` die Marke für `_txBuffer` war und `queueControl()` verweigerte, sobald der Tick
    sie hielt. Dieses Schema hatte zwei Probleme: es machte Steuercodes während eines Telegrammversands
    unbenutzbar, und jede Nutzlast, die die Marke nicht abdeckte (das Wunschflag für den Busmonitor),
    musste vor dem Veröffentlichen geschrieben werden - was sie nicht wurde, mit einem echten Fehler
    dauerhafter Divergenz als Folge.
  - **RX-Warteschlange**: dieselbe Form, andere Richtung - ein Erzeuger (`tick()`), ein Verbraucher
    (`loop()`), Kopf nach Fertigstellung des Eintrags veröffentlicht.
  - **TX-Puffer** (`_txBuffer`/`_txLength`): `TxState` ist seine Besitzmarke - `Idle` heißt, der
    Hauptkontext darf den Puffer füllen, alles andere heißt, der Tick besitzt ihn. Die Regel dabei
    ist die Lehre aus dem Steuercodefall: *alle* Nutzlast gehört hinter das Veröffentlichen.
  Eine blockierende Sperre käme ohnehin nicht in Frage: auf dem RP2040 läuft `tick()` in einem
  Interrupt, und ein Interrupt kann nicht blockieren. Ein einzelnes `volatile bool` funktionierte auf
  dem RP2040 (nur der Hauptkontext schreibt es, und ein ISR kann vom Hauptloop nicht unterbrochen
  werden), aber **nicht** auf dem ESP32, wo der Tick-Task wirklich parallel auf Kern 0 läuft - dort
  öffnet sich das klassische Fenster zwischen Prüfen und Handeln.
  - **Callbacks**: die beiden sind *nicht* gleichartig, und das ist wichtig.
    `registerFrameCallback()` ist unkritisch - der Telegramm-Callback wird nur aus `loop()` gerufen,
    demselben Kontext, in dem er gesetzt wird. Genau das bringt der Ringpuffer ein.
    `registerCheckAcknowledge()` ist der **eine Callback, der aus `tick()` gerufen wird**: die
    Quittungsentscheidung fällt bei Byte 6 eines noch eintreffenden Telegramms, innerhalb des
    Quittungsfensters, sie lässt sich also nicht über den Ringpuffer aufschieben - das Telegramm
    existiert als Eintrag noch gar nicht. Zwei Folgen: die Funktion muss kurz sein und darf weder
    blockieren noch allokieren (auf dem RP2040 läuft sie in einem Interrupt), und sie muss **vor dem
    Start des Tick-Antriebs** gesetzt werden. In der Praxis werden beide einmal beim Hochlauf gesetzt,
    es braucht also keine Absicherung; wer je zur Laufzeit tauschen will, beachte, dass eine
    `std::function`-Zuweisung nicht atomar ist - sie gibt intern frei und allokiert neu - und eine
    Critical Section oder einen veröffentlichten Tauschplatz bräuchte.
  **Keine Datei hier enthält überhaupt Sperren oder plattformspezifische Synchronisation** - frühere
  Entwürfe hatten `noInterrupts()` und später einen `portMUX`; beides erwies sich als überflüssig,
  sobald Besitz und Reihenfolge ausdrücklich festgelegt waren. Halte es so.
