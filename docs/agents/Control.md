# Control.md - Steuerbefehle, Busmonitor, Meldungen

Teil von `AGENTS.md`.

- **Steuerbefehle an die BCU** - `startMonitoring()`, `reset()`, `requestState()`,
  `stopMode(bool)`, `busyMode(bool)`, `powerControl(bool)`. Namen und Bedeutung sind API und
  liegen fest, weil Aufrufercode sie so anspricht.
  Alle liefern `bool` im Sinne von **"Vorbedingung erfüllt"**, nicht "das Byte liegt auf dem Bus" -
  `false` heißt nur: keine Verbindung, falscher Chiptyp oder Steuerwarteschlange voll. Ein Aufrufer,
  der den Wert ignoriert, bleibt damit gültig.
  `stopMode()` und `powerControl()` gibt es nur beim NCN512x und liefern auf einem TPUART2 `false`.
  `busyMode()` funktioniert auf beiden, aber mit verschiedenen Opcodes.
  `requestState()` setzt beim NCN zusätzlich `U_SystemState.req` ab - der *einzige* Weg, auf dem je
  ein `U_SystemStat.ind` eintreffen kann, die eine mehrbytige Steuerantwort, die der Parser
  behandelt. `powerControl()` schreibt ACR0 als 2-Byte-Gruppe über `writeRegister()`; sichtbar wird die
  Wirkung mit der nächsten regulären Statusabfrage.
- **Busmonitor** (`startMonitoring()` / `reset()` / `isBusMonitor()`): `startMonitoring()` reiht
  `U_Busmon.req` (`0x05`) ein; ist der Modus bereits aktiv, wird nichts gesendet und `true`
  geliefert. Es gibt bewusst **kein `setBusMonitor(bool)` und kein
  "aus"** - laut Datenblatt (S. 36, Bild 35) lässt sich der Zustand nur über den Reset-Service
  verlassen, die API bildet das ab: `reset()` sendet `U_Reset.req` (`0x01`), was den Busmonitor als
  Nebenwirkung beendet und den Vorgabe-CRC-Modus wiederherstellt.
  Beachte, dass dieses `reset()` das *nach* dem Verbindungsaufbau ist - es setzt `isConnected()`
  voraus (über `queueControl()`). Der *initiale* Reset, der die Verbindung herstellt, ist ein ganz
  anderer Codepfad: er passiert innerhalb des Verbindungsaufbaus während `begin()`, in
  `BcuState::Searching`, und ist zugleich bei jedem Start der Reset des CRC-Modus (siehe
  "Verbindungsaufbau" oben).
  Zwei Dinge machen die Umschaltung sicher, und beide sind leicht zu brechen:
  1. Der Busmonitor ist ein eigener Zustand (`BcuState::BusMonitor`). Er wird **erst gesetzt, nachdem**
     `processCtrlQueue()` das Byte tatsächlich auf die Leitung gelegt hat, und **aus dem gerade
     gesendeten Code abgeleitet** (`U_BUSMON_REQ`: `Connected` → `BusMonitor`, `U_RESET_REQ` und jede
     `U_Reset.ind`: `BusMonitor` → `Connected`) - beides im Tick, dem alle Übergänge aus `Connected`
     gehören. Ein eigenes "Wunsch"-Feld gibt es bewusst nicht: so ein Feld ist Nutzlast und müsste *vor*
     dem Veröffentlichen des Warteschlangeneintrags geschrieben werden. Diese Reihenfolge war einmal
     falsch - das Veröffentlichen kam zuerst, ein Tick im Zwischenraum las also den alten Wert, und
     Chip-Zustand und Library liefen dauerhaft auseinander. Das Ableiten lässt die Reihenfolgefrage
     verschwinden. `startMonitoring()` liefert während `Identifying` `false`: der Reset am Ende des
     Chip-Bestimmens beendete den Modus gleich wieder.
  2. Das Senden des Codes löst `forceResync()` aus, aber nur für `U_BUSMON_REQ` und `U_RESET_REQ` -
     nur die ändern, was der Bytestrom bedeutet (Bus-Quittungen erscheinen bzw. verschwinden).
     Andere Steuercodes lassen das Format unberührt, dort wäre ein Resync nur ein unnötig verworfenes
     Telegramm. Und `forceResync()` handelt selbst nur, wenn wirklich eine Sequenz im Gang war
     (`RxState::Frame`, `Control` oder `FrameAck`). War `RxState` gleich `Idle`, tut es nichts: es
     ist nichts unterwegs, das zu schützen wäre, und die nächsten Bytes sind schlicht die Antwort auf
     das, was wir gerade gesendet haben (z.B. `U_Reset.ind`). Eine frühere Fassung ging bedingungslos
     in den `Resync` und fraß damit nach jedem Moduswechsel stillschweigend diese Antwort plus das
     erste Telegramm danach - ein echter Fehler, keine Entwurfsentscheidung. `Resync` wird ebenfalls
     in Ruhe gelassen (verwirft ohnehin schon).
  **Was im Busmonitor überhaupt noch wirkt, steht in Tabelle 11 des NCN5130-Datenblatts (S. 32), und
  danach richten sich die Wächter im Code.** Die Tabelle führt je Dienst `E` (wird ausgeführt), `I`
  (wird ignoriert, *ohne* Rückmeldung an den Host) oder `R` (abgelehnt mit Protokollfehler). Im
  Busmonitor sind `I`: `U_State.req`, `U_SystemStat.req`, `U_SetBusy.req`/`U_QuitBusy.req`,
  `U_SetAddress.req`, `U_SetRepetition.req`, `U_Configure.req`, `U_Ackn.req`, `U_PollingState.req` und
  `U_L_DataStart/Cont/End.req`. `E` bleiben nur `U_Reset.req`, `U_StopMode.req`/`U_ExitStopMode.req`
  und `U_IntRegWr.req`/`U_IntRegRd.req`. Dazu die Fußnote, die erklärt warum: *"Bus Monitor state is
  not a separate state. It is applied on top of Normal, Stop, Sync or Power-Up State."*
  Daraus folgen fünf Sperren, und alle fünf sind aus demselben Grund da - was der Chip ohnehin
  ignoriert, gehört nicht in eine Spur, die passiv sein soll:
  - `sendAcknowledge()` steigt sofort aus.
  - `requestState()` liefert `false`, ohne etwas einzureihen. Das deckt zugleich `stopMode()` ab, das es
    zum Sichtbarmachen seiner Wirkung ruft - der Dienst selbst wirkt (`E`), nur die Rückmeldung bleibt
    aus, `SystemState` steht also für die Dauer still.
  - `busyMode()` liefert `false`. Es abzusetzen behauptete sonst über `_busyModeSince` einen Zustand,
    den der Chip gar nicht angenommen hat.
  - `applyConfiguration()` liefert `false`, ohne etwas abzusetzen. Die Konfigurationsepoche bleibt
    damit offen und wird nach dem Verlassen nachgeholt - was sich trifft, denn verlassen wird der
    Modus nur per Reset, und der löscht die Konfiguration im Chip ohnehin.
  - `Transmitter::process()` kehrt um, **vor** dem `Await`-Zweig und vor der Zustandsprüfung.
  **Der gesamte Sendeweg wird beim Umschalten geräumt, nicht eingefroren** - `Transmitter::abort()`,
  gerufen aus `controlByteSent(U_BUSMON_REQ)`. Das umfasst beides: die laufende Übertragung und die
  Warteschlange. Ein halb abgesetztes Telegramm später fortzusetzen ergibt keinen Sinn (auf dem Bus
  hat nie jemand einen Anfang gesehen, und der Chip hat es mit dem Moduswechsel ohnehin verworfen) -
  darin unterscheidet sich `abort()` von `restart()`, das nach einem Reset von vorn beginnt. Und eine
  eingefrorene Warteschlange ginge beim Verlassen auf einen Schlag hinaus, mit Telegrammen, die dann
  längst überholt sind.
  **`abort()` läuft im Tick und verwirft dort NUR DIE VORLAGE**, nicht die Warteschlange. Das ist die
  Aufteilung, die den früheren Wettlauf auflöst: `_txState` behält einen Schreiber (aus dem Hauptkontext
  kollidierte es mit `confirmed()`/`echoReceived()` aus dem Empfangspfad), und die **Warteschlange gehört
  dem Hauptkontext allein** - geräumt wird sie in `stageNextTelegram()`, das im Busmonitor `clear()` ruft.
  Hier stand einmal, `abort()` ziehe `_queueTail` bis an `_queueHead` vor und der Heap werde aus `loop()`
  freigegeben. Beides beschreibt den ersetzten Entwurf: es gibt weder `_queueTail`/`_queueHead` noch Heap
  im Datenpfad, die Sendequeue ist ein linearer Bytepuffer (`TransmitQueue`). **Den Tick dort schreiben zu
  lassen wäre ein echter Fehler** - er fasste `_head` und `_end[]` an, die der Hauptkontext in `push()`
  und `compact()` ebenfalls schreibt, auf dem ESP32 echt parallel. Der Puffer liefe auseinander, und
  `front()` gäbe eine falsche Länge zurück.
  **Busmon an heißt Busy aus, und Auto-Quittung aus.** Der Chip nimmt im Modus keine Telegramme mehr
  an und quittiert nichts; ein Busy-Modus, der das Quittieren nur ersetzt, kann es dort nicht geben,
  und heraus kommt man ohnehin nur per Reset. Der Zustand ist also nicht unbekannt, sondern bekannt
  weg - `controlByteSent(U_BUSMON_REQ)` setzt `_autoAcknowledge` auf falsch und meldet den Busy-Modus
  über `reportBusyModeCancelled()` ab. **Gemeldet statt selbst gelöscht**, weil `_busyModeSince` dem
  Hauptkontext gehört (`busyMode()`, `checkBusyMode()`) und ein zweiter Schreiber aus dem Tick der
  teurere Fehler wäre; den Weg gibt es für genau diesen Zweck schon. Ohne ihn liefe `checkBusyMode()`
  in eine Sackgasse: nach Ablauf der Frist ruft es `busyMode(false)`, und das lehnt im Busmonitor jetzt
  ab - `_busyModeSince` bliebe stehen und der Versuch wiederholte sich endlos.
  **Beim Verlassen wird `_lastReceivedAt` neu aufgezogen** (in `controlByteSent(U_RESET_REQ)`, solange
  der Zustand noch `BusMonitor` ist). Weil die Überwachung während des Modus ruht, ist der Zeitstempel auf
  einem ruhigen Bus beliebig alt - und Busmonitor auf ruhigem Bus ist der Normalfall. Zwischen dem
  `U_Reset.req` und der `U_Reset.ind`, die ihn beantwortet, liegt zwar nur etwa eine Millisekunde,
  aber ein `loop()` genau dort meldete einen Verbindungsabbruch, den es nie gab. Dieselbe Vorsorge
  steht aus demselben Grund schon in `connectDetected()`.
  **Und die Verbindungsüberwachung ruht** (`processConnectionState()` kehrt früh um). Sie ruht auf
  zwei Beinen - Busverkehr als Lebenszeichen und die sekündliche Statusabfrage -, und im Busmonitor
  bricht das zweite weg, weil `U_State.req` dort `I` ist und keine Antwort erzeugt. Auf einem ruhigen
  Bus liefe die 5s-Frist deshalb *zwangsläufig* ab, der Verbindungsverlust setzte den Reconnect in Gang,
  und dessen `U_Reset.req` beendete den Busmonitor: er hielte nie länger als fünf Sekunden. Der Preis
  ist, dass eine im Busmonitor ausfallende BCU unbemerkt bleibt, bis der Modus verlassen wird - bei
  einem passiven Modus hinnehmbar.
  Dasselbe Muster beim Sendepfad: der Wächter steht vor dem `Await`-Zweig, weil ein Telegramm, das
  beim Umschalten schon auf die Bestätigung wartet, sonst nach `TPUART_TX_CONFIRM_TIMEOUT_MS` den
  Wachhund auslöste - und der schickt `U_Reset.req`, beendet also wieder den Busmonitor. Angefangene
  Telegramme frieren stattdessen ein und laufen nach dem Reset von vorn.
  Die Steuercode-Warteschlange ist bewusst von keiner Sperre betroffen: über sie geht der Reset
  hinaus, und der ist der einzige Weg heraus.
  Was der Modus einbringt, ist die Quittung *vom Bus*: `RxState::FrameAck` wartet nach einem
  CRC-gültigen Telegramm ein Byte länger und faltet es
  in die Flags des Telegramms, oder meldet es ohne `ACK`, wenn zuerst die bestätigte Pause zuschlägt -
  dann hat niemand quittiert.
  Das Quittungsbyte wird bewusst **nicht** an den Empfangspuffer angehängt; das Telegramm dort bleibt
  genau so, wie es empfangen wurde. Es liegt in **einem** Feld `_acknowledge`, nicht in einem je
  Herkunft: Busmonitor und eigene Quittung schließen einander aus (wir quittieren im Busmonitor nie,
  und außerhalb davon reicht der Chip keine Bus-Quittung weiter - Siemens S. 32), ein zweites Feld
  wäre also nur ein zweiter Weg zum selben Flag. Welche von beiden es war, trägt
  `ADDRESSED`/`DATA_CON`, nicht das Feld. Erkannt wird sie über `(v & L_ACKN_MASK) == L_ACKN_IND`
  (`0x33`/`0x00`, das Muster `x x 0 0 x x 0 0` aus Bild 35), und beide Flagpaare lesen sich
  **invertiert** - ein gesetztes Maskenbit heißt *nicht* busy / *nicht* nack (TP1: `0xCC` ACK, `0x0C`
  NACK, `0xC0` BUSY). Alle drei von Hand nachgeprüft.
  **Das Warten auf ein einzelnes Byte ruht auf Zeiten, und es braucht eine eigene Frist** - deshalb
  ist das Warten auf eine Antwort (`TPUART_FRAME_ACK_US`, 4000) getrennt von der Erkennung einer
  Buspause (`TPUART_FRAME_WAIT_US`, 2600). Durchgerechnet: die Quittung beginnt 15 Bitzeiten nach dem
  Telegramm (1,56ms bei 9600 auf dem Bus), das Oktett selbst dauert 11 Bit (1,15ms), und die
  Weiterleitung an den Host mit 19200 legt weitere 0,57ms drauf - sie landet also rund 3,3ms nach dem
  Telegrammende auf dem Bus. Unser Fenster beginnt aber nicht dort, sondern sobald wir das letzte
  Byte des Telegramms *gelesen* haben, was selbst ~0,57ms später ist - bleiben ~2,7ms. Gegen eine
  Pausenschwelle von 2600 schlüge die Pause zu, *bevor* die Quittung überhaupt eintrifft, ohne die
  Trennung wäre die Antwort also nie zu sehen; 4000 gibt ~1,3ms Reserve.
  **`TPUART_FRAME_WAIT_US` ist 2600, weil beide Datenblätter das so sagen**, und der Wert ist ein
  *Unterscheidungskriterium*, keine Schätzung: der NCN5130 markiert Frame-Enden - beim Senden wie
  beim Empfangen - mit `>= 2,6 ms Stille` (S. 40, Bilder 44-47 und 50-53), und Siemens sagt dem Host,
  er solle das Paketende "by supervising the EOP gap of 2 - 2,5 ms" erkennen (TP-UART 2 S. 32 / 2+
  S. 33). Ab 2,6ms ist es *garantiert* ein Frame-Ende; darunter rät man, darüber wartet man umsonst.
  Früher stand dort 2800, gemessen gegen den 5,2ms-Abstand zum nächsten Telegramm - die Notbremse
  statt des dokumentierten Frame-Endes, und für beide Chips 200µs zu spät.
  Die untere Schranke ist der Abstand *innerhalb* eines Telegramms: am Host sind das 1,354ms abzüglich
  der Übertragungszeit zur BCU, also ~0,78ms bei 19200 und ~1,07ms bei 38400 - bequem unter 2600. Die
  obere Schranke der alten Begründung gilt weiterhin als Rückfall: das nächste Telegramm kann nicht vor
  50 Bitzeiten (~5,2ms) beginnen, am Host beobachtet ~6,36ms. Beachte, dass das Datenblatt außerdem
  eine Quittungstoleranz von **30 Bitzeiten** dokumentiert ("time-out after 30 bit times", S. 38) -
  ein Gerät, das diesen Spielraum ausschöpft, landete bei ~4,8ms und würde selbst vom 4000er Fenster
  verpasst.
  Ein Byte in `FrameAck`, das keine Quittung ist, wird nicht erwartet; es wird defensiv behandelt
  (Telegramm melden, dann Resync, da dieses Byte bereits verbraucht und der Puffer belegt ist).
  Ein CRC-ungültiges Telegramm wartet **nicht** auf eine Quittung - der Telegrammrand selbst ist an
  diesem Punkt nicht vertrauenswürdig.

- **Meldungen verlassen die Schicht über `registerMessage()`**, samt dem Flag `bool error`. Das ist bewusst *kein*
  Steuerbyte-Callback: Steuerbytes bleiben in der Schicht - sie zu deuten ist ihre Aufgabe -, und was
  der Aufrufer bekommt, ist die fertige Meldung.
  Jede empfangene Steuersequenz wird namentlich protokolliert (`controlServiceName()` in `Types.h`).
  **Nur Überraschungen erreichen die Konsole.** Fünf Dienste werden unterdrückt, weil sie lediglich
  etwas beantworten, das wir selbst getan haben: `U_State.ind` und `U_SystemStat.ind` (die Antwort auf
  die sekündliche Abfrage - sie zu protokollieren füllte die Konsole zweimal je Sekunde mit
  "unverändert"), `U_Configure.ind` (bestätigt die Auto-Quittung, die unsere Adresse eingeschaltet
  hat), `U_Reset.ind` / `U_StopMode.ind` (Antworten auf unser eigenes `U_Reset.req` / `stopMode()`) und
  `L_Data.con`. Letzteres erreicht `handleControlEntry()` nur, wenn es *neben* unserem Telegramm
  eintraf statt dahinter: mit erkanntem Echo wartet der Empfänger in `RxState::FrameAck` darauf und
  hängt es als Flag an. Ohne erkanntes Echo landet es hier - was etwas über den Empfangspfad aussagt,
  nicht über die Bestätigung, und was bereits am fehlenden `A` des gemeldeten Telegramms und an
  `getRxInvalidFrames()` ablesbar ist. Als Konsolenzeile war es nur Lärm. Gemeldet wird stattdessen ihr
  *Inhalt*, und nur, wenn er etwas zu sagen hat: aufgelaufene Fehlerbits über `showStateErrors()`, der
  Systemzustand über `showSystemState()` bei Änderung, der Zustand der Auto-Quittung über
  `isAutoAcknowledge()`.
  **Der eine Reset, der etwas bedeutet, meldet sich selbst, mit Grund**: der Wachhund des Transmitters
  rastet `confirmTimeout()`, und `loop()` macht daraus "No L_Data.con for 10000 ms - BCU reset". Ein
  Reset, den die *BCU* auslöst, bleibt ebenfalls sichtbar - über den Verbindungsverlust oder, auf einem
  NCN, über die Statuszeile, die POWER-UP/SYNC durchläuft (siehe `checkChipRestart()`).
  Die Ausgabe zu unterdrücken ändert nichts an der Auswertung: Reset und Configure werden im *Tick*
  verbraucht (`resetIndication()` / `configureIndication()`), lange bevor der Eintrag hier ankommt. Was
  druckbar bleibt, ist genau das Unerwartete: ein `L_Data.con` oder `L_Ackn.ind` außerhalb des
  erwarteten Ablaufs, ein `U_FrameEnd.ind` (Markermodus, den wir nie einschalten), ein
  `U_FrameState.ind` außerhalb einer Übertragung - und unbekannte Bytes. Ein `U_SystemStat.ind`, dessen
  Länge nicht 2 ist, fällt ebenfalls durch; eine halbe Sequenz ist sehenswert.
  Ein Wert, der zu keinem Dienst aus Tabelle 13 passt, wird als **Fehler** protokolliert, denn die BCU
  kann ein solches Byte nicht senden - es heißt, dass der Bytestrom an dieser Stelle falsch gelesen
  wird. Daneben endet `loop()` mit `showSystemState()` (nur bei Änderung) und `showStateErrors()`.
  Die Fehlerbits aus `U_State.ind` werden ODER-akkumuliert, nicht überschrieben:
  der Chip meldet jedes Ereignis einmal, ein späteres sauberes `U_State.ind` löschte sonst einen
  Fehler, den es gab.
  `printMessage()` formatiert in einen 128-Byte-Stackpuffer (der längste Text ist die ~60 Zeichen
  lange Systemzustandszeile). Nur im Hauptkontext.
