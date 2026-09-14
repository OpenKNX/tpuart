# Connection.md - Verbindung, Register, Konfiguration, SystemState

Teil von `AGENTS.md`.

- **Verbindungsaufbau** (`begin(BcuType)` / `isConnected()` / `connectedBaudRate()`): aus
  `loop()` heraus und nicht blockierend. Solange `isConnected()` nicht wahr ist, laufen die beiden Hälften überhaupt nicht - `RxState`/`TxState` bleiben unangetastet,
  denn Bytes, die bei unbestätigter Baudrate gelesen werden, sind als Protokoll gar nicht deutbar.
  **Der Ablauf ist ein gespeicherter Zustand, `_bcuState` (`BcuState`)**: `Uninitialized` -> `begin()` ->
  `Searching` -> `Identifying` (nur NCN, nur aus `Searching`) -> `Connected` <-> `Disconnected`, dazu
  `Connected` <-> `BusMonitor` (nur per Reset heraus); `end()` führt nach `Uninitialized`. `bcuState()`
  liefert ihn unverändert, `isConnected()` heißt `Identifying`, `Connected` oder `BusMonitor`. Stop und
  Busy sind keine Zustände - sie liegen quer dazu (`isBusyMode()`, `getSystemState()`).
  **Wer das Interface anfassen darf, hängt an diesem Zustand**, und das ist die Invariante, die zu
  erhalten ist: in `Searching` gehört das Interface dem **Hauptkontext**, der `searchBaudRate()` aus
  `loop()` fährt; ab der ersten erfolgreichen Verbindung gehört es bis zum nächsten `begin()` dem
  **Tick**, einschließlich `reconnect()` in `Disconnected`. Die Übergabe passiert je `begin()` genau
  einmal; danach fasst der Hauptkontext das Interface nicht mehr an, es braucht also weder Sperre noch
  Rückgabe.
  Deshalb sind die beiden Fälle getrennte Funktionen: **die Suche konfiguriert das Interface je
  Kandidat neu (`end()`/`begin()`), und das darf niemals in einem Interrupt passieren** -
  `uart_driver_install()` allokiert auf dem ESP32, und `ArduinoSerial` reicht an ein beliebiges
  fremdes `begin()` weiter. **Der Reconnect braucht nichts davon**: die Baudrate steht fest und kann
  sich nicht ändern, es ist also derselbe `U_Reset.req` und dieselbe Antwort auf einem bereits
  offenen Interface - genau das, was der Tick ohnehin tut. Er verwirft dabei auch nicht mehr den
  Empfangspuffer und initialisiert keine korrekt konfigurierte Hardware für nichts neu.
  **Zwei Schreiber, ein Staffelstab**: der Tick schreibt alle Übergänge aus `Connected`, `BusMonitor`
  und `Disconnected`, der Hauptkontext die aus `Searching` und `Identifying`. Jede Seite schreibt nur aus
  Zuständen, die die andere nicht verlässt. Deshalb **schreibt auch den Verbindungsverlust der Tick**:
  der Hauptkontext stellt die Frist fest und zählt `_connectionTimeouts` hoch, der Tick setzt
  `Disconnected` nur, wenn er dann noch in `Connected` steht, und zählt `_connectionLosses` hoch; Meldung
  und Statistik folgen im Hauptkontext. Schriebe der Hauptkontext selbst, kollidierte das mit
  `Connected` -> `BusMonitor` - auf dem ESP32 mit dem Tick auf dem anderen Kern bliebe sonst
  `BusMonitor` stehen, obwohl die BCU weg ist, und die im Monitor ruhende Überwachung merkte es nie.
  `begin()` und `end()` schreiben aus jedem Zustand und tragen den Tick deshalb **zuerst** aus
  (`Timer::remove()` wartet einen laufenden Tick ab). `begin()` setzt immer nach `Searching`, auch nach
  `end()`: `end()` schließt das Interface, und nur die Suche öffnet es wieder.
  Folge für Aufrufer: **`loop()` muss laufen, damit überhaupt eine Verbindung zustande kommt**;
  `tick()` allein kommt nie dorthin. Dafür lässt sich der Tick-Antrieb jederzeit umschalten,
  verbunden oder nicht - die Library erzwingt keine Regel mehr, um die die Anwendung herumbauen
  müsste.
  Je `tick()` wird höchstens ein Byte betrachtet, dieselbe Disziplin wie überall sonst in dieser
  Klasse. Je Baudratenkandidat blockierend zu warten scheidet damit aus - die Erkennung ist ein
  Zustand über mehrere `tick()`-Aufrufe hinweg, getragen von `_detectAwaitingResponse` /
  `_detectRequestSentAt`.
  **Die Kandidaten hängen am `BcuType`**: `Tpuart2` probiert immer nur 19200; `Ncn5120` (deckt
  NCN5120/5121/5130 ab) probiert 19200, dann 38400. Je Kandidat: `interface.end()` +
  `interface.begin(baud)` (ein sauberer Neustart ist für einen Baudratenwechsel nötig - besonders
  das RP2040-Interface würde sonst versuchen, einen zweiten DMA-Kanal zu beanspruchen) plus ein
  einzelnes `U_Reset.req`.
  **Die Zuordnung ist hier der Kern der Korrektheit** (ausdrückliche Anforderung des Anwenders):
  ein `U_Reset.ind`, das zu irgendeinem unbeteiligten Zeitpunkt eintrifft, darf nie als Bestätigung
  missverstanden werden. Es zählen deshalb nur Bytes, die *nach* dem Absenden der Anforderung
  eintreffen, solange `_detectAwaitingResponse` gesetzt ist - und das erste Byte in diesem Fenster
  entscheidet sofort über den Kandidaten: `0x00` wird übersprungen (der Chip schickt am Anfang
  Nullbytes), alles, was nicht `U_RESET_IND` (`0x03`) ist, lässt den
  Kandidaten sofort scheitern (kein Aussitzen der restlichen Frist auf einen späten Treffer), und
  `U_RESET_IND` gelingt - womit Baudrate und der initiale Reset in einem Schritt bestätigt sind.
  Beides ist ein Ereignis, nicht zwei.
  Ein gescheiterter oder abgelaufener Kandidat rückt sofort zum nächsten weiter; ist die ganze Liste
  durch, pausieren die Versuche `TPUART_DETECT_RETRY_INTERVAL_MS` (1000),
  bevor von vorn begonnen wird - **die BCU antwortet erst, wenn Busspannung anliegt**, und das kann
  dem eigenen Hochlauf deutlich hinterherhinken; dass die Versuche eine Weile laufen, ist also der
  Normalfall und kein Notnagel für seltenes Versagen.
  **Eine Baudratenumschaltung nach dem Verbinden gibt es nicht und ist nicht geplant** - laut
  Anwender müsste dafür die physische Bus-Transceiver-Hardware im Betrieb umkonfiguriert werden,
  wofür dieses Projekt keinen Weg hat.
- **Chip bestimmen über die internen Register** (`processRegisterRead()`, `readRegister()`,
  `readNextRegister()`) - nur NCN512x, im Zustand `Identifying`, also einmal je `begin()` nach der
  ersten Verbindung; in `Uninitialized` und `Searching` meldet `bcuChip()` `Unknown`. Gelesen wird nur,
  was den Chip bestimmt: WD (Anker), ACR1 und auf 5121/5130 RevID. Gespeichert werden nur Chip und
  Revision (`bcuChip()`, `ncnRevision()`), kein Registerabbild. Einen verbundenen TPUART2 meldet
  `bcuChip()` allein aus dem `BcuType`: `U_ProductID.req` (`0x20`) liefert laut beiden
  Siemens-Dokumenten ein nacktes Byte `iiirrrrr`, und beide nennen dafür nur "Release a: 0100 0001" -
  TPUART2 und 2+ sind daraus nicht unterscheidbar. Am Gerät bestätigt: NCN5130 rev 5 und NCN5120.
  **Die Antwort ist genau ein Byte ohne Kennung** ("The next byte returns the data", NCN5130 S. 41) -
  anders als `U_SystemStat.ind`, das seinen Opcode trägt. Der Receiver nimmt deshalb das nächste Byte in
  `Idle` als Wert, sobald ein `U_IntRegRd.req` hinausging (`_awaitRegisterValue`, in `controlByteSent()`
  gesetzt, bei jedem `U_Reset.req` und in `connectDetected()` gelöscht - sonst fräße ein ausbleibender
  Wert das nächste Byte). Der Tick reicht ihn über `registerValueReceived()` weiter, Wert zuerst, Zähler
  zuletzt.
  **Nur im Stop-Modus**: "It's advised to only use this service in Stop, Power-Up Stop or Power-Up State.
  In the other state erroneous behavior could occur" (NCN5130 S. 41, NCN5120 S. 36). Ablauf deshalb
  `U_StopMode.req` -> `U_StopMode.ind` -> Register -> `reset()`. Der Chip wechselt erst nach 30
  Bitzeiten Busruhe in den Stop; die Frist dafür (`NCN_STOP_MODE_TIMEOUT_MS`) läuft deshalb ab dem letzten
  empfangenen Byte (`_lastReceivedAt`), nicht ab der Anforderung - ein langes Telegramm verbraucht sie
  sonst. Nach oben begrenzt `NCN_STOP_MODE_MAX_MS` (500) das Warten: kommt der Stop nie und ruht der Bus
  nie 50 ms, stünden sonst Senden, Konfiguration und Überwachung dauerhaft.
  Solange das läuft, ruhen Statusabfrage, Verbindungsüberwachung, `applyConfiguration()` und der
  Telegrammversand (`registerReadActive()`); Quittungen laufen weiter. Auch `requestState()` aus der
  Anwendung liefert `false` - die Antwort landete sonst als Registerwert.
  **Watchdog als Anker**: nach dem Reset der Verbindungsaufnahme muss `0x0F` stehen, sonst bleibt der Chip
  unbekannt. **ACR1 trennt** den 5120 (`0x00`) von 5121/5130 (`0x60`); RevID gibt es nur auf den beiden
  neueren und wird nur dort angefragt.
  **Ausstieg immer per Reset**, auch nach Fristablauf: `U_ExitStopMode.req` wird ignoriert, solange der
  Stop noch nicht erreicht ist, und eine noch scharfe `U_StopMode.req` schaltete den Empfänger sonst später
  unbemerkt ab (`U_State.req` wird im Stop weiter beantwortet). Die Konfigurationsepoche setzt danach alles
  neu ab. **Genau ein Versuch je `begin()`**: ein Wiederverbinden aus `Disconnected` geht direkt nach `Connected`.
  **Adressbits**: schreiben `28-2B`. Lesen steht in unseren PDFs ebenfalls als `38-3B` - RevID auf `0x3D`
  liegt damit außerhalb des Dokumentierten, antwortet am NCN5130 aber. Daher `U_INT_REG_WR_ADDRESS_MASK`
  (0x03) und `U_INT_REG_RD_ADDRESS_MASK` (0x07).

- **`SystemState`** (`SystemState.{h,cpp}`, erreichbar über `getSystemState()`) hält das Byte hinter
  einem `U_SystemStat.ind`: die Regler-/Oszillatorflags und die Betriebsart des Chips. Es füllt sich
  nur als Antwort auf `requestState()`, `isValid()` beginnt deshalb falsch - was *nicht* dasselbe ist
  wie "Power-UP, alles aus", die Lesart, die ein nacktes Nullbyte sonst bekäme. Kein `volatile`:
  anders als `Statistics` wird es aus `loop()` geschrieben, nicht aus `tick()`.
  Achte auf `SYSTEM_STATE_MODE_NORMAL`: `mode()` maskiert mit `0x03`, die Konstante muss dazu
  passen. Steht dort ein Wert mit gesetzten oberen Bits, kann `normalMode()` nie wahr werden -
  ein Fehler, der lange unbemerkt bleibt, weil `modeString()` direkt gegen `0x03` vergleicht und
  weiterhin das Richtige anzeigt.

- **`setOwnAddress()` / `setRepetitions()`** halten Konfiguration, die die BCU bei jedem Reset
  vergisst ("After reset the address evaluation is deactivated again", Siemens S. 23), sie wird danach
  also erneut gesendet. Auslöser ist eine **Konfigurationsepoche**: der Tick zählt sie bei jedem Reset
  hoch, der Hauptkontext zieht nach und ruft `applyConfiguration()` - ein Zähler statt eines Flags,
  damit jede Seite genau einen Schreiber hat. Die Epoche wird nur nachgezogen, wenn wirklich alles in
  die Steuerwarteschlange gekommen ist; `applyConfiguration()` liefert sonst falsch und der nächste
  `loop()` versucht es erneut, denn diese Konfiguration stillschweigend zu verlieren hieße, die
  Quittung für alles an uns Adressierte stillschweigend zu verlieren. Die beiden Setter benutzen
  denselben Weg über `markConfigurationPending()` (die angewandte Epoche um eins zurückdrehen - der
  Hauptkontext ist ihr einziger Schreiber, und der Wert kann nie mit `_configEpoch` kollidieren).
  Gesendet werden nur Abweichungen: keine Adresse heißt kein Service, und die Wiederholungszähler
  gehen nur hinaus, wenn sie von der Vorgabe nach dem Reset (3/3) abweichen.
  Beide Services gibt es auf beiden Chips, sie unterscheiden sich aber in **Opcode, Sequenzlänge und
  Bitbelegung**: `0xF1`+Adr+Adr+Dummy gegen `0x28`+Adr+Adr, und `0xF2`+Zähler+2 Dummies gegen
  `0x24`+Zähler - mit BUSY in den Bits 6-4 (NCN, Bild 37) gegenüber den Bits 7-5 (TPUART2, Bild 20).
  Achtung bei `0x28`: auf einem TPUART2 ist das `U_SetAddress`, auf einem NCN `U_IntRegWr.req` -
  dieselbe Zahl, völlig anderer Dienst. Die Zähler werden als zwei getrennte Werte gehalten und je Chip
  zusammengesetzt, es gibt also kein Zwischenformat, das in das eine oder andere Layout umgerechnet
  werden müsste - genau dort verwechselt man `||` mit `|` und bekommt immer nur 0 oder 1 heraus.
  **Eine Adresse zu setzen aktiviert die Auto-Quittung des Chips**, und beide Datenblätter sagen das
  ausdrücklich - "Sets the physical address of the device and activates the auto-acknowledge function"
  (NCN5130 S. 36), "If the address is set a complete address evaluation in the TP-UART is activated"
  (Siemens S. 23) -, **`applyConfiguration()` setzt `_autoAcknowledge` deshalb selbst**, wenn es die
  Adresse einreiht. Auf die Bestätigung des Chips zu warten funktionierte nicht: `U_Configure.ind`
  **gibt es nur beim NCN**. Die hostgerichteten Dienste des TPUART2 sind Reset-, ProductID-,
  State-Indication und `L_Data.confirm` (Bild 25), mehr nicht. Auf einem NCN trifft die Indikation
  weiterhin ein und behält weiterhin recht, als eigene Meldung des Chips über seinen Zustand. Damit hat
  `_autoAcknowledge` zwei Schreiber (der Hauptkontext setzt, der Tick löscht bei einem Reset und
  korrigiert aus der Indikation), was hinnehmbar ist, weil die Epoche jede Uneinigkeit binnen eines
  `loop()` auflöst.
  **Es ist reine Information und steuert nichts.** Eine frühere Fassung dieser Datei behauptete das
  Gegenteil - dass `sendAcknowledge()` bei aktiver Auto-Quittung schweigen dürfe, da der Chip schneller
  ist und unser `U_Ackn.req` nur einen aktiven BUSY-Modus abbräche ("BUSY mode is deactivated
  immediately if the host controller confirms a frame by sending U_Ackn.req", S. 35). Das war falsch,
  und es hat echte Telegramme gekostet: **die Auto-Quittung des Chips ist ein Rückfall für den Fall,
  dass der Host nicht rechtzeitig antwortet, kein Ersatz für dessen Antwort**, und sie deckt nur die
  eigene physikalische Adresse des Chips ab - keine Gruppenadressen, und auf einem Koppler nicht die
  fremden physikalischen Adressen, die er weiterleitet. Im IP-Router war die Wirkung sofort sichtbar:
  Telegramme kamen mit `ADDRESSED`, aber ohne `ACK`, und der Sender wiederholte sie dreimal, weil
  niemand auf dem Bus quittiert hatte. Quittiert wird hier deshalb bedingungslos. Die Nebenwirkung auf
  den BUSY-Modus ist der hingenommene Preis - BUSY-Modus benutzt nichts.
  **Was ein Reset sonst noch löscht, und was wiederhergestellt wird.** Die Epoche deckt Adresse,
  Wiederholungszähler und - da es sich als zugehörig herausstellte - die **Spannungsregler (ACR0)** ab:
  `Tabelle 16` gibt als Resetwert `0111 0100` an, also alles an, und der RESET-Zustand wird "entered
  after Power On Reset (POR) **or in response to a U_Reset.req**... NCN5130 gets initialized" (S. 30).
  Ein `powerControl(false)` wäre also von jedem Reset stillschweigend rückgängig gemacht worden -
  einschließlich der Resets, die diese Library selbst sendet (der Wachhund des Transmitters). Gesendet
  wird es auf dem NCN deshalb nach jedem Reset, Vorgabe "an" (`_powerControl`); erneutes Senden ist
  idempotent. Sein Rückgabewert hält die Epoche bewusst nicht offen.
  **ACR0 unterscheidet sich zwischen den Varianten**: Resetwert `0x70` auf dem 5120, `0x74` auf dem
  5130 - Bit 2 (`NCN_ACR0_FLAG_V20VCLIMIT`) ist auf dem 5120 reserviert. `powerControl()` wählt deshalb
  nach `bcuChip()`: auf dem 5120 `0x70`/`0x10`, sonst `0x74`/`0x14` - auch solange der Chip unbekannt ist.
  Bewusst **nicht** wiederhergestellt: Busmonitor (ein Reset ist der dokumentierte Weg *heraus*, ihn
  wiederherzustellen machte `reset()` nutzlos), Busy-Modus (von Natur aus flüchtig), Stop-Modus (vom
  Reset beendet). **`U_Configure.req` wird überhaupt nie gesendet** - erweiterte CRC, Auto-Polling und
  der Marker sind allesamt ungenutzt, und die Vorgabe nach einem Reset ist genau das, was der Parser
  erwartet. Wird eines davon je gewollt, ist `applyConfiguration()` die Stelle, und die Epoche deckt es
  automatisch ab.
  **Ein verpasstes `U_Reset.ind` ist ebenfalls abgedeckt**, über ein Signal, das nichts zusätzlich
  kostet: der NCN durchläuft POWER-UP und SYNC nur nach einem Reset ("entered after Reset State",
  S. 30), und der Zustand wird ohnehin sekündlich abgefragt, `checkChipRestart()` markiert die
  Konfiguration also als anstehend, wenn der Chip nach NORMAL zurückkehrt. Das ist der Grund, dass die
  Konfiguration *nicht* periodisch erneut gesendet wird - erkennen ist billiger als wiederholen. STOP
  zählt ebenfalls als "nicht normal"; das kostet je Stop-Zyklus ein überflüssiges erneutes Senden, das
  dieselben Werte schreibt.
  **Die Auto-Quittung lässt sich nicht abschalten** ("Autoacknowledge can only be deactivated by a
  Reset Service", S. 38; auf der Siemens-Seite ebenso, wo sie nach einem Reset weg ist).
  `setOwnAddress(0)` sendet deshalb nichts und der Chip quittiert weiter - `_autoAcknowledge` bleibt
  bewusst gesetzt, da es den Chip beschreibt und nicht unseren Wunsch. Wer sie tatsächlich loswerden
  will, muss `setOwnAddress(0)` ein `reset()` folgen lassen.
- **Verbindungsüberwachung** (`processConnectionState()`, aus `loop()`): der Zustand wird alle
  `TPUART_STATE_INTERVAL_MS` (1s) abgefragt, und die Verbindung gilt nach
  `TPUART_CONNECTION_TIMEOUT_MS` (5s) ohne ein einziges empfangenes Byte als verloren.
  **Erst die Abfrage macht den Unterschied überhaupt erkennbar**: auf einem ruhigen Bus
  kommt minutenlang nichts, Stille allein sagt also nichts. Eine Abfrage muss beantwortet werden
  (`U_State.ind`), und diese Antwort zählt als Lebenszeichen wie jedes andere Byte - ein belebter Bus
  braucht also keinen zusätzlichen Verkehr, um bestätigt zu bleiben.
  Bei Verlust läuft der Verbindungsaufbau erneut, aber **nur mit der bereits gefundenen Baudrate** -
  `_detectCandidateIndex` bleibt unangetastet, und `advanceDetectCandidate()` läuft nur in der Suche.
  Die Baudrate ist eine Hardwareeigenschaft der BCU (der BDS-Pin bei einem TP-UART 2+) und kann sich im
  Betrieb nicht ändern, erneutes Abtasten hieße also nur, für nichts mit falschen Raten an die BCU zu
  schreiben; eine andere zu finden setzt einen Neustart voraus. Der Wiederholungsversuch selbst ist
  geduldig - die Busspannung kann eine Weile weg sein -, und `_connectReported` wird beim Melden des
  Verlusts gelöscht, damit der Reconnect erneut angekündigt wird.
  `connectionLost()` läuft im Tick, der die Detect-Felder in `Disconnected` ohnehin allein benutzt; der
  Zustand kommt dort zuletzt, der Verlustzähler danach. Ein anstehendes Telegramm überlebt: das
  `U_Reset.ind`, das den Reconnect bestätigt, startet es von vorn, dieselbe Regel wie bei jedem anderen
  Reset.
