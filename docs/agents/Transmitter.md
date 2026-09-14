# Transmitter.md - Senden

Teil von `AGENTS.md`.

- **Ein Telegramm senden** (`pushTransmitQueue()`, drei Überladungen): `const Frame &` ist die
  Umsetzung, `(data, length)` wickelt ein, `Frame *` ist KOMPAT und die einzige, die den **Besitz**
  übernimmt. Ein laufender Versand ist kein Ablehnungsgrund.
  **Alle drei nehmen ein VOLLSTÄNDIGES Telegramm einschließlich Prüfsumme** - es gibt hier keine zwei
  Konventionen mehr. Früher erwartete `sendFrame()` es *ohne* und die KOMPAT-Form kürzte deshalb die
  Länge um eins; dieselben Bytes hatten zwei Bedeutungen. **Die Prüfsumme wird geprüft, nicht neu
  gerechnet**: sie gehört zum Telegramm, und sie stillschweigend zu überschreiben verdeckte einen
  Fehler im Aufrufer - das Telegramm ginge dann mit korrekter CRC über falschem Inhalt hinaus.
  Geprüft wird über `Frame::isValid()`, und zwar **vor jeder Veränderung am Puffer**; ein abgelehntes
  Telegramm hat ihn also nie angefasst, was jedes Rückabwickeln erübrigt.
- **Die Warteschlange ist ein linearer Bytepuffer** (`TransmitQueue`, `TPUART_TX_BUFFER_SIZE`,
  Vorgabe 2048) und **gehört dem Hauptkontext allein**. Das ist die Bedingung, unter der darin nach
  Priorität umsortiert werden darf: der Tick fasst sie nicht an.
  **Die Library hat damit keinen Heap mehr im Datenpfad.** Vorher lag jedes wartende Telegramm in
  einem eigenen `malloc`-Block variabler Größe - die einzige fragmentierende Allokation, im Bustakt
  über die Lebensdauer des Geräts.
  **Bytes statt Telegrammzahl**, und das ist der Grund: ein gewöhnliches Gruppentelegramm ist 9-15
  Oktetts, ein maximales 263. Feste Plätze verschenkten rund 95%; dieselben 2048 Byte fassen ~140
  gewöhnliche Telegramme, als feste Plätze wären es 7.
  **Wie viel man braucht, ergibt die Anwendung, nicht der Bus**: der knx-Stack sendet je `loop()`
  genau *ein* Telegramm (`sendNextGroupTelegram()`), der Loop läuft aber um ein Vielfaches schneller,
  als der Bus abfließt (~50 Telegramme/s). 100 gleichzeitig geänderte KOs sind nach ~200ms alle
  eingereiht, während der Bus ~10 geschafft hat - es warten also ~90. Zusammengefasst wird bereits im
  KO selbst (mehrfaches Schreiben vor dem Senden ergibt *ein* Telegramm mit dem letzten Wert), die
  Obergrenze ist also die Zahl gleichzeitig sendebereiter KOs.
  **Über uns puffert nichts**: lehnt die Queue ab, verwirft der knx-Stack das Telegramm und meldet ein
  negatives `L_Data.con` nach oben. Jede Ablehnung ist unmittelbar ein verlorenes Telegramm.
  **Kein Längenpräfix** - ein Telegramm beschreibt seine eigene Länge (`Frame::sizeOf()`), und hier
  kommt nur Geprüftes hinein. **Der Empfangsring macht es anders und zu Recht**: der bewahrt auch
  `INVALID`-Einträge auf, und bei genau denen ist die Selbstbeschreibung das, was man nicht glauben
  darf.
- **Prioritäten: System > Urgent > Normal > Low**, strikt, ohne Aging - genau wie der Bus selbst das
  Medium vergibt. Innerhalb einer Klasse gilt Eingangsreihenfolge. Die Rohwerte des Steuerbytes
  (Bits 3-2) sind dabei **nicht** sortiert: `0=System, 1=Normal, 2=Urgent, 3=Low`, Normal und Urgent
  tauschen also die Plätze. Verifiziert gegen den Code, der die Bits schreibt (`knx_types.h`,
  `CemiFrame::priority()` mit Maske `0x0C`) - die Datenblätter beschreiben die UART-Strecke, nicht die
  Rahmensemantik.
  **Die Reserve ist die tragende Hälfte, nicht die Kür.** Sortieren hilft nur Telegrammen, die es in
  die Queue *geschafft* haben; ein Low-Ansturm füllt sie sonst, und das System-Telegramm der
  ETS-Verbindung wird an der Tür abgewiesen. `TPUART_TX_PRIORITY_RESERVE` (= `TPUART_BUFFER_SIZE`,
  263) ist deshalb abgeleitet, nicht geraten: ein maximales Telegramm oberhalb von Low passt immer.
- **Der Tick bekommt eine Vorlage gereicht**, er holt sich nichts. `_stagedData`/`_stagedLength` plus
  zwei monotone Zähler, je ein Schreiber: `_stagedSeq` (Hauptkontext), `_takenSeq` (Tick), Nutzlast
  vor Zähler veröffentlicht. Drei Regeln tragen das: die Vorlage nur schreiben, wenn beide gleich
  sind; den Platz erst freigeben, wenn `_takenSeq` nachgezogen hat; und der Tick kopiert in `_buffer`,
  weshalb die Bytes danach weg dürfen - auch `restart()` nach einem Reset arbeitet aus `_buffer`.
  **Der Sendestart hängt damit nicht am Hauptloop**: nach der Übernahme hat er die gesamte Dauer der
  laufenden Übertragung Zeit nachzulegen (20ms und mehr), ein 53ms-Flash-Stillstand ist also gedeckt.
  Der Preis ist eine **auf genau ein Telegramm begrenzte Inversion** - eine bereits vorgelegte
  Sendung lässt sich nicht zurückholen. Schlimmster Fall: ein maximales in Übertragung plus ein
  maximales vorgelegt, also ~712ms, gegen eine T_Ack-Frist von rund 3 Sekunden.
  **Das löst zugleich zwei alte Wettläufe auf**: `abort()` verwirft nur noch die Vorlage, den Puffer
  räumt der Hauptkontext in `stageNextTelegram()` - und weil Einstellen und Räumen nun im selben
  Kontext liegen, schließt sich auch das Fenster, das hier als "ohne Sperre nicht zu schließen"
  dokumentiert war.
  Jedes Oktett geht mit seinem eigenen Positionsbyte hinaus (`U_L_DataStart.req` /
  `U_L_DataCont.req`, beide `0x80 | position`, keine Fallunterscheidung nötig), das letzte - die
  Prüfsumme - mit `U_L_DataEnd.req`, was die Übertragung auf dem Bus überhaupt erst startet. Das
  Positionsfeld hat nur 6 Bit, ein erweitertes Telegramm braucht deshalb `U_L_DataOffset.req` für die
  oberen 3; der Chip behält diesen Offset, bis er geändert wird, er wird also nur bei Änderung erneut
  gesendet. **Diesen Service gibt es nur beim NCN** - die Siemens-Tabelle
  (`docs/datasheets/Siemens_TPUART.pdf` S. 21) geht von `U_L_DataContinue` (Index 1..62) direkt zu
  `U_L_DataEnd` (Länge 7..63) und vergibt den Opcode `0x08` gar nicht, ihn dort abzusetzen ergäbe
  also ein unbekanntes Steuerbyte. Ein TPUART2 braucht ihn auch nie: er kann nicht mehr als 64
  Oktetts senden, der Index bleibt also unter 64. **Offset 0 wird zu Beginn jedes Telegramms
  ausdrücklich geschrieben**, und das ist kein überflüssiges Byte: der Offset lebt im Register des
  Chips. Ihn nur in einer eigenen Variablen zu führen und die je Telegramm auf 0 zu setzen, ohne das
  Register anzufassen, setzte ein kleines Telegramm nach einem großen am stehengebliebenen Offset ab.
  Ein Oktett je
  `tick()`, weshalb `TPUART_TX_ATOMIC_BYTES` gleich 3 ist (Offset + Position + Daten).
  **Die Busmonitor-Quittung und unser eigenes `L_Data.con` sind derselbe Vorgang**, sie teilen sich
  deshalb einen Zustand (`RxState::FrameAck`) und eine Frist (`TPUART_FRAME_ACK_US`). Beide stammen
  von derselben Quittung auf dem Bus und treffen deshalb gleich spät ein - ~3,3ms nach dem
  Telegrammende. Ein fertiges Telegramm geht in `FrameAck`, wenn der Zustand `BusMonitor` ist *oder* wenn
  es unser eigenes Echo ist (`isOwnEcho()`). **Die Bestätigung setzt die Quittungsflags**, mit
  derselben Bedeutung, die sie überall sonst haben: `ACK` = es gibt eine Quittung (überhaupt eine
  Bestätigung), `ACK_NACK` kommt dazu, wenn sie negativ war - genau die Kombination, die
  `acknowledgeFlags()` für `AckType::Nack` erzeugt. Dazu `DATA_CON` für "eine Bestätigung ist
  eingetroffen", was eine negative Bestätigung von gar keiner unterscheidet (Frist abgelaufen). Ein
  Telegramm ohne Bestätigung trägt deshalb weder `A` noch `N` noch `B`.
  `BUSY` wird aus einer Bestätigung nie gesetzt, und das ist keine Auslassung: die Bestätigung ist
  **ein Bit** (NCN5130 Tabelle 13, Siemens S. 31 - beide ausdrücklich). Der Chip hat seine eigenen
  Wiederholungen gefahren (bis zu 3 nach NACK, bis zu 3 nach BUSY) und meldet nur das Ergebnis, `N`
  heißt hier also "nicht positiv quittiert" und deckt ein NACK ebenso ab wie ein erschöpftes BUSY oder
  schlichtes Schweigen. Die echte Unterscheidung gibt es allein im Busmonitor, wo das rohe
  Quittungsbyte durchkommt.
  Die Reihenfolge zählt dort: `completeSequence()` muss *vor* der Freigabe von `_txState` laufen, weil
  `isOwnEcho()` den Sendeweg noch belegt braucht, um `TX` zu setzen.
  Was in `FrameAck` außer der erwarteten Antwort eintrifft, wird **neu verarbeitet, nicht verworfen**:
  bei Wiederholungen spiegelt der Chip das Telegramm erneut, das nächste Byte ist also meist der
  Anfang dieses Wiederholungsechos. Das alte Verhalten (melden, dann Resync) warf das gesamte
  Wiederholungsecho weg. Ein `U_FrameState.ind` wird geschluckt - im 8-Bit-UART-Modus geht es dem
  `L_Data.con` voraus (NCN5130 S. 42).
  **`TPUART_TX_CONFIRM_TIMEOUT_MS` (10s) ist ein Wachhund, keine Protokollfrist.** Kommt überhaupt
  keine Bestätigung, ist unbekannt, was im Sendepuffer des Chips liegt, und der einzige Weg in einen
  definierten Zustand ist ein Reset - also sendet er `U_Reset.req`. Das Wiederaufnehmen hängt
  **nicht** an dieser Anforderung, sondern am `U_Reset.ind`, und bewusst **ohne zu fragen, wer den
  Reset verursacht hat**: jeder Reset leert den Sendepuffer des Chips, danach beginnt also alles noch
  Offene einfach von vorn (und ein Busmonitor endet, auch bei einem Reset, den wir nicht
  gesendet haben). Bleibt auch die Indikation aus, läuft dieselbe Frist erneut ab und der Reset wird
  wiederholt - gegen eine stumme BCU gibt es nichts Besseres, als es weiter zu versuchen.
  **Gemessen wird fehlender FORTSCHRITT, nicht fehlender Abschluss, jedes Echo macht sie also neu
  scharf** (`echoReceived()`). Das Echo beweist, dass der Chip das Telegramm gerade auf den Bus legt -
  er spiegelt jedes gesendete Oktett zurück, bei jeder Wiederholung ebenfalls -, solange also Echos
  kommen, *kann* die Bestätigung noch nicht da sein, und ein Abbruch schnitte in eine gesunde
  Übertragung. Der Unterschied zeigt sich auf einem belegten Bus: der Chip muss vor jeder Wiederholung
  auf eine freie Leitung warten, und eine feste Frist ab dem `U_L_DataEnd.req` schlüge mitten in der
  Übertragung zu und verwürfe ein Telegramm, das unterwegs war. Zu messen ist der Abstand zwischen
  Lebenszeichen, nicht die Gesamtdauer. 10s sind dafür großzügig: ein maximal großes Telegramm belegt
  den Bus 356ms plus das Warten auf eine freie Leitung.
  Die Frist wird auch nach dem Zuschlagen des Wachhunds neu scharf gemacht, eine stumme BCU bekommt
  also alle 10s einen Resetversuch statt einem einzigen und nie wieder.
  **Der Chip spiegelt jedes gesendete Oktett zum Host zurück** (Datenblatt S. 42), unser eigenes
  Telegramm trifft also als normales Telegramm ein. `isEcho()` erkennt es am Vergleich mit dem
  Sendepuffer - alles außer dem Steuerbyte (dessen Wiederholungsbit die BCU bei einer Wiederholung
  löscht) und der davon abhängigen Prüfsumme. Zwei Dinge hängen daran: `completeSequence()` markiert
  das Telegramm mit `TP_FRAME_FLAG_TX`, und der Wachhund des Transmitters wird neu scharf gemacht
  (siehe oben).
  **`sendAcknowledge()` steigt beim eigenen Echo sofort aus** (`Transmitter::isEchoPrefix()`, bei
  Byte 6) - **man quittiert nicht sich selbst.**
  Hier stand einmal das Gegenteil, mit der Begründung, ein Gerät sende nicht an sich selbst, der
  Quittungs-Callback antworte für das eigene Ziel also "nicht meins" und die Kette ende von allein; der
  Fall, in dem er "meins" sagt, galt als exotisch. **Am echten Gerät widerlegt**: ein IP-Router leitet
  auf Gruppenadressen weiter, die in seiner eigenen Filtertabelle stehen, und sagt für sein Echo
  deshalb "meins". Gemessen über 15,7 Stunden: **25196 gesendete Quittungen, exakt 4 je eigenem
  Telegramm** (Original plus die drei Wiederholungen des Chips) und keine einzige für ein fremdes
  Telegramm. Das ist der Normalfall des Hauptverbrauchers dieser Library, nicht ein Sonderfall.
  Auf dem Bus hatte das Byte nie Wirkung (der Chip verwirft ein `U_Ackn.req`, das während seines eigenen
  Sendens eintrifft) - gratis war es trotzdem nicht: es belegt einen der vier Plätze in
  `TPUART_TX_INTERFACE_BUFFER`, genau in dem Moment, in dem der Sendepfad drei davon für das nächste
  Oktett braucht, und es machte `getTxAcknowledges()` als Diagnose wertlos.
  Verloren geht dadurch nichts: das `ACK` am Echo kommt aus dem `L_Data.con` über `RxState::FrameAck`,
  nicht aus unserer eigenen Quittung.
  **Geprüft wird der ANFANG gegen das laufende Telegramm, nicht "läuft gerade eine Übertragung"** - die
  naheliegende Variante wäre aktiv schädlich: `TxState::Await` dauert bis zur Bestätigung, und in dieser
  ganzen Zeit würde kein **fremdes** Telegramm mehr quittiert. Dafür steht
  `test_foreign_frame_is_acknowledged_while_awaiting_confirmation`, und er ist der einzige Fall, der
  diese Verwechslung fängt (mutationsgeprüft). Dass ein 6-Byte-Anfang genügt, liegt an der Quelladresse
  darin: ein fremdes Telegramm mit *unserer* Quelladresse wäre eine doppelt vergebene physikalische
  Adresse, also ein Anlagenfehler.
