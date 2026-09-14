# Receiver.md - Empfang: Quittung, Steuerbytes, Telegramm, Wiederholungsfilter

Teil von `AGENTS.md`.

- **Quittung (im `Receiver` entschieden, vom `Transmitter` geschrieben)**: sie läuft genau in dem
  Moment, in dem `_frameSize` erstmals bekannt wird - 6 Byte hinein bei einem Standardtelegramm, 7
  bei einem erweiterten - und schreibt `U_Ackn.req` (`0x10 | nack<<2 | busy<<1 | addressed`, NCN5130
  Tabelle 12) direkt ins Interface. Das ist der früheste Punkt, an dem alles Nötige vorliegt: Ziel
  (`[3][4]` standard, `[4][5]` erweitert), Adresstyp (Gruppenbit in `[5]` standard, `[1]` erweitert)
  **und die verbleibende Telegrammlänge**.
  Es heißt zugleich, dass kein "schon quittiert"-Flag nötig ist - die Größe wird je Telegramm genau
  einmal bekannt, dieser Zweig läuft also von Bauart her genau einmal.
  Die Quittung muss hinaus, **während das Telegramm noch läuft** - der Chip legt die sofortige
  Quittung unmittelbar hinter dem Prüfsummenoktett auf den Bus (Datenblatt Bilder 50-52), es gibt also
  keine Gelegenheit nach dem Telegramm. Das TPUART2+-Datenblatt beziffert es: der Service "must be sent
  latest 2,8 ms after receiving the address type octet of an addressed frame" (S. 25) - dieselben
  2,8ms, die `TPUART_FRAME_WAIT_US` begrenzen, und eine unabhängige Bestätigung, dass zwei
  Buszeichenzeiten hier die maßgebliche Einheit sind.
- **Die Quittung wird unterdrückt, wenn wir hinter dem Bus liegen.** `sendAcknowledge()` vergleicht
  zuerst `_interface.available()` mit `_frameSize - _rxLength` - der Zahl der noch ungelesenen Bytes
  dieses Telegramms. Warten **mindestens so viele** bereits, dann ist das Telegramm samt
  Prüfsummenoktett vollständig eingetroffen, es ist auf der Leitung also längst vorbei und sein
  Quittungsfenster zu. Dann zu quittieren wäre schlimmer als zu schweigen - der Chip hängte sie ans
  *nächste* Telegramm und bestätigte damit ein Telegramm, das nie geprüft wurde. Deshalb liefert
  `Abstract::available()` eine Anzahl statt eines Wahrheitswerts.
  Der Vergleich ist `>=`, und der Unterschied zählt: mit `>` schlüpfte der Gleichheitsfall durch, und
  dieser Fall ist zeitlich unbegrenzt - da ein folgendes Telegramm weitere 50 Bitzeiten nicht beginnen
  kann, gilt `available() == remaining` auch nach einem *beliebig* langen Stillstand. `>` ließ also
  genau die Quittung durch, die die Prüfung verhindern soll. Der Normalbetrieb ist unberührt (0-1
  wartende Bytes gegen mindestens 2 ausstehende).
  Welche Quittung gesendet wird, entscheidet `registerCheckAcknowledge(std::function<AckType(uint16_t
  destination, bool isGroupAddress)>)`. **Ohne registrierten Callback wird überhaupt nichts
  quittiert** - `checkAcknowledge()` liefert dann `AckType::None`.
  Alles zu quittieren ist die Entscheidung eines Aufrufers, nie eine Vorgabe: ein Gerät, das jedes
  Telegramm quittiert, behauptet, unter jeder Zieladresse auf dem Bus erreichbar zu sein.
  (`std::function` ist hier eine bewusste Ausnahme vom sonst STL-abgeneigten Stil dieses Projekts,
  vom Anwender so entschieden. Eine zuzuweisen ist nicht atomar -
  intern wird freigegeben und neu allokiert -, sie muss also vor dem Start des Tick-Antriebs gesetzt
  werden. Eine `noInterrupts()`-Klammer gibt es bewusst **nicht**: sie hülfe auf dem ESP32 nicht, wo
  der Tick als Task auf dem anderen Kern läuft, und die Vorbedingung deckt beide Plattformen ab. Eine
  frühere Fassung dieser Datei behauptete, es gebe eine solche Klammer; die gab es nie.)
- **Steuerbytes werden roh und unklassifiziert durchgereicht.** Gegen das NCN5130-Datenblatt geprüft
  (`docs/datasheets/Onsemi_NCN5130.pdf`, Tabelle 13 "Services to Host Controller", S. 34): jede Indikation in der
  Gruppe **Control Services** ist genau 1 Byte lang und aus dem rohen Wert allein verständlich
  (`U_Reset.ind`, `U_State.ind`, `U_FrameState.ind`, `U_Configure.ind`, `U_FrameEnd.ind`,
  `U_StopMode.ind`, `L_Ackn.ind`, `L_Data.con`) **außer `U_SystemStat.ind`, das 2 Byte hat**. Es gibt
  also keine Typisierung `Reset`/`State`/`Configuration`/... - die Schicht behandelt die eine
  2-Byte-Ausnahme intern (über den Zwischenzustand `RxState::Control`) und überlässt die Deutung dem
  Aufrufer. `U_SystemStat.ind` ist zusätzlich an `BcuType::Ncn5120` gebunden, wie in der alten
  Library - auf einem TPUART2 bedeutet `0x4B` etwas anderes und würde sonst das folgende Byte
  verschlucken. Erreichbar ist der Zweig nur über `requestState()`, da der Chip es ausschließlich als
  Antwort auf `U_SystemState.req` sendet.
  **`L_Poll_Data.ind` (`0xF0`) ist hier die Falle, und es hat einen eigenen Zustand.** Es gehört zur
  Gruppe *transparent DLL*, nicht zu den Steuerdiensten. Auf dem **Bus** ist ein Poll mehrbytig (Bild
  56, Siemens Bild 23: Steuerbyte, Quelle ×2, Polladresse ×2, Slotzahl, Prüfsumme, dann ein Byte je
  Slot, bis zu 15). **Auf der Host-Leitung ist er das normalerweise nicht.** Siemens TP-UART 2 S. 32 /
  2+ S. 33, wörtlich: *"From a L_PollData-request only the Controlbyte is transmitted to the host if
  the TP-UART is a polling slave. If the TP-UART is polling master the complete polling frame is
  transmitted to the host as well if a collision is detected during sending the polling master
  frame."* In Bild 56 stehen die übrigen Bytes entsprechend in der Spalte *KNX Bus*, nicht in der des
  Hosts. Was der Chip weiterreicht, wenn er weder Master noch Slave ist, steht nirgends.
  Beide Fälle deckt `RxState::Poll` mit `processPollByte()` ab, gebaut als Spiegelbild von
  `processFrameByte()`: die Kopflänge steht fest, die Slotzahl ergibt die Gesamtlänge, ein voller
  Zyklus wird also **abgezählt und ohne Warten auf eine Pause abgeschlossen**. Folgt nichts - der
  Normalfall -, schließt `handleVerifiedPause()` die Sequenz, und ein einzelnes `0xF0` wird bewusst
  **nicht** als `INVALID` markiert: daran ist nichts kaputt. Nur ein begonnener und dann
  abgeschnittener Zyklus ist es. Nie quittiert: ein Poll trägt keinen Adresstyp, und antworten hieße,
  den eigenen Slot über `U_PollingState.req` zu füllen, was diese Library nicht benutzt.
  Was nicht passieren darf, ist, es als 1-Byte-Steuerbyte zu behandeln: der Parser läse dann die
  Nutzlast als neue Sequenzen - und ein Master an Adresse 1.0.x hat als oberes Quellbyte `0x10`, was
  *ein* gültiger Anfang eines erweiterten Telegramms ist, er baute also ein Phantomtelegramm und
  könnte ein `U_Ackn.req` für eine aus Polldaten zusammengesetzte Adresse absetzen. Eine fremde
  Quittung auf den Bus zu legen ist das übelste Fehlerbild dieser ganzen Schicht. Der naheliegende
  Kurzschluss - `0xF0` byteweise als "unerwartet" verwerfen und die Telegrammerkennung beim ersten
  Pollbyte wieder aufnehmen, das wie ein Anfang aussieht - führt genau dorthin.
  `controlServiceName()` benennt `0xF0`, damit ein erkannter Poll als `L_Poll_Data.ind` gemeldet wird
  statt als Fehler `Unknown control byte F0`. Kein Verbraucher dieser Library (KNX-Stack, OFM-Network,
  OGM-Common) kennt Polldaten überhaupt, weshalb nichts als Telegramm ausgeliefert wird -
  `Frame::isFrame()` ist für `0xF0` falsch, der Eintrag geht also an `handleControlEntry()`. Echten
  Poll-Verkehr gibt es in keiner heutigen Anlage.
- **Die Telegrammgröße** kommt aus `metadataSize() + apduSize()` (standard = 8 + `data[5]&0x0F`,
  erweitert = 9 + `data[6]`), nicht aus der Zeit.
- **Fortlaufende CRC8**: der laufende XOR-Akkumulator wird mit jedem Telegrammbyte fortgeschrieben
  *außer* dem, das an der Prüfsummenposition landen wird (bekannt, sobald der Kopf zerlegt ist) - die
  Prüfung beim Abschluss ist damit ein O(1)-Vergleich (`~crc == data[frameSize-1]`), ohne zweiten Lauf
  über den Puffer.
- **Die Pausenprüfung beruht bewusst NICHT auf "Zeit seit dem letzten gelesenen Byte"**: ein
  Timer/ISR kann ausgehungert werden (pausiert, oder der Hauptloop blockiert, etwa durch ein
  periodisches `delay()` in einer Testumgebung), ohne dass auf dem Bus eine echte Lücke war -
  `interface.available()` gibt den DMA-Puffer wahrheitsgemäß wieder, unabhängig davon, ob *unsere*
  Verarbeitung mitkam. Die Schicht verfolgt deshalb den Zeitstempel der *ersten* Beobachtung von
  "nichts verfügbar" (`_emptySince`); erst wenn dieser Zustand ununterbrochen
  `TPUART_FRAME_WAIT_US` (2600, also 2,6ms - und `TPUART_FRAME_ACK_US`, 4000, während in `FrameAck`
  auf eine Antwort gewartet wird) angehalten hat, gilt er als echte Buspause. Jedes neu eintreffende
  Byte setzt diese Verfolgung sofort zurück.
  **Was tatsächlich herauskommt, hängt am Tick**: die Messung beginnt beim ersten Tick, der "nichts
  da" sieht, und schlägt beim ersten Tick jenseits der Frist zu, die wirksame Pause ist also die
  Schwelle plus bis zu zwei Tickintervalle. Bei 500µs sind das 2,6-3,6ms - eine Lücke von genau 2,6ms
  wird *nicht* erwischt; bei 250µs oder auf `loop1` schon. Der Weg dorthin ist ein schnellerer Tick,
  keine kleinere Schwelle: unter 2,6ms verlässt man, was die Datenblätter zusichern.
- **Ein Moduswechsel löst ebenfalls einen Resync aus** (siehe Busmonitor oben) - dieser verwirft die
  laufende Sequenz, ohne sie zu melden, da er Folge unseres eigenen Handelns ist und nicht eines
  Busproblems.
- **`Frame` ist überall ein VOLLSTÄNDIGES Telegramm einschließlich Prüfsumme**, in beide Richtungen
  und an jedem Einstiegspunkt. Daran hängt mehr, als es aussieht: solange `sendFrame()` es *ohne*
  erwartete und `pushTransmitQueue()` *mit*, hatten dieselben Bytes zwei Bedeutungen, und die
  KOMPAT-Form musste die Länge um eins kürzen. Es gibt kein `+1`/`-1` mehr.
  **`isValid()` rechnet nach und liest zugleich das Flag** - vier Bedingungen: `INVALID` nicht
  gesetzt, Steuerbyte ist L_Data, `length() == size()` (das erledigt Mindestlänge und Vollständigkeit
  in einem), Prüfsumme stimmt. Beide Anteile tragen etwas Eigenes, keiner reicht allein: das Flag
  stützt sich auf Wissen des Empfängers, das den Bytes nicht mehr anzusehen ist (eine Pause mitten im
  Telegramm), und die Rechnung ist das Einzige, was bei einem selbst gebauten *Sende*telegramm etwas
  aussagt, wo die Flags immer 0 sind. Das Flag steht vorn, ein bereits gemeldetes Telegramm kostet
  also nicht einmal den Durchlauf. `isInvalid()` ist die Verneinung davon.
  **Die Längenableitung gibt es genau einmal**: `Frame::sizeOf(data, available)`, statisch, benutzt
  von `Frame::size()`, vom Empfänger (`processFrameByte()`) und von der Sendewarteschlange. Statisch,
  weil `sizeof(Frame)` 263 Byte ist - im Tick je Telegramm und in jedem `front()` wäre ein Objekt der
  falsche Preis. Sie stand einmal zusätzlich inline im Empfänger; zwei Kopien einer Regel laufen
  früher oder später auseinander. Dasselbe gilt für die 9-gegen-8 (`metadataSizeOf()`) und für die
  Prüfsumme (`calcCRC8()`).
  **`Frame::size()` geht dabei über ein nullgefülltes Kopf-Array**, und das ist kein Zierrat: `at()`
  liefert jenseits von `length()` eine 0, `size()` beantwortet damit "wie lang *sollte* es sein" auch
  bei einem Fragment. `sizeOf()` meldet dort 0 ("noch nicht entscheidbar"), und das wäre hier
  gefährlich - `cemiSize()` rechnet `size() - 1`, mit einer 0 entstünde ein 1-Byte-`malloc`, in das
  `cemiData()` dann hineinschriebe.
  `isTruncated()` gab es einmal; es hatte einen einzigen Nutzer (`printFrame()`) und unterschied
  "abgeschnitten" von "Prüfsumme falsch". Beide tragen `INVALID`, und niemand reagiert verschieden
  darauf - kaputt ist kaputt.

- **`RepetitionFilter`** (`RepetitionFilter.{h,cpp}`) markiert ein Telegramm, das der Sender ein
  zweites Mal auf den Bus gelegt hat, weil er keine Quittung sah. Der Verbraucher bekommt es weiterhin,
  mit `FILTERED` markiert, und kann die Verarbeitung überspringen, statt zweimal zu handeln. Geprüft
  und gemerkt wird für **jedes gültige Telegramm**, nicht nur für wiederholte - ohne den Wert des
  Originals wäre die Wiederholung nicht erkennbar; markiert wird nur, wenn zusätzlich `isRepeated()`
  gilt.
  **Das eigene Echo läuft mit, und `getRxRepeatedFrames()` zählt es mit** - ausdrücklich so (Entscheidung
  des Anwenders): das Echo kommt real über den Empfangspfad herein, das `Rx` im Namen ist also richtig und
  gilt für beide Richtungen, weil das gesendete Telegramm zum empfangenen wird. Der Chip löscht bei einer
  Wiederholung das Wiederholungsbit und spiegelt jede Wiederholung genauso zurück wie den ersten Versuch,
  `isRepeated()` greift also.
  **Bei der Deutung wissen**: quittiert auf der Linie niemand unsere Telegramme, wiederholt der Chip jedes
  bis zu dreimal, und der Zähler füllt sich mit unserem eigenen Versand. An einem Router gemessen: 18897
  von 30861 Telegrammen als wiederholt gemeldet, also 61% - und davon **alle** eigene Echos (exakt 3 je
  eigenem Telegramm), fremde Wiederholungen 0. Das liest sich wie ein schwer gestörter Bus und ist keiner.
  Getrennt zu zählen wurde erwogen und verworfen; wer es doch braucht, hat an der Zählstelle das Flag
  `TP_FRAME_FLAG_TX` zur Hand.
  Gespeichert wird je Sender ein 16-Bit-Fingerabdruck (CRC-16/SPI-FUJITSU, mit erzwungen gesetztem
  Wiederholungsbit, damit Original und Wiederholung gleich hashen), nicht das
  Telegramm: 50 maximal große Telegramme wären 13KB, das hier sind 400 Byte. Der Preis ist eine
  theoretische Kollision, die 16 Bit vertretbar machen.
  **Einträge verfallen nie, und das ist Absicht.** Ein Eintrag je Quelle; ein Eintrag verschwindet nur
  dadurch, dass er verdrängt wird, wenn eine *neue* Quelle auftaucht und alle
  `TPUART_REPETITION_FILTER_COUNT` (50) Plätze belegt sind - die am längsten nicht gesehene Quelle
  fällt heraus. Eine LRU-Liste also, aber ohne `std::list` + `unordered_map`: ein festes Array mit
  linearer Suche über 50 Einträge, was im Hauptkontext nichts kostet.
  **Baue keine Zeitgrenze ein.** Sie wurde probiert (2s) und entfernt. Es gibt keine Schranke
  abzuleiten: wann die Wiederholung eintrifft, hängt davon ab, wann der Sender den Bus das nächste Mal
  frei sieht, und das können Millisekunden oder zehn Sekunden sein. Eine zu kurze Grenze lässt die
  Wiederholung als vermeintliches Original durch - genau das, was der Filter verhindern soll.
  Der eine Fall, in dem der Filter etwas kostet, wird hingenommen: verpassen wir ein *Original*, trifft
  seine Wiederholung ein, und gehört der gespeicherte Fingerabdruck noch zu einem früheren,
  byteidentischen Telegramm derselben Quelle, dann ist die einzige Kopie, die wir bekommen, als
  Wiederholung markiert. Diese beiden Fälle sind von außen nicht unterscheidbar - zu welchem von zwei
  identischen Telegrammen eine Wiederholung gehört, ist schlicht nicht wissbar -, und danach mit einem
  Zeitgeber zu raten ist schlechter, als den Verlust hinzunehmen.

- **Die Puffergröße ist 263, und die zugehörige Größenprüfung trägt.** Das größte *gültige* Telegramm
  ist erweitert mit maximaler APDU: 9 + 254 = 263. 254, nicht 255 - der Wert 255 ist im Längenoktett
  eines erweiterten Telegramms reserviert, eine APDU wird also nie länger als 254. Damit ist
  `_frameSize > 263` erreichbar (ein Längenoktett von 255 ergibt 264), und das ist genau der Fehlerfall
  "unplausible Größe". Lösche diese Prüfung nicht als unerreichbar - unerreichbar ist sie nur, wenn der
  Puffer auf 264 überdimensioniert wird. Ein `static_assert` in `Receiver.cpp` nagelt `>= 9 + 254`
  fest, den Puffer zu verkleinern lässt also den Bau scheitern, statt still überzulaufen; ein zweites
  sichert zu, dass der Ringpuffer mindestens einen maximal großen Eintrag fasst.
