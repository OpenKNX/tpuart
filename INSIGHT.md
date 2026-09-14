# INSIGHT.md

Die TPUart-Library läuft schon eine Weile produktiv, und im Betrieb lernt man dazu - vor allem, wo
es hakt. Die bisherige Umsetzung war selbst schon die zweite oder dritte, und mir war klar, dass sie
nicht die letzte bleiben wird - dafür steckten zu viele Kompromisse drin, und eine Idee, wie es
anders ginge, hatte ich auch schon. Ich wollte aber erst weitere Erkenntnisse sammeln, um irgendwann
eine bessere Version zu bauen. Bis dahin war so ein Umbau schlicht zu viel Arbeit. Mit der KI war
die Hürde viel kleiner, und ich konnte anfangen, das alte Konzept umzusetzen und zu prüfen, ob es
überhaupt funktioniert.

---

## 1. Telegrammerkennung

Das Kernthema. Antrieb, Sperren und Puffergrößen hängen alle daran, wie diese Frage beantwortet wird.

### Das Problem

Ein Telegramm hat ein Startbyte - das Kontrollfeld, erkennbar an seinem Bitmuster - und ein
Längenbyte, aus dem sich das Ende ergibt. **Solange es keine Fehler gibt, ist die Erkennung völlig
unproblematisch:** Anfang aus dem Kontrollbyte, Ende aus der Länge.

**Der Bytestrom trägt aber nicht nur Telegramme.** Dazwischen liegen Steuerbytes der BCU -
`U_Reset.ind`, `U_State.ind`, `L_Data.con`, `L_Ackn.ind` und weitere -, und die sind je ein
einzelnes Byte ohne Rahmen. Ihre Bitmuster unterscheiden sich von dem eines Telegrammanfangs, an
einer bekannten Grenze sind die beiden also auseinanderzuhalten. **Ein Byte im Nutzdatenbereich
eines Telegramms kann aber jeden Wert haben** - auch den eines Steuerbytes oder eines
Kontrollfeldes.

**Aus dem Bytewert allein ist also nicht entscheidbar, was er bedeutet.** Der einzige
Unterscheider ist der Zustand: stecke ich mitten in einem Telegramm, ist es ein Datenbyte, und das
Längenbyte sagt, wie viele noch folgen. Bin ich zwischen zwei Telegrammen, ist es ein Steuerbyte.

Mit einem Fehler ist genau dieser Zustand verloren. Und damit ist nichts mehr deutbar - weder
Telegramme noch Steuerbytes. Was wiederhergestellt werden muss, ist deshalb nicht "der nächste
Telegrammanfang", sondern **der Zustand**.

Fehler entstehen auf zwei Wegen, und der naheliegende ist der seltenere.

**Host-seitig: der UART wird nicht schnell genug geleert.** Die Hardware-FIFO fasst 32 Byte, kommt
der Leser zu spät, fallen Bytes weg - still und an unbekannter Stelle. Auf dem RP2040 ist das durch
die DMA praktisch ausgeschlossen, und die gab es schon in Version 1: sie schreibt in einen großen
Ring, in Hardware, ohne auf den Leser zu warten. Der ArduinoSerial-Adapter hat sie nicht, er hängt an
der Serial-Klasse darunter.

**Bus-seitig: die BCU verliert selbst ein Telegramm.** Das ist der wichtigere Fall, und die NCN-BCU
neigt dazu. Am Host sieht das so aus: die ersten Bytes treffen ein, dann kommt nichts mehr - bis die
BCU sich auf dem Bus wieder aufsynchronisiert hat.

Das Telegramm ist damit abgeschnitten, ohne dass an irgendeinem Bytewert etwas erkennbar wäre. Was
es verrät, ist die **Stille danach**.

**Die DMA nimmt aber die Zeitinformation weg.** Man liest aus einem Ring und weiß nicht mehr, wann
ein Byte angekommen ist. Genau damit könnte man eine Pause auf dem Bus erkennen und daran wieder
aufsetzen.

**Und das war einer der Gründe für die Suche in Version 1.** Ohne verlässliche Zeitinformation
bleibt nur, im Bytestrom selbst nach Struktur zu suchen.

### Das alte Konzept: Suche im Puffer

Version 1 sammelte alle empfangenen Bytes in einem 300 Byte großen `SearchBuffer`. Das `Frame` war
eine **Sicht auf Position 0** dieses Puffers, und die Frage lautete jedes Mal: sieht das, was vorne
liegt, wie ein Telegramm aus? Passte es nicht, wurde ein Byte verworfen, das Fenster
weitergeschoben und erneut gefragt.

Eine Pause auf dem Bus war dabei kein Ereignis, sondern eine **Positionsmarke** im Puffer, die
später mit der erwarteten Bytezahl verglichen wurde.

Der Ansatz braucht keine Zeitinformation, kommt also mit dem aus, was die DMA übrig lässt. Und liegt
hinter dem Müll schon das nächste intakte Telegramm, findet die Suche es sofort.

**Das gilt aber nur für Telegramme.** Steuerbytes haben kein Muster, nach dem sich suchen ließe -
sie sind ein einzelnes Byte, das genauso als Telegramminhalt vorkommen kann. Die Suche kann den
Zustand also gar nicht wiederherstellen, sie kann nur einen Telegrammanfang vermuten. Für alles
andere muss man weiterhin wissen, in welchem Zustand die BCU ist - und das ist der Punkt, an dem
der Ansatz nicht trägt.

**Ein abgeschnittenes Telegramm wird so aber nicht als Telegramm gemeldet.** Erkennen könnte die
Suche es durchaus - der Anfang ist signalisiert, und aus dem Längenbyte steht die erwartete Länge
fest. Nur bleibt es beim Erkennen: passt die Bytezahl nicht zur Positionsmarke, verwirft
`processSearchBufferInvalid()` das erste Byte und schiebt weiter. Der Schaden fällt damit in Form
verworfener Bytes an, nicht als Telegramm mit Fehlermerkmal.

Und das ist nicht dasselbe: ein Telegramm mit falscher Prüfsumme ist eine Aussage über den Bus,
verworfene Bytes sind eine Aussage über die eigene Verarbeitung.

### Das neue Konzept: die Pause als Grenze

Kein Suchpuffer. Eine Zustandsmaschine und ein Telegrammpuffer. Telegrammgrenzen kommen aus dem
Längenbyte; stimmt etwas nicht, geht die Maschine in `RxState::Resync` und verwirft alles **bis zu
einer verifizierten Pause** - das Interface war 2600 µs ununterbrochen leer, der Bytestrom ist
danach synchron.

Die Zeitinformation, die die DMA wegnimmt, wird auf einem Umweg zurückgeholt: die Pausenerkennung
misst nicht den Bus, sondern **wie lange das eigene Interface nichts hergibt**. Das ist gleichwertig,
solange der Leser mitkommt - und der hat im Normalbetrieb großen Abstand: ein Byte je 1,354 ms vom
Bus gegen einen Tick alle 500 µs.

### Vorteile

**Der Zustand ist wiederhergestellt, nicht bloß vermutet.** Nach einer verifizierten Pause steht fest,
dass die BCU zwischen zwei Telegrammen ist. Damit ist das nächste Byte eindeutig - Steuerbyte oder
Telegrammanfang -, und beides ist wieder deutbar. Eine Suche liefert das nicht, sie kennt nur
Telegrammmuster.

**Ein Falschtreffer ist nicht möglich.** Eine Pause ist nachgewiesen, ein Suchtreffer geraten. Die
Suche findet ihr Bitmuster irgendwann auch dort, wo keines ist; was dann folgt, sieht wie eine
Zieladresse aus, und das Gerät quittiert ein Telegramm, das es nie gab. Genau das ist mit dem
Poll-Telegramm passiert: `0xF0` kennt Version 1 nur als Konstante und verwirft es byteweise, bis
ein Poll-Byte wie ein Telegrammanfang aussieht - ein Master unter 1.0.x liefert mit `0x10` genau
so eines, und aus den Poll-Bytes wird ein Telegramm samt Quittung auf eine Adresse, die es nicht
gibt. Hier ist der Poll ein eigener Zustand: der Kopf hat eine feste Länge und der Slot-Count sagt,
wie viele Bytes folgen, also wird abgezählt statt geraten.

**Defekte Telegramme werden als Telegramme gemeldet.** Die Grenze steht durch das Längenbyte fest,
also ist auch entscheidbar, dass ein Telegramm da war und nicht stimmte - falsche Prüfsumme, von einer
Pause abgeschnitten, Länge unmöglich. Es geht mit `INVALID`-Flag nach oben, statt als verworfene Bytes
zu verschwinden, und die Statistik trennt beides seitdem: `getRxInvalidFrames()` gegen
`getRxDroppedBytes()`. Version 1 kannte nur "discarded" und vermischte es.

Für einen Busmonitor ist das der entscheidende Unterschied. Wer den Bus mitschneidet, will ein
kaputtes Telegramm sehen, mit Absender, Ziel und Inhalt soweit angekommen - und nicht die Meldung
"hier wurden 14 Bytes verworfen". Genau das war mit der Suche nicht möglich, sie hatte nichts, was sie
als Telegramm hätte ausgeben können.

**Und es ist weniger Code und weniger Speicher.** Kein byteweises Neuinterpretieren des Puffers bei
jedem Fehlschlag, 300 Byte RAM weniger, keine Positionsmarke, die mit erwarteten Bytezahlen
verrechnet werden muss.


### Der Preis

Nach einem Fehler geht alles verloren, was bis zur nächsten Pause noch kommt - der Sache nach also
auch ein intaktes Telegramm, das die Suche noch gefunden hätte.

**In der Realität passiert das nicht.** Einen größeren Rückstau darf es gar nicht geben, weil die
Verarbeitung wegen der Quittung zeitkritisch ist. Kommt es doch zu einem Interface-Overflow, sind in
der Regel mehrere Bytes betroffen - oft schon das nächste Telegramm. Und eine Unterbrechung der
Tick-Verarbeitung setzt voraus, dass die Timer aktiv abgeschaltet werden; das passiert primär beim
Schreiben in den Flash.

Es muss schon eine sehr ungünstige Konstellation zusammenkommen, damit wirklich ein Telegramm
rettbar gewesen wäre.

### Der Sonderfall: Rückstand in der DMA

**Genau für diesen Fall ist die DMA da.** Während die Interrupts gesperrt sind, läuft sie in Hardware
weiter und sammelt die Bytes ein - der Flash-Zugriff kostet dadurch keine Telegramme, sie kommen nur
später.

**Damit entsteht das neue Problem: im Ring steht kein Zeitstempel.** Wann ein Byte angekommen ist,
lässt sich daraus nicht ablesen - und die Zeit ist genau die Information, an der ein abgeschnittenes
Telegramm erkennbar wäre. Sie wird durch die DMA also erst zum kritischen Punkt.

Die Pausenerkennung behilft sich deshalb mit dem Leerlauf des eigenen Lesers, und die Gleichsetzung
"Interface leer" = "Bus still" trägt nur, solange er mitkommt. Während ein Rückstand abgearbeitet
wird, meldet das Interface durchgehend Daten - `checkPause()` wird nie erreicht, **es wird also keine
Pause erkannt.**

Solange der Empfang selbst ungestört war, kostet das nichts. Die DMA verliert nichts, sie liefert nur
später: der Bytestrom ist vollständig, die Grenzen kommen aus den Längenbytes. Eine Pause braucht man
nur zum Wiederaufsetzen, und zu korrigieren ist hier nichts.

Steckt im Rückstand doch ein Fehler - eine falsche Prüfsumme, oder ein Überlauf, weil die Blockade
länger anhielt als der Ring fasst -, dann ist es dasselbe Problem wie oben: es muss alles verworfen
werden, bis wieder eine Pause erkannt wird. Und die kommt erst, wenn der Rückstand abgearbeitet ist.

## 2. Antrieb und Sperren

### Das Problem

Verarbeitet wird normalerweise aus dem `loop()`. Sobald dort aber eine blockierende Anfrage
dazwischenkommt, steht die Verarbeitung, und es gehen Daten verloren. Deshalb gab es schon in der
alten Umsetzung einen asynchronen Weg, unabhängig vom Hauptloop: einen Interrupt, der am UART- bzw.
DMA-IRQ hing, also am Eintreffen eines Bytes.

Funktionieren sollte es aber auch minimalistisch, ohne diesen Weg. Die Verarbeitung war deshalb so
gebaut, dass sie über verschiedene Wege angestoßen werden konnte - und das verlangte
Sperrmechanismen, damit die Wege sich nicht in die Quere kommen.

Damit war es nicht getan, denn aus dem Empfangspfad muss auch gesendet werden: die Quittung gehört zu
einem einlaufenden Telegramm. Also noch eine Sperre und noch ein Zwischenspeicher. Und lief gerade ein
Telegrammversand, durfte das Acknowledge nicht einfach dazwischengeschrieben werden - zwischen Offset-
und Nutzbyte gehört nichts, sonst deutet die BCU die Reste als eigenen Befehl. Das zeitkritischste
Byte des Protokolls landete damit im Zwischenspeicher und wurde später nachgeschickt, sobald die
Sperre frei war.

So entstanden immer mehr Sonderfälle, die abgefangen werden mussten, und die Komplexität stieg mit
jedem davon weiter an.

Naheliegend wäre, Senden und Empfangen zusammenzulegen. Das ging aber nicht, weil der Versand nicht am
Eintreffen eines Bytes hängen darf - wenn man senden will, kommt keines.


### Das neue Konzept: ein fester Tick

Die Idee, die Verarbeitung auf einen festen, zyklischen Aufruf zu stellen, schlummert fast seit dem
ersten Tag - ob als Timer-Interrupt, als eigener RTOS-Task oder auf dem zweiten Kern. Damit wandert
die ganze Synchronität aus der Verarbeitung heraus.

Weil nun ohne Verzögerung zyklisch verarbeitet wird, kann ein Schritt **lesen und schreiben**. Und
damit gibt es nur noch eine Stelle, die beides tut: Sperren braucht es nicht mehr, der Austausch
zwischen den beiden Kontexten läuft über Zustände, die atomar geschrieben werden. Auch die
zwischengespeicherte Quittung entfällt - sie geht raus, wenn sie fällig ist, oder gar nicht. Ein Tick
hat dabei festen Aufwand: ein Byte je Richtung, kein Warten, keine Schleife über unbekannt viele
Bytes.

Der Nachteil ist die Voraussetzung. Man braucht immer einen ausgelagerten Aufruf, und der muss oft
genug kommen - sonst funktioniert die Pausenerkennung nicht richtig. Auf dem RP2040 geht das per
Timer-Interrupt oder über `loop1()`, wenn der zweite Kern sonst frei ist; auf dem ESP32 reicht ein
RTOS-Task.

Einfach ist die Umsetzung damit nicht geworden, jedenfalls nicht so einfach wie erhofft. Der
Grundablauf ist es aber: ein Takt, zwei Hälften, ein Ringpuffer dazwischen.



## 3. Die Quittung

### Das Problem

Die Quittung ist das zeitkritischste Byte des Protokolls: sie muss beim Chip sein, bevor das Telegramm
auf dem Bus zu Ende ist. Zwei Dinge arbeiten dagegen - was noch vor ihr im Sendepuffer liegt, und die
Frage, wie weit man im Bytestrom überhaupt schon ist.

### Das alte Konzept

Das Acknowledge wurde in den Sendepuffer des UART geschrieben. Der konnte aber groß genug sein, dass
dort noch viele Bytes eines laufenden Telegramms lagen - und dann kam die Quittung zu spät bei der BCU
an.

Dazu kam die verschachtelte Suche: aus ihr war nicht immer klar, wie viele Bytes jetzt sicher
vorliegen. Damit ließ sich auch nicht abschätzen, ob die Quittung noch zu dem Telegramm passt, für das
sie gedacht war. Im schlechtesten Fall wurde ein falsches Telegramm quittiert.

Das war eine bewusste Entscheidung: ein Router soll normalerweise ohnehin alle Telegramme quittieren,
damit hätte eine Verwechslung keine Folgen.

### Das neue Konzept

Mit der neuen Verarbeitung ist es deutlich einfacher geworden. Geprüft wird, wie viele Bytes bereits
vorliegen, und ob das mehr sind, als von diesem Frame noch aussteht. Ist es mehr, ist das Telegramm
auf dem Bus längst durch - dann wird die Quittung im Zweifel weggelassen statt zu spät gesendet.

Und es wird versucht, nie mehr als ein Telegrammbyte im Sendepuffer liegen zu lassen, soweit das
Interface das hergibt. Damit ist die Quittung spätestens nach vier Bytes dran und kommt rechtzeitig.

## Fazit

Der Ablauf ist deutlich einfacher geworden, und damit sollte die Verarbeitung auch deutlich robuster
sein. Etliche Schwachstellen waren im alten Aufbau nicht ohne weiteres zu beheben und wurden
hingenommen, obwohl sie in manchen Situationen zu bekanntem Fehlverhalten führten; sie sind jetzt
weg, zusammen mit einigen, die vorher niemandem aufgefallen waren. Und fehlerhafte Telegramme
lassen sich erkennen - etwas, das mit der Suche gar nicht möglich war.

Dazu kommt der Speicher: die alte Version belegt knapp 13 kB RAM, nach einer ersten Messung sind es
jetzt rund 3 kB.
