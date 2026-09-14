# Api.md - Die API gehört den Verbrauchern

Teil von `AGENTS.md`.

## Die API gehört den Verbrauchern

Drei Projekte binden diese Library ein - `knx/src/knx/tpuart_data_link_layer.{h,cpp}`, `OGM-Common`
(`Console.cpp`, `Common.cpp`, `Hardware.cpp`) und `OFM-Network` (`TelegramJson.cpp`, `Module.cpp`,
`Webserver/GroupMonitor.cpp`). Sie müssen übersetzen, ihre Aufrufe legen damit einen Teil der
Oberfläche fest. Drei Gruppen, und nur die erste ist schuldenfrei:

1. **Echt und bleibt.** `TPUart.h` (Sammelheader), **mehrere Telegramm-Callbacks**
   (`registerFrameCallback()` hängt an - der IP-Router hat **vier** Zuhörer: den Empfangspfad des
   knx-Stacks, `bcu debug` von OGM-Common, den MQTT-Publisher von OFM-Network und dessen
   Gruppenmonitor; mit nur einem Platz hätte der letzte alle davor stillschweigend abgeschaltet),
   `Frame::data()` mit Rückgabetyp **`const char *`** statt `const uint8_t *` (OFM-Network schreibt
   `const char *d = frame.data()`; da der Rückgabetyp nicht überladbar ist, heißt der Schreibzugriff
   für den, der ein Telegramm füllt, `buffer()`), `Frame::cemiData()`/`cemiSize()` (der Stack braucht
   die cEMI-Sicht; `cemiData()` liefert einen `malloc`-Puffer, **den der Aufrufer freigeben muss**),
   und die Klassennamen `ESP32`/`RP2040` samt Dateinamen (`ESP32.h`, `RP2040.h` - ein `#include`
   unterscheidet unter Linux Groß- und Kleinschreibung).
2. **Im Code mit `KOMPAT` markiert, entfernbar, sobald die Verbraucher nachgezogen sind.** Doppelte
   Schreibweisen (`isMonitoring()`, `registerReceivedFrame()`, `getBcuStateInfo()`, `BcuType::BCU_*`,
   `AcknowledgeType`/`ACK_*`, `Statistics::getRxDiscardedBytes()` und Verwandte) und zusätzliche
   Einstiegspunkte (`DataLinkLayer()` ohne Interface plus `begin(type, interface*)`, `process()` - heute nur noch `loop()`, früher `tick()` + `loop()` -, `end()`, `pushTransmitQueue(Frame*)`, das bei Erfolg den Besitz übernimmt und
   die Länge um eins kürzt, weil die Library die Prüfsumme selbst rechnet) und `Frame(const char*,
   size_t)`.
3. **Platzhalter aus der Zeit des SearchBuffers**, den es hier nicht mehr gibt.
   `Receiver::getSearchBufferPosition()` liefert den Füllstand des Empfangspuffers und
   `Receiver::getAwaitBytes()` die noch ausstehenden Bytes des laufenden Telegramms (0, solange die
   Größe noch nicht feststeht) - beide statt der früheren konstanten 0.
   **`OGM-Common`s `bcu`-Befehl druckt sie inzwischen nicht mehr**; sie stehen damit ohne Aufrufer da
   und bleiben nur auf ausdrückliche Entscheidung des Anwenders erhalten, nicht mehr aus einem
   Kompatibilitätszwang. Wer hier aufräumt, kann sie streichen, sobald das bestätigt ist.
   Bei **0** bleibt allein `Statistics::getRxSearchBufferOverflow()` - dafür gibt es in dieser
   Library nichts Vergleichbares, jeder Ersatzwert wäre eine Falschaussage. Die Parameter `irq`/`dma`
   des `RP2040`-Konstruktors werden aus demselben Grund ignoriert wie früher: es gibt nur den
   DMA-Pfad.

**Nichts in diesem Repository fasst diese Oberfläche an**, kein Test und kein Beispiel - die Gruppen
2 und 3 werden also nirgends übersetzt. Ein Bruch fällt deshalb erst auf, wenn eines der drei Projekte
gegen diese Library gebaut wird. Hier stand einmal eine Datei `src/compat_check.cpp`, die genau dafür
jeden Aufruf der Verbraucher einmal ausschrieb; sie ist entfernt, weil sie eine handgepflegte Kopie
war und der Bau des Verbrauchers die eigentliche Prüfung ist. Wie leicht es passiert, zeigt der eine
Fund, den es dort gab: OFM-Network war gar nicht erfasst, bis der IP-Router zum ersten Mal gegen diese
Library gebaut wurde - und die Signatur von `data()` brach sofort.
**Baue nach einer Änderung an der Oberfläche einen Verbraucher, bevor du sie für fertig hältst.**
Der schnellste ist **`OAM-TestApp`**: dort liegen die OpenKNX-Repositories als relative Symlinks in `lib/`,
diese Library gehört als `lib/tpuart -> ../../tpuart` dazu (unter Windows `mklink /D`, und relativ wie die
übrigen Einträge). Damit übersetzt ein Bau der TestApp `knx`, `OGM-Common` und `OFM-Network` gegen den
Arbeitsstand hier - genau die drei Verbraucher, deren Aufrufe die Gruppen 2 und 3 festlegen.
