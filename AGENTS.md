# AGENTS.md

Leitfaden für KI-Agenten, die in diesem Repository arbeiten. Die Einzelheiten stehen in
Teildateien - lies die passende, bevor du in ihrem Bereich etwas änderst.

## Projekt

`TPUart` ist eine PlatformIO-C++-Library, die den Datalink Layer für die
TP-UART-Kommunikation in KNX-Anlagen umsetzt. Zielplattformen sind RP2040 und ESP32, und
die Interfaces sprechen dort im Wesentlichen das jeweilige Hersteller-SDK an - pico-sdk
(`uart_*`, `dma_*`, `critical_section_t`) beziehungsweise ESP-IDF (`uart_driver_install`
und Verwandte). Vom Arduino-Framework kommen nur die Zeitfunktionen und, im generischen
Adapter `ArduinoSerial<T>`, die `Stream`-artige Klasse des Aufrufers. Welche Cores und
Plattformversionen dafür gezogen werden, steht in `platformio.ini` und nur dort.

In **`docs/datasheets/`** liegen fünf Herstellerdokumente, und sie sind die Autorität für
Protokollzeiten und Opcodes. **Sie sind nicht im Repository und dürfen dort auch nicht hinein**
(`.gitignore`) - lokal können sie also fehlen. Verwiesen wird trotzdem weiter über diese Pfade; fehlt eine
Datei, den Anwender danach fragen, statt die Angabe zu raten oder die Datei einzuchecken:
**`docs/datasheets/Onsemi_NCN5130.pdf`**, **`docs/datasheets/Onsemi_NCN5121.pdf`** und
**`docs/datasheets/Onsemi_NCN5120.pdf`** (OnSemi; 5130 und 5120 laufen auf Testhardware),
**`docs/datasheets/Siemens_TPUART.pdf`** (Siemens TPUART2, technisches Handbuch, 2012) und
**`docs/datasheets/Siemens_TPUART2.pdf`** (Siemens TP-UART **2+**, 2013). Die drei NCN-Varianten sprechen dasselbe
Protokoll, unterscheiden sich aber in den internen Registern (ACR0 Bit 2, ACR1, RevID nur auf 5121/5130 -
siehe `docs/agents/Connection.md`). Die Dokumente sind sich nicht in allem einig - wo sie abweichen, ist das
ein echter Unterschied zwischen den Chips, kein Dokumentationsfehler (siehe
`BcuType`). Zwei Dinge zum 2+ sind wissenswert: seine Servicetabelle ist mit der des
älteren TPUART2 identisch (dieselben Grenzen `81-BE` / `47-7F`, kein
`U_L_DataOffset`), seine **Host-Baudrate ist aber 115200 oder 19200, ausgewählt über
den BDS-Pin** - nicht 38400. `begin(BcuType::Tpuart2)` probiert nur 19200, ein auf
115200 gestrappter 2+ würde also nie erkannt.

## Teildateien

Bewusst nicht per `@` eingebunden, damit sie nur bei Bedarf in den Kontext kommen.

| Datei | Inhalt |
|---|---|
| `docs/agents/Interface.md` | UART-Interfaces: `Abstract`, `RP2040`, `ESP32`, `ArduinoSerial`, `Dummy` |
| `docs/agents/DataLinkLayer.md` | Aufbau des DataLinkLayer, `tick()`/`loop()`, Ringpuffer, Frame-Flags, Zustände, Nebenläufigkeit |
| `docs/agents/Timer.md` | Tick-Antrieb, Timer-Singleton, IRQ-Priorität, `Timer::trigger()` |
| `docs/agents/Connection.md` | Verbindungsaufbau, Lesen der internen Register, Konfigurationsepoche, Verbindungsüberwachung, `SystemState` |
| `docs/agents/Control.md` | Steuerbefehle, Busmonitor, Meldungen |
| `docs/agents/Receiver.md` | Quittung, Steuerbytes, Telegrammgröße/CRC/Pause, `Frame`, Wiederholungsfilter, Puffergröße |
| `docs/agents/Transmitter.md` | Sendewarteschlange, Prioritäten, Sendeablauf, Echo, Wachhund |
| `docs/agents/Statistics.md` | Zähler, Taktmessung, Flash-Stillstand, Buslast |
| `docs/agents/Build.md` | Build-Umgebungen, Tests, `library.json` |
| `docs/agents/Api.md` | Oberfläche für knx, OGM-Common und OFM-Network, `KOMPAT` |
| `docs/agents/History.md` | Zurückgestellte Entscheidungen, frühere Fehler, offene Punkte |

Immer gilt: nach einer Änderung an der öffentlichen Oberfläche einen Verbraucher bauen (`docs/agents/Api.md`),
und vor einer Freigabe zählen die Hardwareläufe, nicht `native_test` allein (`docs/agents/Build.md`).

## Codestil

- **Jede Membervariable trägt das Präfix `_`** - ausnahmslos, einschließlich der Felder einfacher
  Datenstrukturen (`Dummy::QueuedByte`, `TPUartDmaCompleteRegistration` in `RP2040.cpp`), nicht nur
  gekapselter Klassenzustand.
- Kommentare sind auf Deutsch, Bezeichner auf Englisch; diese Datei ist auf Deutsch.
- STL und Heap-Allokation grundsätzlich meiden. Drei Ausnahmen, alle bewusst: `std::function` für die
  Callbacks und `malloc`/`free` für die Telegramme der Sendequeue (beides ausdrückliche Entscheidungen
  des Anwenders - siehe `docs/agents/Transmitter.md`), dazu `std::vector`/`std::string` innerhalb des
  **`Dummy`-Interfaces und in `Frame::printFrame()`**, was Test- bzw. Diagnosecode ist und nicht
  Datenpfad. In der Schicht selbst gibt es keinen STL-Container, und der Heap wird nur aus dem
  Hauptkontext angefasst.

## Arbeitsweise in diesem Projekt

Der Anwender treibt Entwurfsentscheidungen Schritt für Schritt und will mehrdeutige Verzweigungen
ausdrücklich vor der Umsetzung hinterfragt haben (Rückfragen bündeln statt anzunehmen - ähnlich der
"grill-me"-Methode: einen Entscheidungsbaum der offenen Zweige aufbauen, alles derzeit Beantwortbare
in einem Schwung fragen, eigenständig recherchierbare technische Tatsachen selbst klären statt zu
fragen, und Zweig für Zweig weitermachen, bis nichts mehr offen ist). Wenn etwas wirklich mehrdeutig
oder eine Abwägung ist, frage; wenn es eine sachliche/technische Frage ist, die du selbst prüfen kannst
(z.B. ob eine Plattform eine API unterstützt), recherchiere sie und berichte das Ergebnis, statt den
Anwender nachschlagen zu lassen.
