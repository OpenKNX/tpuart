# OpenKNX TPUart

OpenKNX TP1 data-link driver for the Siemens **TP-UART 2** (5WG1117-2AB12) and OnSemi
**NCN5120 / 5121 / 5130** bus-coupler chips ("BCU"). It turns a raw UART byte stream into
parsed KNX frames (RX), pushes queued frames onto the TP1 twisted pair (TX), and keeps the
coupler chip connected and self-healed. Part of OpenKNX; consumed by the OpenKNX `knx`
stack as its TP data-link layer.

> Perf/stability behaviours (RX-drain, TX-fast, sticky-offset, auto-reconnect, backstop, health
> counters) are built into the driver; see "Behaviour details". Opt-in switches are marked `[TPUART_*]`.

---

## System overview

```
┌────────────────────────────────────────────────────────────────┐
│ Consumer   (OpenKNX knx stack)                                 │
│ - registerReceivedFrame()    - registerCheckAcknowledge()      │
│ - pushTransmitQueue()        - process()  (every main loop)    │
└────────────────────────────────┬───────────────────────────────┘
                                 │   frames up   /   send + control down
                                 ▼
┌────────────────────────────────────────────────────────────────┐
│ TPUart::DataLinkLayer    -- control plane / orchestrator       │
│ - BCU state machine + reset() / U_RESET handshake              │
│ - process() loop: RX drain, TX pump, watchdog, keep-alive      │
│ owns 4 sub-components:                                         │
└──────┬────────────────┬────────────────┬────────────────┬──────┘
       │                │                │                │
       ▼                ▼                ▼                ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│ Receiver    │  │ Transmitter │  │ SystemState │  │ Statistics  │
│ RX parser   │  │ TX queue +  │  │ NCN status  │  │ bus load +  │
│ bytes->frm  │  │ pump + WD   │  │ rail/temp   │  │ BCU health  │
└─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘
       │                │     Receiver reads bytes / Transmitter writes bytes
       ▼                ▼
┌────────────────────────────────────────────────────────────────┐
│ Interface::Abstract   -- UART backend (one per platform)       │
│ - ESP32   : FreeRTOS task + IDF ring    (READ-time stamp)      │
│ - RP2040  : per-byte RX ISR             (ARRIVAL-time stamp)   │
│ - ArduinoSerial (polled)   /   Dummy (test/replay)             │
└────────────────────────────────┬───────────────────────────────┘
                                 │   UART @ 19200 / 38400 baud
                                 ▼
┌────────────────────────────────────────────────────────────────┐
│ NCN5120 / TP-UART2    bus coupler chip ("BCU")                 │
│ - puts frames on TP1, echoes our TX frames back on UART        │
│ - appends L_Data.con, reports U_State (SC/RE/TE/PE/TW)         │
└────────────────────────────────┬───────────────────────────────┘
                                 │
                                 ▼
┌────────────────────────────────────────────────────────────────┐
│ KNX TP1 twisted-pair bus    (9600 baud on the wire)            │
└────────────────────────────────────────────────────────────────┘
```

---

## Public API  (class `TPUart::DataLinkLayer`)

```
  lifecycle / loop        begin(BcuType, Interface*)   end()   process()   reset()
  register callbacks      registerReceivedFrame(fn)    registerCheckAcknowledge(fn)
                          registerMessage(fn)
  send                    pushTransmitQueue(Frame*)          // strictly one-in-flight
  chip / mode             setOwnAddress()  setRepetitions()  startMonitoring()
                          powerControl()  stopMode()  busyMode()      // NCN5120 only
  introspection           isConnected()  getBcuState()  getBcuStateInfo()
                          getStatistics()  getSystemState()  getReceiver()  getTransmitter()
```

Typical consumer loop:

```
  begin(BCU_NCN5120, iface);
  registerReceivedFrame(onFrame);           // called with each parsed Frame&
  registerCheckAcknowledge(shouldAck);      // decide ACK/NACK/BUSY per destination
  for (;;) { process(); /* ...other work... */ }   // process() drives RX + TX + watchdog
  pushTransmitQueue(frame);                 // enqueue outbound frame
```

**BcuType** (`Types.h`): `BCU_TPUART2` (Siemens 5WG1117-2AB12) vs `BCU_NCN5120` (OnSemi
NCN5120 / 5121 / 5130). They diverge in: baudrate scan (NCN auto / TPUART2 19200 only),
CRC config, set-address/repetition opcodes, max frame size (NCN 263 B /
**TPUART2 rejects > 64 B**), and system-state + power/stop-mode (NCN only).

---

## RX path :  byte -> frame

Code path: `Interface -> processReceviedByte() -> pushSearchBuffer() ->
processSearchBuffer() [FSM] -> processCompleteFrame() -> pushRxFrameBuffer() ->
RepetitionFilter -> registerReceivedFrame() callbacks`.

The parser is a **byte-accumulator**: each `RxState` waits for its bytes, then advances.
Frame boundaries are found by **length**, not by a delimiter.

```
┌──────────────────────┐
│ RX_IDLE              │  waiting for a frame-start control byte
│                      │  (ctrl & 0xD3 == 0x90 std / 0x10 ext)
└───────┬──────────────┘
        │  frame-start byte
        ▼
┌──────────────────────┐
│ RX_FRAME             │  collect control + source + destination  (6 B)
└───────┬──────────────┘
        │  6 B in
        ▼
┌──────────────────────┐
│ RX_FRAME_DESTINATION │  read the length field -> full frame size now known
└───────┬──────────────┘
        │  size in
        ▼
┌──────────────────────┐
│ RX_FRAME_SIZE        │  CRC8 check
│                      │  bad -> discard 1 B, _invalid, back to RX_IDLE
└───────┬──────────────┘
        │  CRC ok
        ▼
┌──────────────────────┐
│ RX_FRAME_WAIT_ACKN   │  wait 1 tail byte = CON (0x0B) / L_ACKN
└───────┬──────────────┘
        │  CON byte in
        ▼
┌──────────────────────┐
│ RX_FRAME_COMPLETE    │  deliver frame up-stack, then back to RX_IDLE
└──────────────────────┘

  escape: no byte for TPUART_RX_TIMEOUT = 5 ms -> WAIT_ACKN force-completes (missing CON);
          any other stage discards 1 B and re-syncs
```

`_invalid` = "receiver desynced -> discard bytes until a real frame start reappears";
cleared on a clean completed frame or on `reset()`.

---

## Interface timing model  (why RX behaviour is hardware-specific)

```
┌──────────────────────────┐
│ ESP32                    │  FreeRTOS task + IDF ring buffer
│                          │  wire -> IDF ISR/DMA -> [ring] -> runTask() -> _callback()
│                          │  -> processReceviedByte() -> _lastReceivedTime = millis()
│                          │  => READ-time stamp (a queue sits between arrival and read)
└──────────────────────────┘

┌──────────────────────────┐
│ RP2040                   │  per-byte RX ISR (optional DMA)
│                          │  wire -> RX ISR -> interrupt() -> _callback()
│                          │  -> processReceviedByte() -> _lastReceivedTime = millis()
│                          │  => ARRIVAL-time stamp (the ISR parses directly)
└──────────────────────────┘
```

Frames are delimited by **length**, not by this clock. `_lastReceivedTime` is the single
"last byte read" timestamp; it only drives the **5 ms inter-byte idle-timeout** (plus
liveness / quiet-bus checks). On the ESP32 the ring+task queue means that idle-timeout is
measured in read-time and can drift from true arrival under load -> a false gap force-
completes or re-syncs a frame -> our TX echo's trailing `L_DATA_CON` gets misparsed and
lost. The RP2040 reads straight from the RX ISR (read == arrival) and has no such drift.
This is why several fixes below are ESP32-only.

---

## TX path :  frame -> bus  (round-trip via the echo)

Code path: `pushTransmitQueue() -> Transmitter::pushQueue() [_queue] -> processQueue()
-> byte pump (processTransmitByte) -> NCN echo -> RX matches it -> L_Data.con ->
finalize()`.

An outbound frame is a **round-trip**: the NCN puts it on the bus AND echoes it back on
the UART -- our own RX parser sees it -- then appends the confirm (CON) that releases TX.

```
┌──────────────────────┐
│ TX_IDLE              │  processQueue(): pop next frame from [_queue] (max 50)
└───────┬──────────────┘
        │  armed  [TX_IDLE && !_invalid]
        ▼
┌──────────────────────┐
│ TX_TRANSMIT          │  byte pump -> NCN -> frame goes out on the TP1 bus
│                      │  (write U_L_DATA_START / END / OFFSET, per byte)
└───────┬──────────────┘
        │  last byte -> resetWatchdogTimer()
        ▼
┌──────────────────────┐
│ TX_AWAIT             │  NCN echoes the whole frame back  (RX: setTransmitted),
│                      │  then appends L_Data.con  -> back to TX_IDLE
└──────────────────────┘
```

Byte pump: `taskYIELD` per byte, bytes go to the HW TX FIFO (ESP32).
Three ways back from `TX_AWAIT` to `TX_IDLE` (see "Recovery ladder"):

```
  finalize()   CON matched normally
  finalize()   CON-rescue: echo proven but CON lost
  reset()      guarded fast reset (~8 s), else the 60 s watchdog cap
```

---

## BCU state machine

States: `BCU_UNINITIALIZED`, `BCU_CONNECTED`, `BCU_DISCONNECTED`, `BCU_BUSMONITOR`.

```
  UNINITIALIZED -> CONNECTED     begin(): write U_RESET_REQ @19200 (+ @38400 for NCN),
                                 wait <= 50 ms for U_RESET_IND    (retry every > 1 s)

  CONNECTED -> DISCONNECTED      no RX byte > 5000 ms AND no valid frame > 30000 ms

  DISCONNECTED -> CONNECTED      active reconnect: poke U_RESET_REQ, backoff
                                 1 s -> 30 s, until a real U_RESET_IND

  CONNECTED -> BUSMONITOR        startMonitoring(): write U_BUSMON_REQ    (one-way)
```

**`reset()` ("Reset BCU")**: transmitter reset, clear uState / receiver / frame-buffer /
rep-filter, flush UART, state -> CONNECTED, write `U_RESET_REQ`, re-arm RX liveness. Health
counters are **not** wiped. `applyConfiguration()` runs afterwards, once the NCN answers
with `U_RESET_IND`.

---

## process()  --  one main-loop pass

```
   1  (re)initialize if UNINITIALIZED
   2  reconnect branch      active U_RESET_REQ poke while DISCONNECTED
   3  busy-mode auto-off (700 ms)
   4  keep-alive: U_STATE_REQ every 1000 ms  (skipped while _invalid)
   5  handleReset(): applyConfiguration() + requestState() after a U_RESET_IND
   6  RX drain:   while available() processReceviedByte()
   7  Receiver::process()          loop-driven CON-rescue + RX timeout
   8  Transmitter::processQueue()  arm next frame if TX_IDLE
   9  TX pump:    while isTransmitting() processTransmitByte()   (fairness break > 20 ms)
  10  processRxFrameBuffer()       pop frames -> RepetitionFilter -> callbacks
  11  processQueue() again         arm next frame in the same pass after the CON
  12  surface errors: overflow / U_State / discarded / system-state
  13  liveness check: CONNECTED && stale -> BCU_DISCONNECTED
  14  processWatchdog(): guarded fast reset (~8 s), else the 60 s cap
```

---

## Recovery ladder  (TX stranded in TX_AWAIT, waiting for a CON)

```
  cause                             recovery                           latency
  --------------------------------  -------------------------------    -----------
  CON matched normally              finalize()                         immediate
  echo PROVEN, CON lost/misparsed   CON-rescue finalize()              RX-quiet
  echo NEVER matched (bus error)    guarded fast reset, else watchdog  8 s / 12 s / 60 s
  NCN mute / desynced               active U_RESET_REQ poke            1 s .. 30 s
```

**NCN U_State error bits** (surfaced by `showStateError()`, counted in `Statistics`):

```
  0x80 SC Slave-Collision    0x40 RE Receive-Error        0x20 TE Transmit-Error
  0x10 PE Protocol-Error     0x08 TW Temperature-Warning
```

**Hardware latch -- NOT software-recoverable.** The ladder above heals *software* desyncs.
A soft reboot re-inits the ESP UART but does **not** reset the NCN5120 or the LAN PHY
(separate chips on their own supply). If a chip latches -- e.g. a floating / poorly-grounded
USB cable or supply corrupts the NCN receive reference -- neither auto-reconnect nor a reboot
recovers it; **only a full power-cycle** does. Field symptom: GA reads *and* device-info
reads both dead, a reboot changes nothing, and everything works again only after power is
fully removed. This is a wiring problem, not a firmware one -- no floating USB, isolate the
ESP/NCN supply.

---

## Behaviour details

Perf/stability behaviours built into the driver (opt-in ones are marked):

```
RX drain       (ESP32)
   The RX task drains the whole IDF UART ring per wakeup (guard 2048), then taskYIELD. Driver ring
   1024 B; rx_full_threshold 1 (a lone L_ACK byte is delivered at once -- a high threshold would hold
   it until the rx-timeout and break ETS app-download).

TX fast        (ESP32 pump)
   Byte pump uses taskYIELD; bytes go to the HW TX FIFO. The next queued frame is armed in the same
   pass after the CON (step 11), still strictly one-in-flight (no pipelining past an unconfirmed CON).

Sticky offset  (all platforms; only affects frames > 64 octets)
   U_L_DataOffset is sent only when the offset changes. The NCN5130 stores it internally until a new
   offset is provided (DS p.42), so a 263-octet frame sends 4 offset bytes instead of 199 -- 195 of
   725 UART bytes, +14% throughput (RP2350, pkg 253). It costs full time because the chip starts the
   bus transmission only once the whole frame has arrived (DS p.49) -- UART time adds to bus time.
   <= 64 octets: nothing emitted (offset 0), group telegrams untouched. The mid-frame 0x80
   (U_L_DataStart.req at each 64-B block) does NOT reset the offset (DS p.49 = new frame only;
   verified across all four boundaries -- expected CRC, 0 retries).

Auto-reconnect (all platforms)
   On a mute/desynced NCN the driver actively pokes U_RESET_REQ (backoff 1 s -> 30 s) until it answers;
   the U_RESET_IND reply is parsed UNCONDITIONALLY, even while _invalid (the only path back to CONNECTED
   on a desynced NCN). Disconnect gate: > 5 s no byte AND > 30 s no valid frame.

Backstop       (all platforms, RP2040-safe)
   TX stranded in TX_AWAIT with the echo never matched and the bus idle: a guarded fast reset fires at
   TX_BACKSTOP_MS = 8000 (TX_BACKSTOP_INVALID_MS = 12000 if _invalid is latched on a quiet bus) instead
   of the 60 s watchdog. Guard: re-check under rxLock + !available() + RX quiet, re-checked after unlock
   -> a late-but-valid CON is processed first, never dropped. Cooldown TX_BACKSTOP_COOLDOWN_MS = 30000:
   a persistent fault degrades back to the 60 s cap.

Health counters (all platforms)
   Lifetime BCU health counters (resets / disconnects / con-rescues / TW + SC/RE/TE/PE), shown in the
   `bcu` report; persist across reset() (cleared only on reboot).

CON-rescue     (all platforms)
   Releases TX when our echo was proven but the CON was lost (3 sites).

BCU debug      (opt-in: TPUART_BCU_DEBUG)
   forceDisconnect() console hook ("bcu dis"), CONr trace, watchdog give-up dump.
```

Also always on: ESP32 task stack `TPUART_ESP32_TASK_STACK_SIZE` = 4096 (recursive parser + VLAs),
RX-liveness re-arm (`_lastReceivedTime` set on every reset(), no spurious disconnect).

---

## Flag table (complete)

```
  FLAG                          DEFAULT   SCOPE    EFFECT
  ----------------------------  --------  -------  -----------------------------------------------------------------------------
  TPUART_NCN_TW_AUTORESET       off       all      NCN thermal-warning auto-heal (L2): when TW persists, issue ONE guarded U_RESET (like `bcu rst`) to clear a latched TW, then back off if it returns (genuinely hot). NOTE: L1 edge-logging of TW is ALWAYS on regardless of this flag -- the old 1/s "TP Error: Temperature-Warning" flood becomes one line on set, an optional heartbeat, one on clear.

  tunables (#ifndef, have a default):
  TPUART_ESP32_TASK_STACK_SIZE  4096      ESP32    RX task stack (raised from 2048)
  TPUART_RX_TIMEOUT             5 ms      all      RX inter-byte / quiet-bus window
  TX_BACKSTOP_MS                8000 ms   backstop fast-reset trigger delay
  TX_BACKSTOP_COOLDOWN_MS       30000 ms  backstop after a fast reset only the 60 s cap fires
  TX_BACKSTOP_INVALID_MS        12000 ms  backstop latched-_invalid reset window
  TPUART_TW_HEARTBEAT_MS        60000 ms  TW: min spacing of the "still set" heartbeat while it holds
  TPUART_TW_AUTORESET_AFTER_MS  45000 ms  TW auto-heal: let TW persist this long before the U_RESET
  TPUART_TW_AUTORESET_COOLDOWN_MS 300000  TW auto-heal: min spacing between two auto-resets (cross-episode backoff)
  TPUART_TW_RELAPSE_MS          8000 ms   TW still set this long after the reset => genuinely-hot verdict

  other switches / tunables:
  TPUART_REPETITION_SIMPLE           off      rep-filter store: 1 entry/source map (default: bounded LRU list+map, MAX_SIZE 50)
  TPUART_MAX_PROCESS_TIME_PER_LOOP   30       per-loop process budget
  TPUART_MAX_RXQUEUE_TIME_PER_LOOP   20       per-loop RX-frame-buffer drain budget
  TPUART_RX_FRAME_BUFFER_SIZE        6000     RX frame output ring (bytes)
  TPUART_RX_SEARCH_BUFFER_SIZE       300      parser search window (263 + slack)
  TPUART_RP2040_BUFFER_SIZE / _EXP   256 / 8  RP2040 DMA ring geometry

  debug:
  TPUART_BCU_DEBUG              off       all      test hooks + diagnostic traces
  TPUART_RX_TIMEOUT_DEBUG       off       all      print each RX search-buffer timeout mark
  TPUART_PRINT                 off       all      frame hex/string print helper (Frame.h) -- dormant: its #ifdef is commented out in source
```

## Recommended presets

```
Production -- LAN-TP-Base (ESP32 IP-Router/IP-Interface)   (matches the field-tested build)
  -D TPUART_NCN_TW_AUTORESET    optional: NCN thermal-warning auto-heal
  built in (automatic): RX-drain . TX-fast . sticky-offset . auto-reconnect . backstop . health counters
                        . CON-rescue . task-stack 4096 . threshold 1 . RX-liveness re-arm

Debug / test only -- leave OFF in production
; -D TPUART_BCU_DEBUG           'bcu dis' hook + watchdog/CON diagnostic traces
; -D TPUART_RX_TIMEOUT_DEBUG    print each RX search-buffer timeout mark
```

Build markers (`TPUart.cpp`): only the still-optional switches print a `>>> TPUart: <NAME> = ON <<<`
line (currently `TPUART_BCU_DEBUG`) so the build log states which opt-in features are compiled.
