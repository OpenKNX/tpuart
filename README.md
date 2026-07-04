# OpenKNX TPUart

OpenKNX TP1 data-link driver for the Siemens **TP-UART 2** (5WG1117-2AB12) and OnSemi
**NCN5120 / 5121 / 5130** bus-coupler chips ("BCU"). It turns a raw UART byte stream into
parsed KNX frames (RX), pushes queued frames onto the TP1 twisted pair (TX), and keeps the
coupler chip connected and self-healed. Part of OpenKNX; consumed by the OpenKNX `knx`
stack as its TP data-link layer.

> This README describes the **stock** architecture. Project-specific changes are OFF by
> default and shown as `[TPUART_*]` overlays (see "Modifications").

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

Byte pump: `stock vTaskDelay(1)/byte` ...... `[TPUART_TX_FAST: taskYIELD]`.
Three ways back from `TX_AWAIT` to `TX_IDLE` (see "Recovery ladder"):

```
  finalize()   CON matched normally
  finalize()   CON-rescue: echo proven but CON lost ............. [always-on]
  reset()      60s watchdog, or fast reset ...................... [TPUART_BCU_BACKSTOP]
```

---

## BCU state machine

States: `BCU_UNINITIALIZED`, `BCU_CONNECTED`, `BCU_DISCONNECTED`, `BCU_BUSMONITOR`.

```
  UNINITIALIZED -> CONNECTED     begin(): write U_RESET_REQ @19200 (+ @38400 for NCN),
                                 wait <= 50 ms for U_RESET_IND    (retry every > 1 s)

  CONNECTED -> DISCONNECTED      no RX byte > 5000 ms
                                 [AUTORECONNECT: AND no valid frame > 30000 ms]

  DISCONNECTED -> CONNECTED      stock          : passive -- any raw byte flips it
                                 [AUTORECONNECT] : active  -- poke U_RESET_REQ, backoff
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
   2  reconnect branch      stock: passive   |   [TPUART_BCU_AUTORECONNECT]: active poke
   3  busy-mode auto-off (700 ms)
   4  keep-alive: U_STATE_REQ every 1000 ms  (skipped while _invalid)
   5  handleReset(): applyConfiguration() + requestState() after a U_RESET_IND
   6  RX drain:   while available() processReceviedByte()
   7  Receiver::process()          loop-driven CON-rescue + RX timeout ...... [always-on]
   8  Transmitter::processQueue()  arm next frame if TX_IDLE
   9  TX pump:    while isTransmitting() processTransmitByte()   (fairness break > 20 ms)
  10  processRxFrameBuffer()       pop frames -> RepetitionFilter -> callbacks
  11  processQueue() again         arm next frame same pass ............. [TPUART_TX_FAST]
  12  surface errors: overflow / U_State / discarded / system-state
  13  liveness check: CONNECTED && stale -> BCU_DISCONNECTED
  14  processWatchdog(): Transmitter 60s cap .................... [+ TPUART_BCU_BACKSTOP]
```

---

## Recovery ladder  (TX stranded in TX_AWAIT, waiting for a CON)

```
  cause                             recovery                           latency
  --------------------------------  -------------------------------    -----------
  CON matched normally              finalize()                         immediate
  echo PROVEN, CON lost/misparsed   CON-rescue finalize()  [always]    RX-quiet
  echo NEVER matched (bus error)    60s watchdog -> reset()            60 s
                                    fast reset  [TPUART_BCU_BACKSTOP]  8 s / 12 s
  NCN mute / desynced               active poke [AUTORECONNECT]        1 s .. 30 s
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

## Modifications  (opt-in overlays; stock = baseline above)

Each flag is **OFF by default**; enabling it changes only the marked spot. Build markers
(`#pragma message` in `TPUart.cpp`) print which are compiled, e.g. `>>> TPUart: TX FAST = ON <<<`.

```
[TPUART_RX_DRAIN_FAST]   ESP32 only
   RX task wakeup     : stock 1 byte/event + vTaskDelay(1)  (~1000 B/s ceiling) fast  drain whole ring/wakeup + taskYIELD  (guard 2048)
   IDF driver ring    : stock 512 B               ->  fast 1024 B
   rx_full_threshold  : 1 in BOTH branches (a lone L_ACK byte must arrive promptly; a high threshold holds it until rx-timeout -> breaks ETS download)

[TPUART_TX_FAST]         ESP32 pump
   byte-pump delay    : stock vTaskDelay(1)/byte  ->  fast taskYIELD  (bytes -> HW TX FIFO)
   next frame         : stock next loop pass      ->  fast same pass after CON (step 11) still strictly one-in-flight (no pipelining past an unconfirmed CON)

[TPUART_BCU_AUTORECONNECT]
   reconnect          : stock passive (waits for a byte a mute NCN never sends -> terminal) ->  active U_RESET_REQ poke, backoff 1s..30s
   reset reply        : U_RESET_IND parsed UNCONDITIONALLY, even while _invalid (the only path back to CONNECTED on a desynced NCN)
   disconnect gate    : stock > 5s no byte        ->  > 5s no byte AND > 30s no valid frame

[TPUART_BCU_BACKSTOP]    verified, RP2040-safe
   case: echo never matched, CON lost, bus quiet -> stock would wait the full 60 s
   fast reset when TX_AWAIT and RX healthy-idle:
      TX_BACKSTOP_MS          = 8000    (> worst-case legit CON: 263 B ext @9600, 3+3 repeats)
      TX_BACKSTOP_INVALID_MS  = 12000   (latched _invalid on a quiet bus)
      TX_BACKSTOP_COOLDOWN_MS = 30000   (persistent fault degrades back to the 60 s cap)
   guard: re-check under rxLock + !available() + RX quiet + re-check after unlock
          -> a late-but-valid CON is processed first, never dropped

[TPUART_BCU_DEBUG]       test / diagnosis only
   forceDisconnect() console hook ("bcu dis"), "CONr: idleBus" trace,
   watchdog give-up dump (rxState / rxTx / _invalid / ...)

ALWAYS ON  (no flag)  -- real reliability fixes; stock default without them is broken/degraded
   CON-rescue          release TX when our echo was proven but the CON was lost (3 sites)
   ESP32 task stack    TPUART_ESP32_TASK_STACK_SIZE = 4096 (stock 2048; recursive parser+VLAs)
   RX-liveness re-arm  _lastReceivedTime set on every reset() (no spurious disconnect)
```

**Where each modification hooks in** -- `◀──` marks the spot; `stock:` is the baseline it
replaces (so you see exactly what changes vs the stock flow above):

```
RX PATH   (ESP32 wakeup -> parser -> deliver frame)
  runTask() wakeup                ◀── [RX_DRAIN_FAST]  drain whole ring + taskYIELD
    │                                 stock: 1 byte + vTaskDelay(1); ring 512 -> 1024
    ▼
  processSearchBuffer() [FSM]     ◀── [AUTORECONNECT]  parse U_RESET_IND
    │                                 unconditionally  (stock: swallowed while _invalid)
    ▼
  processCompleteFrame()          ◀── CON-rescue (always on)  release TX when the echo
                                      was proven but the CON got lost

TX PATH   (queue -> pump -> confirm -> watchdog)
  processTransmitByte() pump      ◀── [TX_FAST]  taskYIELD per byte
    │                                 stock: vTaskDelay(1) per byte
    ▼
  after CON: processQueue()       ◀── [TX_FAST]  arm next frame in the same pass
    │                                 stock: waits for the next loop pass
    ▼
  processWatchdog() (TX_AWAIT)    ◀── [BACKSTOP]  + 8 s / 12 s guarded fast reset
                                      stock: 60 s cap only

CONTROL   (BCU state + bookkeeping)
  BCU_DISCONNECTED reconnect      ◀── [AUTORECONNECT]  active U_RESET_REQ poke,
    │                                 backoff 1 s..30 s  (stock: passive -> terminal)
    ▼
  every reset / error path        ◀── health counters (always on)  resets / disc /
                                      con-rescues + SC/RE/TE/PE/TW, persist across reset()
```

**The RX ring (why `[TPUART_RX_DRAIN_FAST]` matters).** The IDF UART driver fills a ring
buffer from the wire; the FreeRTOS task drains it into the parser. Stock drains 1 byte per
wakeup -- slower than a burst fills it -- so the ring backs up and overflows. Draining the
whole ring each wakeup keeps it empty:

```
IDF UART RX ring  (ESP32): the driver ISR/DMA fills it from the wire,
the FreeRTOS task drains it into the parser.   ( # = byte waiting )

STOCK   (1 byte / wakeup, then vTaskDelay(1);  ring 512 B)
         ┌────────────────────────────────────────┐   <- drain 1 byte / wakeup
         │####################################    │   fill up to ~3490 B/s (38400 baud)
         └────────────────────────────────────────┘   drain only ~1000 B/s
         => ring backs up => OVERFLOW, byte lost => receiver desync

FAST    [TPUART_RX_DRAIN_FAST]  (drain whole ring, then taskYIELD;  ring 1024 B)
         ┌────────────────────────────────────────┐   <- drain ALL  while(available())
         │                                        │   emptied every wakeup
         └────────────────────────────────────────┘   keeps up with the fill
         => no overflow, no ~1000 B/s ceiling
         (rx_full_threshold stays 1: a lone L_ACK byte is delivered at once)
```

**The TX queue (why `[TPUART_TX_FAST]` matters).** The mirror image on the send side: the
consumer fills a frame queue, the byte pump drains it to the bus. Stock drains too slowly
(a `vTaskDelay(1)` self-brake per byte, plus only one frame armed per loop pass), so under a
forward burst the queue backs up and drops frames. `TX_FAST` removes the brake and arms the
next frame in the same pass, draining at wire speed:

```
TX frame queue  (_queue, max 50): the consumer/IP fills it, the pump drains it to
the bus.   ( # = frame waiting )

STOCK   (vTaskDelay(1)/byte pump + next frame on the NEXT loop pass)
         ┌──────────────────────────────┐   <- drain ~33-38 frames/s (self-brake + 1/loop)
         │##############################│   fill: IP->TP forward bursts
         └──────────────────────────────┘
         => backs up => "Ignore frame because transmit queue is full!" => DROPPED

FAST    [TPUART_TX_FAST]  (taskYIELD pump + arm next frame in the SAME pass after CON)
         ┌──────────────────────────────┐   <- drain ~45-50 frames/s (at the 9600-baud TP floor)
         │####                          │   self-brake gone
         └──────────────────────────────┘
         => queue stays low => no "queue full" under normal load
         (the 9600-baud TP wire is still the ultimate cap: a real overload still fills it)
```

**Self-healing reconnect (why `[TPUART_BCU_AUTORECONNECT]` matters).** Stock waits passively
for a byte from the NCN -- a mute / desynced NCN never sends one, so it stays disconnected
forever. The flag actively pokes `U_RESET_REQ` until the NCN answers (and the reply is parsed
even while the receiver is desynced):

```
STOCK   passive reconnect
   [DISCONNECTED] ──── wait for an incoming byte ────▶  a mute / desynced NCN sends
                                                        nothing  =>  STUCK forever
                                                        (terminal "BCU disconnected")

FAST    [TPUART_BCU_AUTORECONNECT]   active poke + unconditional U_RESET_IND parse
   [DISCONNECTED]
        │  U_RESET_REQ  ──────▶  NCN      retry 1s -> 2s -> 4s -> ... capped at 30s
        │  ◀──────  U_RESET_IND  NCN      parsed even while _invalid
        ▼
   [CONNECTED]   self-healed
```

**Fast recovery (why `[TPUART_BCU_BACKSTOP]` matters).** When the TX is stranded in
`TX_AWAIT` and the echo was never matched, only the 60 s watchdog recovers -- a long
forwarding stall. The backstop cuts it to ~8 s, but only when it can prove no transaction is
in flight (so a late-but-valid CON is never dropped):

```
   TX stuck in TX_AWAIT (echo never matched, CON lost)   ── time ──▶

STOCK   60 s watchdog cap only
   0s ├────────────────────────────────────────────────────┤ 60 s ▶ reset()
      (forwarding stalled the whole 60 s)

FAST    [TPUART_BCU_BACKSTOP]
   0s ├─────┤ 8 s ▶ reset()      (12 s if _invalid is latched on a quiet bus)
      guard: TX_AWAIT + RX idle + bus quiet, re-checked under rxLock
             => a late-but-valid CON is finalized first, never dropped
      cooldown 30 s => a persistent fault degrades back to the 60 s cap
```

---

## Flag table (complete)

```
  FLAG                          DEFAULT   SCOPE    EFFECT
  ----------------------------  --------  -------  -----------------------------------------------------------------------------
  TPUART_RX_DRAIN_FAST          off       ESP32    drain whole RX ring/wakeup; ring 1024; threshold 1; removes ~1000 B/s ceiling
  TPUART_TX_FAST                off       ESP32    taskYIELD pump + arm next frame same pass
  TPUART_BCU_AUTORECONNECT      off       all      active U_RESET_REQ reconnect + reset-parse
  TPUART_BCU_BACKSTOP           off       all      fast reset before the 60 s cap (guarded)
  TPUART_BCU_HEALTH             off       all      BCU lifetime health counters + their BCU<Health> / <NCN-Err> line in the `bcu` report. Off = counters are no-op stubs (0 footprint), report omits them.

  tunables (#ifndef, have a default):
  TPUART_ESP32_TASK_STACK_SIZE  4096      ESP32    RX task stack (raised from 2048)
  TPUART_RX_TIMEOUT             5 ms      all      RX inter-byte / quiet-bus window
  TX_BACKSTOP_MS                8000 ms   backstop fast-reset trigger delay
  TX_BACKSTOP_COOLDOWN_MS       30000 ms  backstop after a fast reset only the 60 s cap fires
  TX_BACKSTOP_INVALID_MS        12000 ms  backstop latched-_invalid reset window

  stock switches / tunables:
  TPUART_REPETITION_SIMPLE           off      rep-filter store: 1 entry/source map (stock: bounded LRU list+map, MAX_SIZE 50)
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
Production -- LAN-TP-Base (ESP32 IP-Router)   (matches the field-tested build)
  -D TPUART_BCU_AUTORECONNECT   BCU self-heals (mute/desync) -- real stability
  -D TPUART_RX_DRAIN_FAST       drain RX ring + threshold 1 (ETS app-download works)
  -D TPUART_TX_FAST             higher IP->TP forward rate (no "queue full" under load)
  -D TPUART_BCU_HEALTH          bcu Health/NCN-Err report (field diagnostics, on-demand)
; -D TPUART_BCU_BACKSTOP        optional: lost-CON 60 s stall -> ~8 s (verified, RP-safe)
  always-on (automatic): CON-rescue . task-stack 4096 . threshold 1 . RX-liveness re-arm

Debug / test only -- leave OFF in production
; -D TPUART_BCU_DEBUG           'bcu dis' hook + watchdog/CON diagnostic traces
; -D TPUART_RX_TIMEOUT_DEBUG    print each RX search-buffer timeout mark
```

Build markers (`TPUart.cpp`): an always-on line confirms the local clone is in use
(`==== TPUart: LOCAL lib/TPUart clone in use ====`); each active feature flag prints a
`>>> TPUart: <NAME> = ON <<<` line so the build log states the exact configuration.
