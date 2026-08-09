# Changes

## ec/v1.2.0-beta.1: 2026-08-09

Enhanced TPUart release. All perf/stability behaviours below are **on by default**; the opt-in
switches (`TPUART_NCN_TW_AUTORESET`, `TPUART_BCU_DEBUG`) and the `#ifndef` tunables are documented in
the README. Includes all 1.1 fixes (listed below).

**Reliability**
* Feature: BCU self-healing / auto-reconnect — on a mute or desynced NCN the driver actively pokes `U_RESET_REQ` (backoff 1 s → 30 s) until it answers; the `U_RESET_IND` reply is parsed unconditionally even while the receiver is `_invalid` (the only path back to `CONNECTED` on a desync). Disconnect needs BOTH no byte for 5 s AND no valid frame for 30 s, so a busy burst cannot cause a spurious mid-burst disconnect. Ends the terminal "BCU disconnected".
* Feature: CON-rescue (3 sites) — release the transmitter when our own bus-echo was proven but the `L_Data.con` was lost/misparsed, instead of stalling in `TX_AWAIT`.
* Feature: Backstop — a guarded fast reset (~8 s, or 12 s if `_invalid` is latched on a quiet bus) for the residual lost-CON stall instead of the 60 s watchdog. Re-checked under `rxLock` with the bus quiet, so a late-but-valid CON is finalized first, never dropped; a 30 s cooldown degrades a persistent fault back to the 60 s cap.
* Feature: NCN thermal-warning handling — L1 edge-logging is always on (the old 1/s "Temperature-Warning" flood becomes one line on set, an optional heartbeat, one on clear); opt-in L2 auto-heal (`TPUART_NCN_TW_AUTORESET`) issues one guarded `U_RESET` on a latched TW and backs off if it stays hot.
* Fix: rate-limit the `U_State` transient-error log — a bus/NCN scan no longer floods the console with "TP Error: Protocol-Error".
* Fix: bound the RX-frame stack array (was a VLA sized from an untrusted length header) — stack-overflow guard.

**Throughput (ESP32)**
* Feature: RX drain — the RX task drains the whole IDF UART ring per wakeup (then `taskYIELD`), ring raised to 1024 B, `rx_full_threshold` kept at 1. Removes the ~1000 B/s drain ceiling; ETS app-download works reliably.
* Feature: TX fast — `taskYIELD` byte-pump (bytes go to the HW TX FIFO) + arm the next queued frame in the same `process()` pass after the CON, still strictly one-in-flight. Higher IP→TP forward rate, no "transmit queue full" under load.
* Feature: TX sticky data-offset — send `U_L_DataOffset` only when the offset changes (the NCN keeps it internally, DS p.42): 4 offset bytes instead of 199 on a 263-octet frame, +14 % measured. Frames ≤ 64 octets are byte-identical.

**Diagnostics**
* Feature: BCU health counters — lifetime resets / disconnects / con-rescues / temperature-warnings + `SC/RE/TE/PE`, shown in the `bcu` report; persist across `reset()` (cleared only on reboot).
* Feature: NCN register snapshot — read the Revision-ID and ASR0 at boot for the `bcu stat` chip/rails section (NCN5120 / 5121 / 5130).
* Feature: `getBaudrate()` accessor — the last negotiated BCU baudrate (19200 / 38400).

**HW busmonitor**
* Feature: forward standalone L2 acknowledges to the busmon in monitor mode.
* Feature: forward FCS-failed frames raw to the busmon in monitor mode (tagged as errored).

**Platform** (experimental — these tuning values may still be re-evaluated / re-tuned)
* Feature: RP2040 DMA RX ring 256 → 2048 B (`BUFFER_EXP` 8 → 11) — no corruption on > 4 KB fast transfers.
* Feature: ESP32 RX-task stack 2048 → 4096 (the recursive search-buffer parser + VLAs need the headroom).

**Change**
* Change: the six proven perf/stability behaviours above (auto-reconnect, RX-drain, TX-fast, sticky-offset, backstop, health counters) are now the **default** — the opt-in build flags and their stock fallbacks were removed. Still opt-in: `TPUART_NCN_TW_AUTORESET`, `TPUART_BCU_DEBUG`. Behaviour is identical to the field-tested "all flags on" build.

## 1.1: 2026-07-30

* Change: Send `U_L_DataOffset` only when the offset actually changes instead of before every byte. The BCU keeps the offset internally until it is changed, so of ~199 offset bytes in a 263-octet frame only ~4 actually change it. Frames of 64 octets or less are unaffected (offset is always 0 there). (@erkanc)
* Fix: Extended frames with a LEN octet > 127 were unreceivable. `apduSize()`/`metadataSize()` returned signed `char`, so on signed-char targets (ESP32) the value sign-extended negative and wrapped the arithmetic in `size()`. (@erkanc)
* Fix: `processRxFrameBuffer()` sized a stack array from an untrusted length header; a `bufferSize < 3` underflow wrapped it to ~65533. The length is now checked against the search buffer size, and a mismatch is logged and resets the BCU instead of being papered over. (@erkanc)
* Fix: `Transmitter::_state` is now `volatile` — written by `finalize()` on the RX path, read by the main-loop TX path, matching `Receiver::_state`. (@erkanc)
* Fix: `reset()` did not re-arm `_lastReceivedTime`, so a reset on a bus that had been idle for more than 5 s made the very next `process()` report a spurious disconnect.
* Fix: On `U_RESET_IND` the transmitter kept its position and would have continued mid-frame, putting the tail of a telegram on the wire as if it were a whole one. The transmitter is now reset fully (queue, in-flight frame, offset).
