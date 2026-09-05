# Changes


## ec/v1.2.0-beta.1 -- second batch: 2026-08-29

The tag was moved from `fa2eb70` to `f018d90`, so everything below is part of it.

**Signed-char bugs (ESP32 only, RP2040 unaffected)**
* Fix: `readByte()` returned the UART octet as `char`; a `0xFF` byte sign-extended to `-1` and aliased the no-data sentinel, so the byte was dropped
* Fix: CRC16, source/destination address and hex printing read frame bytes signed; any octet `>= 0x80` sign-extended into a wrong checksum, a wrong address or wrong output
* Fix: the receiver control byte was read signed, so the `0xFF`/`0xFE`/`0xFD`/`0xFC` comparisons never matched and the no-op swallow was dead code

**Bus monitor**
* Feature: `TPUART_BUSMON_INTEGRITY` (opt-in) reports every gap in a capture — in monitor mode the chip is transparent (no frame-end marker, no CRC, no byte stuffing, DS Table 11 p.36), so truncated telegrams and poll frames used to vanish into the discard ring
  * frame flags widened to 16 bit (the low 8 were exhausted) and the received length carried along; the widening is unconditional, the reporting is not
  * `rawLength()` returns the octets actually present, so a short carrier is never sized from `size()`
  * a truncated telegram is forwarded with `TRUNCATED`, which sets the cEMI Lost flag (03_06_03 4.1.5.8.1)
  * poll frames are counted out (F0 + 6 header octets + FCS + slots) instead of swept byte by byte; a swept poll body used to stop at the first frame-looking byte and get acknowledged on the bus
  * the chip's per-octet bit error is read from the UART parity bit (DS p.27) into the Bit-error flag; RP2040 takes `DR[11:8]` from 16-bit DMA entries and refuses DMA on a misaligned heap buffer, ESP32 handles `UART_PARITY_ERR`/`UART_FRAME_ERR` instead of dropping the event
  * busmon-only carriers are excluded from the repetition filter, which would read past them via `size()`/`source()`
* Fix: no TX and no ACK while in bus-monitor mode — the transmit queue is skipped
* Fix: a monitored frame with no trailing ACK stranded in `WAIT_ACKN` until the next byte, or was dropped on busmon disconnect; a monitor-gated idle timeout finalizes it

**Reliability**
* Fix: a control byte is never interpreted while the receiver is desynced — `processSearchBuffer()` honoured `U_RESET_IND` at position 0 unconditionally, bypassing the `_invalid` gate. Seen on hardware: a `0x2703` CRC low byte taken for a reset dropped the in-flight frame and the whole transmit queue. A desync watchdog now forces a guarded BCU reset after 20 s latched, with a 60 s cooldown, and stays off in `BUSMONITOR` where a reset would end the capture
* Fix: `U_TPUART2_SET_REPETITION_REQ` wrote a value that collapsed to 0 or 1 — the old expression used logical `||` instead of a bit combination
* Fix: `NORMAL` mode compared the `0x33` constant against `mode()`'s masked 2-bit value, so it could never match
* Fix: two `pop()` calls in one expression had undefined evaluation order; sequenced into locals and read unsigned

**Priority and bus state**
* Fix: medium-access priority is honoured on TP egress — four per-priority FIFOs (system > urgent > normal > low), taken from the frame's own CTRL bits, with headroom reserved so a low-priority burst cannot starve a higher class. An ETS system-priority `T_Connect`/`T_Ack` now reaches the medium instead of queuing behind a backlog; the per-byte TX path is untouched, so throughput is unchanged
* Feature: `busOperational()` for the tunnel heartbeat — `isConnected()` only tracks the host-to-chip link, which stays up on an externally powered NCN when the KNX bus voltage drops. The new call combines the link state with the NCN System State VBUS bit; `SystemState::seen()` makes a non-NCN chip fall back to the link state instead of reporting a false bus-down

**Diagnostics (opt-in, no effect on a normal build)**
* Feature: `OPENKNX_CON_DIAG` counts echo recognition, `WAIT_ACKN`/complete stages, frames dropped on rx-buffer overflow and frames handed to `TX_TRANSMIT`; without the flag the build is byte-identical
* Feature: `TPUART_BCU_MARKER` adds a console `bcu marker on|off` to measure what the chip emits at frame end. Bench use only — the parser does not handle `0xCB`/`0x13`, so the device stops delivering frames while it is on

**Bus load**
* Feature: `Statistics::getRxFrameBits()` adds up the line time every received frame occupied, in bit times per 03_02_02 — 11 bit per character plus 2 bit to the next one, the 15 bit ACK window, the 11 bit ACK character when one was actually on the line, and the 50 bit bus-free time
* Feature: this makes real line occupancy computable, which a byte rate alone cannot express — the ACK and the bus-free gap fall per telegram, not per byte
* Note: the ACK character is taken from the frame ACK flag, not from the local acknowledge bool, which is also true when the CON reports that nobody answered
* Note: `TODO(tpuart-v2)` marks booking the bits by frame timestamp instead of by completion time

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
