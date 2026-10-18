# Changes


## ec/1.4.0-beta.2: 2026-10-18

One busmonitor fix on top of the previous beta.

**Busmonitor**
* Fix: a configuration indication is ignored while the chip monitors the bus. At `U_Busmon.req` the chip still delivers the frame in flight in normal format with its two CRC octets; a CRC octet such as `0x79` was read as `U_Configure.ind` with the CRC bit and switched the receiver to CRC frames, so every following busmonitor frame waited for two octets that never come and was swept until the session ended. Measured on an NCN5130: 220-235 of about 840 octets discarded and the Lost bit on nearly every indication in affected sessions; with the fix, 0 of 24 sessions affected. Pinned by `HT-1.23` of the host stack harness in OAM-IP-Interface, red without the guard
* Fix: `startMonitoring()` sets the busmonitor state before it clears the CRC flag, so an indication parsed between the two cannot leave the flag set
* Known limit: the same window can deliver a `0x03`, which reads as `U_Reset.ind`, returns the state to connected and ends the busmonitor session; a real chip reset sends the same octet, so it is not ignored


## ec/1.4.0-beta.1: 2026-10-11

Receive-path work, centred on the idle gap that decides where a telegram ends, plus the repetition
filter and two signedness defects. The gap fixes are pinned by `HT-1.21` and `HT-1.22` of the host
stack harness in OAM-IP-Interface, each proven red without its guard, and confirmed on hardware
(RP2350 and NCN5130 against a NeoPixel device).

A second batch adds an interrupt-driven transmitter on RP2040, keeps a host acknowledge inside the time the
shortest frame leaves, keeps the confirm wait across probe answers and repetitions, reports a silent chip as a
bus fault, and collects the build knobs in `src/TPUart/Config.h`. The confirm fix is measured on hardware with bursts
of 1000 routing indications from IP to TP: before it, 0 to 4 frames per burst neither reached the bus nor were
reported as lost (12 in 8 bursts); on the final code, none in 2 bursts.

**Receive path**
* Fix: the idle gap is measured at the byte that follows it. `processTimeout()` runs from the loop, after the drain has already pushed the whole UART buffer through the parser, so a pause between two telegrams inside one drain was invisible and the telegram that arrived after it was appended to a parked torso instead of starting a frame
* Fix: the gap marker is set only while the interface runs empty. `_lastReceivedTime` is a READ time, so a blocked host looked exactly like a bus that went quiet -- a 4 kB flash erase runs with interrupts off while the octets keep arriving, and a complete telegram was swept. Measured on an RP2350: a 27 ms gap with the telegram complete in the buffer, three destroyed in one download. The sweep latches `_invalid`, which then eats valid control bytes until a whole frame gets through again
* Fix: a marker is not swept while octets are still waiting. Two detectors set it and only the loop-driven one consulted the interface; a marker that predates buffered octets cannot stand for a quiet bus, whoever set it. The length octet and the check octet decide now instead of a time estimate
* Fix: a chip indication whose companion octet arrives late survives. `U_SYSTEM_STAT_IND` asks for a second octet, which on hardware followed 29 ms later; the marker was then older than the octets it swept and the chip's status answer was torn apart. Outside a frame only this indication and the poll telegram (`L_POLL_DATA_IND`) raise `_awaitBytes`, so frame resynchronisation is untouched
* Fix: the wait for the confirmation of our own frame survives a probe answer. The 1 Hz `U_State.ind` (or `U_SystemStat.ind` with its value octet) arriving between the echo and the `L_Data.con` completed the frame without a confirmation, the transmitter was released early, the next frame's `U_L_DataStart.req` reached the NCN while it was still sending, and the NCN answered with a protocol error and dropped that frame. The answer is now evaluated and taken out of the buffer, and the wait goes on
* Fix: on the NCN a frame start after our echo ends only that attempt. The chip sends no `L_Data.con` between its repetitions (NCN5120 DS p.33), so our own repetition, or a foreign frame in the pause between two attempts, released the transmitter early as well; a repetition is now counted as one and matched as the echo again, a foreign frame is parsed, and the confirmation that follows the last attempt ends the wait
* Fix: when a frame start ended our attempt and no octet at all follows for `TPUART_TX_ATTEMPT_QUIET_MS` (new, 50 ms), the frame is handed up as an unacknowledged echo, so knx confirms it negatively once and the transmitter is released without a BCU reset, so no queued frame is dropped. Before, the transmitter waited for the 8 s backstop, which reset the BCU and dropped the queue; that path remains when foreign frames keep the bus busier than the window (frames of n octets less than 50 ms + (n - 1) x 13 bit times apart, several senders add up). `TPUART_TX_CON_QUIET_MS` and `TPUART_TX_ATTEMPT_QUIET_MS` must be at least 25 ms. Known limits: a rescue push can delay disconnect detection by up to 30 s once per frame; a frame waiting for its confirmation when busmonitor mode starts can show up as a pushed copy about 50 ms later, and the wait can also survive into busmonitor mode; a frame confirmed negatively can have been acknowledged on the bus when only its confirmation was lost; after a foreign frame between two attempts the repetition count and the bus-load bits miss our attempt; two older paths still release the transmitter without a confirm to knx, an own echo with a CRC error and a confirmation outside the wait; with `OPENKNX_DUALCORE` (off in every product) the second rescue can confirm a frame the second core has just started, because the flag is not re-checked under the receive lock
* Fix: a confirmation octet that is already in the buffer when the gap marker is handled is judged as a confirmation instead of the frame being swept as unconfirmed. As a side effect, for busmonitor frames and our own echo the awaited octet after a gap longer than `TPUART_RX_TIMEOUT` is judged as well: an acknowledge-like octet now counts as that frame's acknowledge, a `U_State.ind` (NCN: also a `U_SystemStat.ind`) is absorbed and on the NCN a frame start behind our echo ends the attempt
* Fix: in busmonitor mode an errored frame is forwarded once; the resynchronisation forwarded every frame-like octet inside it as another errored frame
* Change: the "Bytes are discarded" dump keeps its own 256 octets (`TPUART_DISCARD_LOG_SIZE`) instead of a second 6001-octet ring, 5744 octets less RAM. More than 256 octets discarded between two dumps are cut to the first 256 and the count shows "N+", also across a reset; the old ring printed up to 6000

**Repetition filter**
* Fix: an entry ages out. 03_02_02 2.4.2 p.39 suppresses the repetition of the directly preceding frame; with no bound, a repetition whose original this device missed was dropped forever whenever that source's last frame carried the same payload -- which is the normal state of a cyclic sensor. `TPUART_REPETITION_TTL_MS` defaults to 3000 ms, above the roughly 2.6 s a legitimate repetition burst of a maximum extended frame takes
* Fix: this device's own transmissions are recorded as well. Keeping the echo out left the filter without the original, so a chip retransmission arrived as a foreign telegram carrying our own source address -- measured on the line as one answer plus three repetitions, after which the device reported its own individual address as duplicated. The confirmation branch in the knx receive path runs before the filter gate, so a retried transmission is still confirmed
* Fix: the octet that keys the filter is shifted unsigned. `frame.data()` returns a signed char, and shifting one left is undefined from 80h up, so the CRC was undefined for exactly the frames it has to tell apart
* Change: the cache entry carries named fields instead of a pair, so the source and the checksum cannot be swapped at a call site; the `cleanup()` declaration, which had no definition anywhere, is removed

**Transmit path**
* Fix: a transmission is waited out before its confirmation is declared lost. The rescue used `TPUART_RX_TIMEOUT`, which guards an intra-frame gap at 5 ms, while the chip's own retransmission needs 30 bit times of acknowledge window plus 50 idle on NACK or 150 on BUSY before the first retry octet, and hundreds of ms for a full sequence -- so a transmission still in progress was reported as failed. `TPUART_TX_CON_QUIET_MS` defaults to 200 ms; the 8 s transmit backstop stays the outer net
* Fix: a parked acknowledge is written only below 2432 us after the decision instead of up to 3007 us; the age test counts 64 us ticks, so it is dropped between 2369 and 2432 us. The acknowledge waits behind the octet on the wire and needs its own character, 2 x 572.9 us at 19200 baud, so 3000 us let it reach the chip up to 4153 us after the decision, past the 3698 us the shortest frame leaves (03_02_02 2.2.7). A static_assert keeps `TPUART_ACK_WINDOW_US` in 64..2552
* Fix: with `OPENKNX_DUALCORE` a parked acknowledge is written once; both cores flush it, and the second one wrote it again after the first had cleared it
* Feature: on RP2040 and RP2350 with the UART interrupt and DMA receive and without `OPENKNX_DUALCORE`, the UART interrupt sends from a 256-octet ring (`TPUART_RP2040_TX_RING_SIZE`), so the main loop no longer waits per octet. An acknowledge goes out at the next octet boundary ahead of queued octets, or is dropped once older than `TPUART_ACK_WINDOW_US`, and `droppedAcknowledges()` counts it
* Change: multi-octet requests (the frame octet pairs, set address, set repetition and the ACR0 write) are written as one block, which the RP2040 interrupt transmitter keeps together, so an acknowledge it sends ahead of queued octets waits for the end of the block; on the other interfaces `txLock` already kept acknowledges out of them
* Change: on every platform a TX pump pass ends when `txLock` is taken, instead of re-polling for up to `TPUART_TX_PUMP_MAX_MS`; the frame continues in the next pass
* Fix: `ArduinoSerial::write()` returns true for a written octet; it returned the negation, which a block write would have taken as a failure after the first octet
* Feature: a repetition of a frame this device answered BUSY is acknowledged, so the sender does not run through all its BUSY repetitions and give up. Only the knx router answers BUSY, for a frame bound for IP while a `ROUTING_BUSY` pause runs; a repetition whose original was not received is acknowledged during the pause as well

**Bus state**
* Fix: `busOperational()` reports the bus down once the third 1 Hz state probe goes out with the two before it unanswered (`TPUART_BUS_SILENT_PROBES`), about 2 to 3 s after the last octet. A bus-powered NCN loses its supply with the bus and sends nothing, and the last VBUS state was kept until the 30 s disconnect
* Fix: the state probe keeps running while the receiver is desynced, so the silence count rises there as well; `U_SYSTEM_STATE_REQ` stays off then, because its value octet can parse as a frame head. Since the probe answers refresh the octet timer, the disconnect after 30 s without valid receive data now also fires while the receiver is desynced

**Configuration**
* Change: the build knobs live in `src/TPUart/Config.h`, each overridable with `-D`. The remaining fixed limits are knobs with unchanged defaults: `TPUART_RESET_PROBE_MS`, `TPUART_TX_PUMP_MAX_MS`, `TPUART_TX_WATCHDOG_MS`, `TPUART_REPETITION_FILTER_SIZE`, `TPUART_ESP32_DRAIN_GUARD`, `TPUART_ESP32_RX_BUFFER_SIZE`, `TPUART_ESP32_TX_BUFFER_SIZE`, `TPUART_ESP32_EVENT_QUEUE_SIZE` and `TPUART_RP2040_DMA_BUFFER_EXP`; static_asserts reject values the code cannot hold
* Change: the backstop knobs are `TPUART_TX_BACKSTOP_MS`, `TPUART_TX_BACKSTOP_COOLDOWN_MS` and `TPUART_TX_BACKSTOP_INVALID_MS`; a build that still sets an old name stops with `#error`
* Change: the transmit queue limit is the build constant `TPUART_TX_QUEUE_SIZE`; `Transmitter::setQueueSize()`, which nothing called, is removed

**Addressing**
* Fix: the own individual address is held unsigned. `_ownAddress` and `setOwnAddress()` were `short`, so an address at or above 0x8000 -- areas 8 to 15, and the 15.15.255 default -- was negative, and the test for a positive value excluded exactly those addresses from the chip acknowledge path

**Housekeeping**
* Fix: the interface pointer is no longer pretended to be deleted. The condition tested it for null before deleting, so nothing was ever deleted; the interface belongs to the Platform, which hands it out and outlives this layer. The parameter stays for the signature and the member is cleared
* Change: a dead store at the top of the RX drain is removed, together with `TPUART_MAX_PROCESS_TIME_PER_LOOP`, which nothing referenced -- `TPUART_MAX_RXQUEUE_TIME_PER_LOOP` is the live one. Why the drain stays unbounded is recorded in place: cutting it short trades a loop stall for a UART FIFO overflow
* Change: the comments are in English, and wrong ones are corrected: the repetition-count bit layout, the frame-size wrap, the largest extended frame (263 octets), the interface owner and the poll telegram as a second multi-octet control sequence; the README lists every knob of Config.h
* Change: the `U_Reset.ind` count is incremented without `++` on a volatile, which C++20 deprecates
* Change: the `OPENKNX_CON_DIAG` confirmation counters are removed; the knx tunnel server, their only reader, no longer declares them


## ec/1.3.0-beta.1: 2026-09-03

The tag was moved again on 2026-09-15, from `472d7ca` to `8de964d`, so the two entries directly below
are part of it. It had been left behind: `knx` pins this tag, and the stack already used members that
only the newer commit carries, so every consumer that resolved the tag failed to compile while a local
checkout built fine.

* Feature: `U_Reset.ind` is counted, so a chip that swallowed a reset request can be told from one that
  answered it -- the driver saw only the absence of a reply and could not distinguish the two
* Fix: a failed frame allocation is not copied into

The tag was moved from `340897e` so the volatile-counter fix below is part of it.

**Transmit path**
* Fix: a dropped frame is reported only when one was actually in flight -- `reset()` reported the in-flight frame unconditionally, so a frame already confirmed positively produced a second, negative `L_Data.con` on every console reset, busmonitor exit and watchdog reset
* Feature: a frame the driver has to drop is handed to a new dropped-frame callback, so the upper layer can confirm it negatively instead of leaving the caller in a timeout
* Fix: `receivedReset()` no longer resets the transmitter from the UART interrupt, where its blocking `txLock` hung the device whenever the main loop held that lock -- `handleReset()` does it in main-loop context
* Fix: the transmit pump and both `processQueue()` calls stop while a chip reset is pending, so no partial LPDU reaches the line and no frame is armed only to be dropped unsent
* Fix: an early return in `processTransmitByte` left `txLock` held permanently, which killed every acknowledge and all transmission
* Fix: the queue scan checked three buckets and then took `front()` of the fourth unconditionally, which is undefined on an empty deque
* Fix: the repetition filter skips raw busmonitor carriers, whose buffer is shorter than `size()` claims

**Acknowledge**
* Fix: a blocked acknowledge is parked and written as soon as `txLock` frees, instead of being cached and flushed only at the end of `processTransmitByte` -- which returns immediately while the transmitter is idle, so the cached acknowledge sat until the next transmission and was then applied to whatever frame the chip was receiving at that moment, acknowledging a foreign telegram and suppressing the TP1 repetition its receiver depends on
* Fix: a parked acknowledge is discarded once its bus window has elapsed. The window is derived from 03_02_02: the decision falls six octets in, the acknowledge must start at `13N+13`, so the budget is `13N-63` bit times; the shortest standard frame is LG 0 at N=8, which leaves 41 bit times, minus the 11-bit host character at the 19200 strap
* Fix: byte and timestamp live in one word, because two fields cannot be read consistently from the receive context and the main loop; the timestamp counts 64 us ticks, so 24 bits cover 1073 s and a long main-loop stall cannot alias a stale park back into the window
* Fix: monitor mode discards a parked acknowledge instead of writing it, because the chip is passive there
* Feature: what no longer fits its window is counted and shown in `bcu stat`

**Receive path**
* Fix: a bus-side bit error no longer silences the transmitter. On ESP32 `UART_PARITY_ERR`/`UART_FRAME_ERR` fell through into the overflow case, so a parity error set `_overflow` like a lost octet; `Receiver` turns that into `_invalid` and `processQueue()` refuses to transmit while it is set. The NCN encodes a bus-side bit error into the parity bit, so a single bit error muted the device for the full 20 s watchdog period while its address kept acknowledging on the wire
* Fix: `Frame::cemiData()` checks its allocation before writing into the buffer

**Diagnostics**
* Feature: the UART byte status is split into framing, parity, break and overrun and counted separately -- the receive path had one flag for all of them, so a starved host link, a wrong baud rate and a bus-side bit error were indistinguishable
* Feature: `TPUART_API_LEVEL` lets a dependent gate on the new accessors, so it still builds against an older driver
* Feature: `bcu autoack on|off` withholds `U_SetAddress.req`, which is what turns the chip's automatic acknowledge on; the host then decides every acknowledge alone. Default unchanged; refused while a busmonitor runs

**Bus load**
* Fix: only the bit times that reach TP1 are counted -- the sum added 26 bit times per frame for the two CRC-CCITT octets the NCN appends on the host UART, and the 15 bit times of the acknowledge window even when no acknowledge followed, although 03_02_02 2.3.1 measures the 50 bit times of bus-free from the last bit of the message cycle. Reported load was 6 to 20 percent too high

**Initialisation**
* Fix: the probed baud rate is cross-checked before it is accepted -- one octet looking like `U_RESET_IND` was enough, and reading a stream at the wrong rate hands out re-framed fragments of neighbouring octets. The first pass now requires a probe without a framing error; the second repeats the old rule so a board that always reports one still comes up

**NCN chip identification**
* Fix: the identification read the RevID register while the KNX receiver was on, so the header-less reply was whatever octet arrived first -- a bus telegram byte, an acknowledge or a state indication became the chip id
* Fix: the part number was taken as `revid & 0x1F` with no plausibility check, so eight different byte values each read as an NCN5130 and eight more as an NCN5121
* Fix: the read ran inside the per-baud probe, so it also executed on baud rates the probe went on to reject, writing register opcodes into the chip at the wrong rate
* The read now runs once on the accepted baud, inside a Stop-mode window, and is preceded by a watchdog-register read whose reset value 0x0F is documented for all three parts -- a mismatch means the byte did not come from the register file
* An NCN5120 is identified by the ACR1 reset value, which is documented on all three parts, so the undefined opcode 0x3D is only sent to a part that has the register; a RevID reply carrying no documented part number reports "unknown" instead of guessing
* U_ExitStopMode.req is ignored outside Stop and nothing can prove Stop was reached, so the window always closes with U_Reset.req -- an armed U_StopMode.req firing later would switch the receiver off while U_State.req keeps answering, which the liveness watchdog cannot see
* Gated behind `TPUART_BCU_REGISTER_INFO`: a product that does not report the chip keeps its previous boot sequence
* `TPUART_API_LEVEL` raised to 3 for `getNcnChip`/`getNcnChipName`/`ncnChipInferred`/`ncnAsr0Valid`

**Build**
* Fix: the counters no longer use `++`/`--` on a volatile-qualified type, which C++20 deprecates -- eight warnings in every product build. `+= 1`/`-= 1` is what these files already used for every other counter, the result value was unused at all eight sites, and the object code is byte-identical


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
