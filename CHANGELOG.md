# Changes

## 2.0: 2026-11-13

Complete rewrite of the datalink layer. Telegrams are no longer found by searching the byte stream but
recognised by a state machine, and processing runs on a fixed tick the library generates itself.
Memory usage drops from just under 13 kB to around 3 kB.

* Change: Telegram boundaries come from the length octet. If anything does not add up, the receiver discards everything until a verified bus pause and then knows the BCU is between two telegrams. The search buffer could also hit where there was no telegram at all, up to acknowledging an address that never existed.
* Change: Broken telegrams are delivered as telegrams - with source, destination and as much as arrived - instead of vanishing as discarded bytes.
* Change: The library drives `tick()` from its own hardware timer (500 µs), one byte per direction per tick. The application calls `loop()`. One timer serves all instances, `TPUART_TIMER_MAX_CLIENTS` (default 1) limits how many BCUs register.
* Change: The RX/TX locks and the cached acknowledge are gone - the acknowledge goes out when it is due, or not at all.
* Change: The RP2040 interface always uses IRQ and DMA.
* Change: New names `isBusMonitor()`, `bcuStateName()`, `registerFrameCallback()`, `AckType`, `BcuType::Tpuart2`/`BcuType::Ncn5120`.
* Change: `library.json` declares the Arduino framework on `raspberrypi` and `espressif32`.
* Compatibility: The 1.x API stays available as a compatibility layer, marked `KOMPAT` in the code, so that knx, OGM-Common and OFM-Network keep building. It covers the constructor without interface plus `begin(type, interface)`, `end()`, `process()` (calls `loop()`), `pushTransmitQueue(Frame *)` taking ownership, `Frame(const char *, size_t)` and `cemiData()`/`cemiSize()`, the old names (`registerReceivedFrame()`, `isMonitoring()`, `getBcuStateInfo()`, `BcuType::BCU_*`, `AcknowledgeType`/`ACK_*`, the old `Statistics` getters), the RP2040 constructor's `irq`/`dma` parameters (ignored) and the search-buffer diagnostics, mapped onto the new receive path. New code should use the new API; the layer goes once the consumers have moved.
* New: Several frame callbacks can be registered; each `registerFrameCallback()` adds one.
* New: On NCN chips the internal registers are read once per `begin()` to tell NCN5120, NCN5121 and NCN5130 apart (`bcuChip()`, `ncnRevision()`); `writeRegister()` sets a register.
* New: `pushTransmitQueue()` accepts a `const Frame &` or a raw buffer and copies it.
* New: Unit tests for the host (`native_test`) with a `Dummy` interface, plus hardware test environments `pico_test` and `esp32_test`.

## 1.1: 2026-07-30

* Change: Send `U_L_DataOffset` only when the offset actually changes instead of before every byte. The BCU keeps the offset internally until it is changed, so of ~199 offset bytes in a 263-octet frame only ~4 actually change it. Frames of 64 octets or less are unaffected (offset is always 0 there). (@erkanc)
* Fix: Extended frames with a LEN octet > 127 were unreceivable. `apduSize()`/`metadataSize()` returned signed `char`, so on signed-char targets (ESP32) the value sign-extended negative and wrapped the arithmetic in `size()`. (@erkanc)
* Fix: `processRxFrameBuffer()` sized a stack array from an untrusted length header; a `bufferSize < 3` underflow wrapped it to ~65533. The length is now checked against the search buffer size, and a mismatch is logged and resets the BCU instead of being papered over. (@erkanc)
* Fix: `Transmitter::_state` is now `volatile` — written by `finalize()` on the RX path, read by the main-loop TX path, matching `Receiver::_state`. (@erkanc)
* Fix: `reset()` did not re-arm `_lastReceivedTime`, so a reset on a bus that had been idle for more than 5 s made the very next `process()` report a spurious disconnect.
* Fix: On `U_RESET_IND` the transmitter kept its position and would have continued mid-frame, putting the tail of a telegram on the wire as if it were a whole one. The transmitter is now reset fully (queue, in-flight frame, offset).
