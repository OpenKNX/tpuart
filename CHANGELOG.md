# Changes

## 2.0: Work in progress

Complete rewrite of the datalink layer. What changed fundamentally is how telegrams are recognised.
Previously the library collected the byte stream in a search buffer and repeatedly asked whether
whatever sat at the front looked like a telegram; if it did not, one byte was dropped and the window
moved on. There is now a state machine: telegram boundaries come from the length octet, and if
anything does not add up, everything is discarded until a verified bus pause. After that it is
established that the BCU is between two telegrams - the state is restored rather than guessed, and the
next byte is once again unambiguously either a control byte or the start of a telegram. A search, by
contrast, could also hit where there was no telegram at all, up to acknowledging an address that never
existed.

Broken telegrams are therefore reported as telegrams - with source, destination and as much as
arrived - instead of vanishing as discarded bytes. For a bus monitor that is the decisive difference.

Processing no longer hangs on the arrival of a byte but on a fixed tick the library generates itself:
two halves with a ring buffer between them, one byte per direction per call. Because a single step can
now both read and write, the locks and the cached acknowledge are gone - it goes out when it is due,
or not at all.

Memory usage drops from just under 13 kB to around 3 kB.

## 1.1: 2026-07-30

* Change: Send `U_L_DataOffset` only when the offset actually changes instead of before every byte. The BCU keeps the offset internally until it is changed, so of ~199 offset bytes in a 263-octet frame only ~4 actually change it. Frames of 64 octets or less are unaffected (offset is always 0 there). (@erkanc)
* Fix: Extended frames with a LEN octet > 127 were unreceivable. `apduSize()`/`metadataSize()` returned signed `char`, so on signed-char targets (ESP32) the value sign-extended negative and wrapped the arithmetic in `size()`. (@erkanc)
* Fix: `processRxFrameBuffer()` sized a stack array from an untrusted length header; a `bufferSize < 3` underflow wrapped it to ~65533. The length is now checked against the search buffer size, and a mismatch is logged and resets the BCU instead of being papered over. (@erkanc)
* Fix: `Transmitter::_state` is now `volatile` — written by `finalize()` on the RX path, read by the main-loop TX path, matching `Receiver::_state`. (@erkanc)
* Fix: `reset()` did not re-arm `_lastReceivedTime`, so a reset on a bus that had been idle for more than 5 s made the very next `process()` report a spurious disconnect.
* Fix: On `U_RESET_IND` the transmitter kept its position and would have continued mid-frame, putting the tail of a telegram on the wire as if it were a whole one. The transmitter is now reset fully (queue, in-flight frame, offset).
