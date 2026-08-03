# Changes

## 1.1: 2026-07-30

* Change: Send `U_L_DataOffset` only when the offset actually changes instead of before every byte. The BCU keeps the offset internally until it is changed, so of ~199 offset bytes in a 263-octet frame only ~4 actually change it. Frames of 64 octets or less are unaffected (offset is always 0 there). (@erkanc)
* Fix: Extended frames with a LEN octet > 127 were unreceivable. `apduSize()`/`metadataSize()` returned signed `char`, so on signed-char targets (ESP32) the value sign-extended negative and wrapped the arithmetic in `size()`. (@erkanc)
* Fix: `processRxFrameBuffer()` sized a stack array from an untrusted length header; a `bufferSize < 3` underflow wrapped it to ~65533. The length is now checked against the search buffer size, and a mismatch is logged and resets the BCU instead of being papered over. (@erkanc)
* Fix: `Transmitter::_state` is now `volatile` — written by `finalize()` on the RX path, read by the main-loop TX path, matching `Receiver::_state`. (@erkanc)
* Fix: `reset()` did not re-arm `_lastReceivedTime`, so a reset on a bus that had been idle for more than 5 s made the very next `process()` report a spurious disconnect.
* Fix: On `U_RESET_IND` the transmitter kept its position and would have continued mid-frame, putting the tail of a telegram on the wire as if it were a whole one. The transmitter is now reset fully (queue, in-flight frame, offset).
