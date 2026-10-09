// #pragma GCC optimize("O3")
#include "TPUart/Receiver.h"
#include "TPUart/DataLinkLayer.h"

#ifdef OPENKNX_CON_DIAG
// L_Data.con-generation diagnostics (counters dumped by the IP tunnel server). Gated by OPENKNX_CON_DIAG.
uint16_t g_rxTxWaitAckn = 0, g_rxTxCompleted = 0, g_txAwaitEcho = 0, g_echoMismatch = 0, g_echoTxIdle = 0;
#endif

namespace TPUart
{
    Receiver::Receiver(DataLinkLayer &dll) : _dll(dll)
    {
    }

    void Receiver::process()
    {
        processTimeout();

        // Own echo in RX_FRAME_WAIT_ACKN whose CON is lost on a quiet bus: no octet drives the parser, so the loop forces
        // the completion after TPUART_TX_CON_QUIET_MS, which releases the Transmitter (processCompleteFrame).
        if (_state == RX_FRAME_WAIT_ACKN
            && millis() - _lastReceivedTime >= TPUART_TX_CON_QUIET_MS
            && _searchBuffer.frame().isTransmitted()
            && _dll.getTransmitter().awaitResponse())
        {
            _dll.rxLock(true); // process() runs without rxLock; the processSearchBuffer* family needs it
            // Re-check under the lock; require a genuinely idle interface (a pending byte -- possibly
            // the real CON -- must be read first via the normal byte-driven path).
            if (_state == RX_FRAME_WAIT_ACKN
                && _searchBuffer.frame().isTransmitted()
                && _dll.getTransmitter().awaitResponse()
                && !_dll._interface->available()
                && millis() - _lastReceivedTime >= TPUART_TX_CON_QUIET_MS)
            {
#ifdef TPUART_BCU_DEBUG
                _dll.printError("CONr: idleBus");
#endif
                // Marker at frameSize (< _awaitBytes == frameSize+1) routes processSearchBufferTimeout()
                // into the RX_FRAME_WAIT_ACKN forced completion, which carries the finalize() below.
                if (!_searchBuffer.timeout()) _searchBuffer.timeout(_searchBuffer.position());
                processSearchBufferTimeout();
            }
            _dll.rxUnlock();
        }
        // Attempt ended by a frame start, then no octet for TPUART_TX_ATTEMPT_QUIET_MS: the own frame goes up as an unacknowledged echo, a negative confirm.
        if (_dll.getTransmitter().attemptEnded()
            && _state == RX_IDLE
            && millis() - _lastReceivedTime >= TPUART_TX_ATTEMPT_QUIET_MS
            && _dll.getTransmitter().awaitResponse())
        {
            _dll.rxLock(true);
            if (_state == RX_IDLE
                && _dll.getTransmitter().awaitResponse()
                && !_dll._interface->available()
                && millis() - _lastReceivedTime >= TPUART_TX_ATTEMPT_QUIET_MS)
            {
                Frame *frame = _dll.getTransmitter().currentFrame();
                frame->setTransmitted();
                _dll.pushRxFrameBuffer(*frame);
                _dll._statistics.incrementBcuConRescues();
                _dll.getTransmitter().finalize();
            }
            _dll.rxUnlock();
        }
        if (_state == RX_FRAME_WAIT_ACKN
            && _dll.isMonitoring()
            && millis() - _lastReceivedTime >= TPUART_RX_TIMEOUT)
        {
            _dll.rxLock(true);
            if (_state == RX_FRAME_WAIT_ACKN
                && _dll.isMonitoring()
                && !_dll._interface->available()
                && millis() - _lastReceivedTime >= TPUART_RX_TIMEOUT)
            {
                if (!_searchBuffer.timeout()) _searchBuffer.timeout(_searchBuffer.position());
                processSearchBufferTimeout();
            }
            _dll.rxUnlock();
        }
    }

    /*
     * @brief Handles fewer octets than awaited before the timeout marker, once the interface is empty.
     * Completes a frame whose acknowledge octet is missing and keeps a complete RX_IDLE control sequence;
     * otherwise discards up to the next frame-looking octet and sets _invalid.
     */
    void Receiver::processSearchBufferTimeout()
    {
        if (!_searchBuffer.timeout()) return;
        if (_searchBuffer.timeout() >= _awaitBytes) return;
        // Octets still queued: the gap may be a host stall, not bus silence; length and checksum decide instead.
        if (_dll._interface->available()) return;

        // A control sequence (U_SYSTEM_STAT_IND, L_POLL_DATA_IND) complete in RX_IDLE: a gap inside it is no desync.
        if (_state == RX_IDLE && _awaitBytes > 1 && _searchBuffer.position() >= _awaitBytes) return;

        if(_state == RX_FRAME_WAIT_ACKN)
        {
                // The awaited octet is there: processSearchBufferAcknowledge() judges it, so a late CON stays the CON.
                if (_searchBuffer.position() >= _awaitBytes) return;
                _sysStatValuePending = false; // its value octet did not come
                _state = RX_FRAME_COMPLETE;
                processCompleteFrame();
                return;
        }

#ifdef TPUART_BUSMON_INTEGRITY
        // Cut short on the bus: forward what arrived before the sweep discards it (03_06_03 4.1.5.8.1,
        // a lost frame piece sets the Lost flag; the FCS is the monitor application's job). Length from
        // the timeout marker, never size() -- that is the promised length, not what actually arrived.
        if (_dll.isMonitoring() && _searchBuffer.timeout() > 0 &&
            (_state == RX_FRAME || _state == RX_FRAME_DESTINATION || _state == RX_FRAME_SIZE) &&
            forwardErroredOnce((unsigned short)_searchBuffer.timeout()))
        {
            Frame &frame = _searchBuffer.frame();
            frame.addFlags(TP_FRAME_FLAG_TRUNCATED);
            if (_byteErrorInFrame) frame.addFlags(TP_FRAME_FLAG_BIT_ERROR);
            _dll.pushRxFrameBuffer(frame, (unsigned short)_searchBuffer.timeout());
        }
#endif

        processSearchBufferInvalid(4);
    }

    /*
     * @brief Sets the timeout marker at the current position once the interface is empty and nothing arrived for TPUART_RX_TIMEOUT ms.
     */
    void Receiver::processTimeout()
    {
        // Pre-check without the lock, cheapest test first
        if (_searchBuffer.timeout() == _searchBuffer.position()) return;
        if (_dll._interface->available()) return;
        if (millis() - _lastReceivedTime < TPUART_RX_TIMEOUT) return;

        _dll.rxLock(true);
        size_t timeout = 0;
        // Same check under the lock
        if (_searchBuffer.timeout() != _searchBuffer.position() && !_dll._interface->available() && millis() - _lastReceivedTime >= TPUART_RX_TIMEOUT)
        {
            timeout = _searchBuffer.position();
            _searchBuffer.timeout(timeout);
        }
        _dll.rxUnlock();

#ifdef TPUART_RX_TIMEOUT_DEBUG
        if (timeout) _dll.printError("TIMEOUT: %u %u", millis() - _lastReceivedTime, timeout);
#endif
    }

    /*
     * @brief Reads one octet from the interface into the search buffer and processes it.
     */
    bool Receiver::processReceviedByte()
    {
        if (!_dll.rxLock()) return false;
        if (_dll._interface->overflow())
        {
            _invalid = true;
            _dll._rxInterfaceOverflow = true;
            _dll._statistics.incrementRxUartOverflow();
        }
        const int value = _dll._interface->read();

        if (value != -1)
        {
            const unsigned long now = millis();
            // _lastReceivedTime is a read time: the gap is a bus gap only with the interface empty, else a host stall (no marker).
            const bool gap = _searchBuffer.position() && !_searchBuffer.timeout()
                             && (now - _lastReceivedTime) >= TPUART_RX_TIMEOUT;
            if (gap && !_dll._interface->available())
                _searchBuffer.timeout(_searchBuffer.position());

            _lastReceivedTime = now;
            _unansweredProbes = 0;
#ifdef TPUART_BUSMON_INTEGRITY
            if (_dll._interface->lastByteErrored()) _byteErrorInFrame = true;
            // Counted, not acted on: the four causes need separate numbers or a starved host link and a
            // bus-side bit error are indistinguishable.
            _dll._statistics.incrementRxByteStatus(_dll._interface->lastByteStatus());
#endif

            const uint start = micros();
            _dll._statistics.incrementRxReceivedBytes();
            pushSearchBuffer(value);
            uint duration = micros() - start;
            _dll._statsDuration += duration;
            _dll._statsDurationCount = _dll._statsDurationCount + 1;
            if (duration > _dll._statsDurationMax) _dll._statsDurationMax = duration;
            if (duration < _dll._statsDurationMin) _dll._statsDurationMin = duration;
        }

        _dll.rxUnlock();
        return true;
    }

    /*
     * @brief Called per new octet while a frame awaits its acknowledge; an L_DATA_CON or L_ACKN_IND sets its acknowledge state.
     * A U_State.ind (NCN: U_SystemStat.ind) is absorbed; on the NCN a frame start after our echo ends only this attempt; any other octet completes the frame.
     */
    void Receiver::processSearchBufferAcknowledge()
    {
        if (_state != RX_FRAME_WAIT_ACKN) return;

        const size_t at = _awaitBytes - 1;
        const char value = _searchBuffer.get(at);

        if (_sysStatValuePending) // checked first: the value octet can take any form, also that of a CON
        {
            _sysStatValuePending = false;
            _dll._systemState.update(value);
            _searchBuffer.remove(at);
            _searchBuffer.timeout(0);
            return;
        }

        bool acknowledge = false;
        if ((value & L_DATA_CON_MASK) == L_DATA_CON)
        {
            if ((value ^ L_DATA_CON_MASK) >> 7)
            {
                _searchBuffer.frame().setAcknowledge();
            }
            acknowledge = true;
            _dll.getTransmitter().finalize();
        }
        else if ((value & L_ACKN_MASK) == L_ACKN_IND)
        {
            const bool isBusy = !(value & L_ACKN_BUSY_MASK);
            const bool isNack = !(value & L_ACKN_NACK_MASK);
            _searchBuffer.frame().setAcknowledge(isBusy, isNack);
            acknowledge = true;
        }
        // Probe answer between echo and CON (DS Table 13: U_State.ind ends 111, 0xFF unknown; U_SystemStat.ind 0x4B + value); timeout(0) clears its gap marker.
        else if ((value & U_STATE_MASK) == U_STATE_IND && (uint8_t)value != 0xFF)
        {
            _dll.receivedState(value);
            _searchBuffer.remove(at);
            _searchBuffer.timeout(0);
            return;
        }
        else if (value == U_SYSTEM_STAT_IND && _dll._bcuType == BCU_NCN5120)
        {
            _sysStatValuePending = true;
            _searchBuffer.remove(at);
            _searchBuffer.timeout(0);
            return;
        }
        // NCN sends no CON between attempts (DS p.33): a frame start after our echo (our repetition or a foreign frame) ends only this attempt.
        else if (((value & L_DATA_MASK) == L_DATA_STANDARD_IND || (value & L_DATA_MASK) == L_DATA_EXTENDED_IND) &&
                 _searchBuffer.frame().isTransmitted() && _dll._bcuType == BCU_NCN5120)
        {
            if ((uint8_t)value == ((uint8_t)_searchBuffer.get(0) & ~0x20))
            {
                const unsigned short frameSize = _searchBuffer.frame().size();
                _dll._statistics.incrementRxRepetitions();
                _dll._statistics.incrementRxFrameBits((int)(13u * (unsigned int)frameSize - 2u + 50u)); // attempt without ACK
            }
            _dll.getTransmitter().markAttemptEnded();
            _searchBuffer.frame().resetFlags();
            _searchBuffer.move(at);
            _awaitBytes = 1;
            _state = RX_IDLE;
            _byteErrorInFrame = false;
            return;
        }

        processCompleteFrame(acknowledge);
    }

    void Receiver::processCompleteFrame(bool acknowledge)
    {
        if (_state == RX_FRAME_COMPLETE || _state == RX_FRAME_WAIT_ACKN)
        {
#ifdef OPENKNX_CON_DIAG
            if (_searchBuffer.frame().isTransmitted()) g_rxTxCompleted++;
#endif
            // Own echo (matched on control, destination, source) completed while the Transmitter still awaits its CON: release it.
            // No printError: can run in the UART IRQ or the UART task.
            if (_searchBuffer.frame().isTransmitted() && _dll.getTransmitter().awaitResponse())
            {
                _dll._statistics.incrementBcuConRescues(); // only genuine rescues (awaitResponse still set)
                _dll.getTransmitter().finalize();
            }

#ifdef TPUART_BUSMON_INTEGRITY
            if (_byteErrorInFrame) _searchBuffer.frame().addFlags(TP_FRAME_FLAG_BIT_ERROR);
#endif
            unsigned short size = _searchBuffer.frame().size();
            if (!_dll.isMonitoring() && _dll._modeExtendedCRC) size += 2;
            if (acknowledge) size++; // an ACKN or DATA_CON follows
            const bool ackOnLine = _searchBuffer.frame().isAck(); // read before resetFlags() clears it
            _dll.pushRxFrameBuffer(_searchBuffer.frame());
            _searchBuffer.frame().resetFlags();

            _dll._statistics.incrementRxFrameBytes(_searchBuffer.frame().size());
            // _dll._statistics.incrementRxControlBytes(size - _searchBuffer.frame().size());
            _dll._statistics.incrementRxFrames();

            // Line time this frame occupied, in bit times (03_02_02). Counted here because only this
            // point knows whether an ACK actually followed.
            const unsigned short frameSize = _searchBuffer.frame().size();
            if (frameSize)
            {
                unsigned int busBits = 13u * (unsigned int)frameSize - 2u;
                // 03_02_02 2.3.1: the 50 bit times of bus-free run from the last bit of the message
                // cycle, so without an acknowledge the 15 bit times of ACK idle are inside them.
                if (ackOnLine) busBits += 15u + 11u;                                // ACK idle + ACK character
                busBits += 50u;                                                    // mandatory bus-free time
                _dll._statistics.incrementRxFrameBits((int)busBits);
                // TODO(tpuart-v2): book by frame timestamp, not by completion time.
            }

            _searchBuffer.move(size);
            _awaitBytes = 1;
            _state = RX_IDLE;
            _byteErrorInFrame = false;

            // An empty buffer also clears the invalid flag
            //_dll.printError("FV! %u %u %u", _dll._interface->available(), _searchBuffer.position(), _awaitBytes);
            if (!_dll._interface->available())
            {
                if (_searchBuffer.empty())
                {
                    _invalid = false;
                }
            }
        }
        else
        {
            if (_searchBuffer.position()) processSearchBufferInvalid(1);
        }
    }

    void Receiver::processSearchBuffer()
    {
        if (_searchBuffer.empty()) return;
        processSearchBufferTimeout();

        if (!sufficientlyBytes()) return;

        // While _invalid is set, octet 0 can be any value (a CRC octet 0x03 reads as U_RESET_IND), so control octets are not interpreted.

        processSearchBufferAcknowledge();
        if (_searchBuffer.empty()) return;

        if (_searchBuffer.frame().isFrame())
            processSearchBufferFrame();

        else if (_invalid)
            processSearchBufferInvalid(2);
        else
            processControlBytes();

        // next
        processSearchBuffer();
    }

    void Receiver::processSearchBufferInvalid(int x)
    {
        // Releases the Transmitter for an own echo before the sweep; no printError: can run in the UART IRQ or UART task.
        if (_searchBuffer.frame().isTransmitted() && _dll.getTransmitter().awaitResponse())
        {
            _dll._statistics.incrementBcuConRescues(); // only genuine rescues (awaitResponse still set)
            _dll.getTransmitter().finalize();
        }

        while (_searchBuffer.position())
        {
            char value = _searchBuffer.get(0);
            //_dll.printError("IVB1: %02X  - H:%u I:%u P:%u T:%u A:%u", value, x, _invalid, _searchBuffer.position(), _searchBuffer.timeout(), (uint)_awaitBytes);
            _lastDiscarded = millis();
            asm volatile("" ::: "memory");
            _discardedBytes.push(value);
            _searchBuffer.move(1);
            _dll._statistics.incrementRxDiscardedBytes();

            if (_searchBuffer.frame().isFrame()) break;
        }

        _searchBuffer.frame().resetFlags();
        _state = RX_IDLE;
        _byteErrorInFrame = false;
        _invalid = true;
        _awaitBytes = 1;
    }

    void Receiver::processSearchBufferFrame()
    {
        Frame &frame = _searchBuffer.frame();

        if (_state == RX_IDLE) _state = RX_FRAME;

        if (_state == RX_FRAME)
        {
            _awaitBytes = frame.awaitDestination();

            if (!sufficientlyBytes()) return;

            // Destination in: match our own echo, then decide the acknowledge
            // _dll.printMessage("    DST ADDRESS %s", frame.humanDestination().c_str());
            if (_dll.getTransmitter().awaitResponse())
            {
#ifdef OPENKNX_CON_DIAG
                g_txAwaitEcho++; // echo parsed while we await our own TX's confirm
#endif
                // _dll.printMessage("      awaitResponse %i", _searchBuffer.position());
                if (!((frame.data(0) ^ _dll.getTransmitter().currentFrame()->data(0)) & ~0x20) && frame.destination() == _dll.getTransmitter().currentFrame()->destination() && frame.source() == _dll.getTransmitter().currentFrame()->source())
                {
                    frame.setTransmitted();
                    // _dll.getTransmitter().resetWatchdogTimer();
                }
#ifdef OPENKNX_CON_DIAG
                else g_echoMismatch++; // awaiting confirm but control/dest/source mismatch
#endif
            }
#ifdef OPENKNX_CON_DIAG
            else if (_dll.getTransmitter().currentFrame() != nullptr && !((frame.data(0) ^ _dll.getTransmitter().currentFrame()->data(0)) & ~0x20) && frame.destination() == _dll.getTransmitter().currentFrame()->destination() && frame.source() == _dll.getTransmitter().currentFrame()->source())
                g_echoTxIdle++; // matches last-TX content but transmitter not awaiting -> a con would be missed here
#endif

            if (!_dll.isMonitoring())
            {
                AcknowledgeType acknowledge = frame.isTransmitted() ? ACK_None : _dll.checkAcknowledge(frame.destination(), frame.isGroupAddress());
                // A repetition is acknowledged instead of answered BUSY again; if its original was received, the repetition filter marks it filtered.
                if (acknowledge == ACK_Busy && frame.isRepeated())
                    acknowledge = ACK_Addressed;
                if (acknowledge != ACK_None)
                {
                    _dll.getTransmitter().sendAcknowledge(acknowledge);
                    frame.setAcknowledge(acknowledge);
                }
            }

            _state = RX_FRAME_DESTINATION;
        }

        if (_state == RX_FRAME_DESTINATION)
        {
            // Frame size
            _awaitBytes = frame.awaitSize();
            if (!sufficientlyBytes()) return;

            _awaitBytes = frame.size();

            if (_dll._modeExtendedCRC) _awaitBytes += 2;

            _state = RX_FRAME_SIZE;
        }

        if (_state == RX_FRAME_SIZE)
        {
            if (!sufficientlyBytes()) return;

            bool valid = frame.isValid();

            // don't check CRC if the frame is not valid
            if (valid && _dll._modeExtendedCRC)
            {
                if (_dll._bcuType == BCU_NCN5120 && !frame.checkCRC16CCITT()) valid = false;
                if (_dll._bcuType == BCU_TPUART2 && !frame.checkCRC16SPI()) valid = false;
            }

            if (valid)
            {
                // _dll.printMessage("    COMPLETED VALID %i %i", frame.size(), _awaitBytes);

                // Wait for a DATA_CON or ACKN
                if (_dll.isMonitoring() || frame.isTransmitted())
                {
#ifdef OPENKNX_CON_DIAG
                    if (frame.isTransmitted()) g_rxTxWaitAckn++;
#endif
                    _awaitBytes = _awaitBytes + 1; // await one more octet, expected ACKN or DATA_CON
                    _state = RX_FRAME_WAIT_ACKN;
                    return;
                }

                _state = RX_FRAME_COMPLETE;
                processCompleteFrame();
            }
            else
            {
                // Raw busmon: forward the FCS-failed frame exactly as captured (incl. FCS) tagged as
                // errored, then discard it. Monitor-gated -> no change off-monitor. The errored frame is
                // never delivered up to the link layer (knx processRxFrame returns after the busmon push).
                if (_dll.isMonitoring() && forwardErroredOnce(frame.size()))
                {
                    frame.addFlags(TP_FRAME_FLAG_FCS_ERROR);
                    _dll.pushRxFrameBuffer(frame);
                }
                processSearchBufferInvalid(3);
            }
        }
    }

    bool Receiver::pushSearchBuffer(const char value)
    {
        // _dll.printMessage("pushSearchBuffer: %02X", value);
        if (!_searchBuffer.add(value))
        {
            _dll._rxSearchBufferOverflow = true;
            _dll._statistics.incrementRxSearchBufferOverflow();
            return false;
        }

        processSearchBuffer();

        return true;
    }

    /**
     * @brief Busmon: false for an errored frame starting inside the errored frame forwarded last, else records it.
     * Its span is the claimed length, or for a cut frame the octets that arrived (len). Known loss: a real errored
     * frame behind a frame-like noise octet or behind a corrupted control or length octet.
     */
    bool Receiver::forwardErroredOnce(unsigned short len)
    {
        const unsigned int start = _dll._statistics.getRxReceivedBytes() - (unsigned int)_searchBuffer.position();
        if (start - _errForwardedStart < _errForwardedLen) return false; // modulo 2^32, like the octet count
        _errForwardedStart = start;
        _errForwardedLen = len;
        return true;
    }

    void Receiver::reset()
    {
        _errForwardedLen = 0; // forget the last errored frame; the octet count restarts only in DataLinkLayer::reset()
        _searchBuffer.clear();
        _state = RX_IDLE;
        _awaitBytes = 1;
        _sysStatValuePending = false;
        _invalid = false;
    }

    unsigned short Receiver::getAwaitBytes()
    {
        return _awaitBytes;
    }

    unsigned short Receiver::getSearchBufferPosition()
    {
        return _searchBuffer.position();
    }

    inline bool Receiver::sufficientlyBytes()
    {
        return _searchBuffer.position() >= _awaitBytes;
    }

    void Receiver::processControlBytes()
    {
        const uint8_t value = (uint8_t)_searchBuffer.get(0);
        uint8_t count = 1;
        // _dll.printMessage("processControlBytes %02X", value);

        if (value == U_RESET_IND)
        {
            _dll.receivedReset();
        }
        else if (value == 0xFF || value == 0xFE || value == 0xFD || value == 0xFC)
        {
        }
        else if (value == U_STOP_MODE_IND && _dll._bcuType == BCU_NCN5120)
        {
            // Maybe trigger an requestSystemState to get SystemState faster
        }
        else if ((value & U_STATE_MASK) == U_STATE_IND)
        {
            _dll.receivedState(value);
        }
        else if (value == U_SYSTEM_STAT_IND && _dll._bcuType == BCU_NCN5120)
        {
            if (_searchBuffer.position() < 2)
            {
                _awaitBytes = 2;
                return;
            }
            // _dll.printMessage("U_SYSTEM_STAT_IND %02X", _searchBuffer.get(1));
            _dll._systemState.update(_searchBuffer.get(1));
            count = 2;
        }
        else if (_dll.isMonitoring() && (value == 0xFF || value == 0xFD))
        {
        }
        else if (value == L_POLL_DATA_IND)
        {
            /*
             * Poll telegram (DS p.53, Fig. 56): F0 SAH SAL PAH PAL SlotCount FCS + SlotCount slots.
             * Swept byte by byte instead, the sweep stops at the first frame-looking byte -- for a master
             * at 1.0.x that is its own source-address high byte 0x10, and the phantom frame gets
             * ACKNOWLEDGED on the bus. The FCS check separates a real header from a stray 0xF0.
             */
            if (_searchBuffer.position() < 7)
            {
                _awaitBytes = 7;
                return;
            }

            uint8_t chk = 0;
            for (uint8_t i = 0; i < 6; i++)
                chk ^= (uint8_t)_searchBuffer.get(i);

            if ((uint8_t)(~chk) != (uint8_t)_searchBuffer.get(6))
            {
                // Not a poll header: discarded like any unexpected control octet
                _lastDiscarded = millis();
                asm volatile("" ::: "memory");
                _discardedBytes.push((char)value);
                _dll._statistics.incrementRxDiscardedBytes();
                _invalid = true;
            }
            else
            {
                const unsigned short total = 7 + (uint8_t)_searchBuffer.get(5); // + slot octets
                if (_searchBuffer.position() < total)
                {
                    _awaitBytes = total;
                    return;
                }
#ifdef TPUART_BUSMON_INTEGRITY
                // Busmon: forward the poll telegram as a raw carrier.
                if (_dll.isMonitoring())
                {
                    Frame &frame = _searchBuffer.frame();
                    frame.addFlags(TP_FRAME_FLAG_RAW);
                    if (_byteErrorInFrame) frame.addFlags(TP_FRAME_FLAG_BIT_ERROR);
                    _dll.pushRxFrameBuffer(frame, total);
                    frame.resetFlags();
                    _byteErrorInFrame = false;
                }
#endif
                count = total;
            }
        }

        else if ((value & U_CONFIGURE_MASK) == U_CONFIGURE_IND)
        {
            _dll.receivedConfiguration(value);
        }
        else if ((value & L_DATA_CON_MASK) == L_DATA_CON)
        {
            _dll.getTransmitter().finalize();
        }

        else if ((value & L_ACKN_MASK) == L_ACKN_IND)
        {
            // Standalone L2 acknowledge (ACK/NAK/BUSY). This is the bare-ackn control path (a frame-
            // trailing ack is instead folded by processSearchBufferAcknowledge in RX_FRAME_WAIT_ACKN and
            // never reaches here). Monitor-only: forward the single raw octet to the busmon as a 1-byte
            // carrier (03_08_04: acknowledges shall also be transferred). Off-monitor this stays a no-op.
            if (_dll.isMonitoring())
                _dll.pushRxAcknByte(value);
        }

        else
        {
            // Unexpected
            _lastDiscarded = millis();
            asm volatile("" ::: "memory");
            _discardedBytes.push(value);
            _dll._statistics.incrementRxDiscardedBytes();
            // _dll.printMessage("IVB2: %02X", value);
            _invalid = true;
        }

        // if (!_invalid) _dll._statistics.incrementRxControlBytes(count);
        _searchBuffer.move(count);
        _awaitBytes = 1;
#ifdef TPUART_BUSMON_INTEGRITY
        // Belongs to the frame being assembled -- a flagged control byte must not tag the next one.
        _byteErrorInFrame = false;
#endif
    }

} // namespace TPUart