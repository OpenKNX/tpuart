#pragma GCC optimize("O3")
#include "TPUart/Transmitter.h"
#include "TPUart/DataLinkLayer.h"

#ifdef OPENKNX_CON_DIAG
uint16_t g_txSent = 0; // frames handed to TX_TRANSMIT (con diag)
#endif

namespace TPUart
{
    const size_t MAX_QUEUE_SIZE = 50;
    const unsigned short MAX_WAIT_TIME = 60000;

#ifndef TP_TX_PRIO_RESERVE
#define TP_TX_PRIO_RESERVE 6 // slots kept free of low-priority frames so a higher-priority frame (console/
                             // management telegram) is never starved by a bulk low-priority upload burst
#endif

    /**
     * @brief Constructs a new Transmitter object.
     *
     * @param dll Reference to a DataLinkLayer object.
     *
     * This constructor initializes the Transmitter object with the provided
     * DataLinkLayer reference. It also sets the initial values for
     * _cachedAcknowledge, _awaitResponse, and _time to 0.
     */
    Transmitter::Transmitter(DataLinkLayer &dll) : _dll(dll)
    {
        _cachedAcknowledge = 0;
        _state = TX_IDLE;
        _transmitPos = 0;
        _lastOffset = 0; // the chip resets its own offset on U_L_DataStart.req (DS p.49)
        _time = 0;
        _maxQueueSize = MAX_QUEUE_SIZE;
    }

    /**
     * @brief Destructor for the Transmitter class.
     *
     * This destructor is responsible for cleaning up the resources
     * allocated by the Transmitter instance. Specifically, it checks
     * if the _frame pointer is not null and deletes the allocated memory
     * to prevent memory leaks.
     */
    Transmitter::~Transmitter()
    {
        reset(false); // no callback: the DataLinkLayer callbacks are destroyed before this member
    }

    /**
     * @brief Finalizes the transmitter by setting the _awaitResponse flag to false.
     *
     * This function is used to indicate that the transmitter should no longer
     * await a response. It is typically called when the transmission process
     * is complete.
     */
    void Transmitter::finalize()
    {
        if (_state == TX_AWAIT) _state = TX_IDLE;
    }

    /**
     * @brief Processes the transmission queue.
     *
     * This function checks several conditions before processing the transmission queue:
     * - If a response is awaited, the function returns immediately.
     * - If the queue is empty, the function returns immediately.
     * - If the receiver is in an invalid state, the function returns immediately.
     *
     * If a frame is currently being processed, it is deleted. The next frame in the queue is then
     * retrieved and the transmission statistics are updated.
     *
     * If the chip type is TPUart2 and the frame size exceeds 64 bytes, the function returns immediately.
     *
     * Finally, the frame is transmitted.
     */
    void Transmitter::processQueue()
    {
        if (_state != TX_IDLE) return;
        if (_dll.isMonitoring()) return;
        if (_dll._receiver._invalid) return;
        if (_txCount == 0) return;

        if (_frame != nullptr)
        {
            delete _frame;
            _frame = nullptr;
            _transmitPos = 0;
            _lastOffset = 0; // reset the mirror with _transmitPos: U_L_DataStart.req of the next frame resets the chip's offset (DS p.49)
        }

        // pick the highest-priority non-empty bucket; _txCount>0 should guarantee one exists in [0..3]
        unsigned char r = 0;
        while (r < 4 && _queue[r].empty()) ++r;
        if (r == 4) // all buckets empty: stale count, never front() an empty deque
        {
            _txCount = 0;
            return;
        }
        _frame = _queue[r].front();
        _queue[r].pop();
        --_txCount;
        _dll._statistics.incrementTxFrames();

        // Fallback if the frame is too big - Filtered on DLL, too
        if (_dll._bcuType == BCU_TPUART2 && _frame->size() > 64)
        {
            delete _frame;
            _frame = nullptr;
            return;
        }

        asm volatile("" ::: "memory");
#ifdef OPENKNX_CON_DIAG
        g_txSent++;
#endif
        _state = TX_TRANSMIT;
    }

    /**
     * @brief Processes the expiration of a waiting response.
     *
     * This function checks if the transmitter is awaiting a response and if the
     * waiting time has exceeded a specified timeout (60 seconds). If the response
     * wait time has expired, reset the BCU.
     */
    void Transmitter::processWatchdog()
    {
        if (_state != TX_AWAIT) return;
        // _last could be updated in parallel, so it must be temporarily stored
        const uint32_t time = _time;

        // Backstop for the residual transient lost-CON: our frame's bus-echo was never matched, so the
        // isTransmitted-gated CON-rescue can't fire and only the 60s watchdog below recovers the stranded
        // TX. Cut that stall without racing a late CON: re-check under rxLock, require a genuinely quiet
        // interface, and throttle via a cooldown so a persistent fault degrades to the 60s cap.
#ifndef TX_BACKSTOP_MS
#define TX_BACKSTOP_MS 8000 // > worst-case legit CON latency (263B ext frame, 9600 baud, NCN 3+3 repeats)
#endif
#ifndef TX_BACKSTOP_COOLDOWN_MS
#define TX_BACKSTOP_COOLDOWN_MS 30000 // after a fast reset, only the 60s cap may fire for this long
#endif
#ifndef TPUART_RX_TIMEOUT
#define TPUART_RX_TIMEOUT 5 // mirror of Receiver.cpp (RX inter-byte / idle-bus quiet window, ms)
#endif
#ifndef TX_BACKSTOP_INVALID_MS
#define TX_BACKSTOP_INVALID_MS 12000 // latched-_invalid on a quiet bus: longer window, give _invalid a chance to clear first
#endif
        if (millis() - time >= TX_BACKSTOP_MS &&
            (_lastBackstopReset == 0 || millis() - _lastBackstopReset >= TX_BACKSTOP_COOLDOWN_MS))
        {
            bool doReset = false;
            _dll.rxLock(true);
            // Re-test everything under the lock: still stranded, receiver healthy-idle, bus quiet, and no
            // byte pending (a pending byte may be the real CON -> let the normal path finalize first).
            if (_state == TX_AWAIT &&
                _dll._receiver._state == RX_IDLE &&
                !_dll._receiver._invalid &&
                !_dll._interface->available() &&
                millis() - _dll._receiver._lastReceivedTime >= TPUART_RX_TIMEOUT)
            {
                doReset = true;
            }
            _dll.rxUnlock(); // MUST unlock before reset(): reset() re-takes rxLock (non-recursive on RP2040)

            // Re-read _state after unlock -- if a late CON finalized us in the gap, skip the needless reset.
            if (doReset && _state == TX_AWAIT)
            {
                _lastBackstopReset = millis();
                _dll._statistics.incrementBcuResets();
                _dll.printError("Watchdog: confirm lost (rx idle), fast reset.");
                _dll.reset();
                return;
            }
        }

        // Latched-_invalid sub-case: after a bus-error storm the
        // receiver latches _invalid; on a QUIET bus its clear-condition never triggers, so _invalid stays
        // set -> the branch above (gated !_invalid) is blocked, the isTransmitted CON-rescue can't fire
        // and even the 1Hz keep-alive probe is suppressed while _invalid. Only the 60s
        // watchdog recovers. A latched _invalid that won't drain on a quiet bus IS the proof of a dead
        // transmit -> force a real reset after a longer window (gives _invalid a chance to clear first).
        // Separate `if` (not else-if) so it is evaluated even when the branch above ran-but-didn't-reset;
        // the gates are mutually exclusive (_invalid vs !_invalid) so the two can never double-fire.
        if (millis() - time >= TX_BACKSTOP_INVALID_MS &&
            (_lastBackstopReset == 0 || millis() - _lastBackstopReset >= TX_BACKSTOP_COOLDOWN_MS))
        {
            bool doReset = false;
            _dll.rxLock(true);
            // Re-test under the lock: still stranded, receiver latched _invalid, bus genuinely quiet
            // (a pending byte could still clear _invalid or be a late CON -> let the normal path run first).
            if (_state == TX_AWAIT &&
                _dll._receiver._invalid &&
                !_dll._interface->available() &&
                millis() - _dll._receiver._lastReceivedTime >= TPUART_RX_TIMEOUT)
            {
                doReset = true;
            }
            _dll.rxUnlock(); // MUST unlock before reset(): reset() re-takes rxLock (non-recursive on RP2040)

            // Re-read _state after unlock -- if a late CON finalized us in the gap, skip the needless reset.
            if (doReset && _state == TX_AWAIT)
            {
                _lastBackstopReset = millis();
                _dll._statistics.incrementBcuResets();
                _dll.printError("Watchdog: stuck _invalid (quiet bus), fast reset.");
                _dll.reset();
                return;
            }
        }

        if (millis() - time < 60000) return;

#ifdef TPUART_BCU_DEBUG
        // Diagnostic dump at the moment the watchdog gives up: which lost-CON case stranded the TX?
        //   rxState 5 (RX_FRAME_WAIT_ACKN) && rxTx=1 -> our echo WAS matched, the isTransmitted rescue
        //                                               should have fired (=> bug in the rescue), while
        //   rxTx=0 / rxState!=5            -> the echo was never matched (no signal the rescue can use).
        // RxState: 0=IDLE 1=FRAME 2=DEST 3=SIZE 4=COMPLETE 5=WAIT_ACKN.
        _dll.printError("WD reset: rxState=%u rxTx=%u rxInval=%u await=%u pos=%u sinceByte=%lu sinceDisc=%lu",
                        (unsigned)_dll._receiver._state,
                        (unsigned)_dll._receiver._searchBuffer.frame().isTransmitted(),
                        (unsigned)_dll._receiver._invalid,
                        (unsigned)_dll._receiver._awaitBytes,
                        (unsigned)_dll._receiver._searchBuffer.position(),
                        (unsigned long)(millis() - _dll._receiver._lastReceivedTime),
                        (unsigned long)(millis() - _dll._receiver._lastDiscarded));
#endif
        _dll._statistics.incrementBcuResets();
        _dll.printError("Watchdog: Transmitter did not get confirm.");
        _dll.reset();
    }

    /*
     * Each frame must be initiated with a U_L_DATA_START_REQ and each subsequent byte with another position byte (6 bits).
     * Since the position byte consists of the U_L_DATA_START_REQ + position and we start with 0 anyway, no further
     * distinction is necessary.
     *
     * However, the last byte (checksum) uses the U_L_DATA_END_REQ + position!
     * Additionally, there is another peculiarity with extended frames that can be up to 263 bytes long, where 6 bits are no longer sufficient.
     * Here, a U_L_DATA_OFFSET_REQ + position (3 bits) must be prefixed. Thus, 9 bits are available for the position.
     */

    void Transmitter::processTransmitByte()
    {
        if (_state != TX_TRANSMIT) return;
        // if (!_awaitResponse) return;
        if (!_dll.txLock()) return;
        // Double check
        if (_state != TX_TRANSMIT) return;

        const unsigned short size = _frame->size();
        // if (_transmitPos >= size)
        // {
        //     _dll.txUnlock();
        //     return;
        // }

        // _dll.printMessage("Transmitting %u of %u", _transmitPos, size);
        const bool last = _transmitPos == (size - 1);
        const unsigned char offset = (_transmitPos >> 6);
        const unsigned char position = (_transmitPos & 0x3F);

        // Offset is sticky: the NCN5130 keeps it until changed (DS p.42) -> resend only on change.
        // (The mid-frame U_L_DataStart.req at each 64-B block does NOT reset it -- p.49 means a new frame only.)
        if (offset != _lastOffset)
        {
            _dll._interface->write(U_L_DATA_OFFSET_REQ | offset);
            _lastOffset = offset;
        }

        if (last) // Last byte (Checksum) - the transmit
        {
            _dll._interface->write(U_L_DATA_END_REQ | position);
        }
        else
        {
            _dll._interface->write(U_L_DATA_START_REQ | position);
        }

        _dll._interface->write(_frame->data(_transmitPos));
        if (last)
        {
            resetWatchdogTimer();
            _state = TX_AWAIT;
        }

        _transmitPos++;
        _dll.txUnlock();

        sendCachedAcknowledge();
    }

    /**
     * @brief Adds a frame to the transmission queue.
     *
     * This function attempts to add a frame to the transmission queue. If the queue
     * has reached its maximum size, the frame will not be added and the function
     * will return false.
     *
     * @param frame Pointer to the Frame object to be added to the queue.
     * @return true if the frame was successfully added to the queue, false if the queue is full.
     */
    bool Transmitter::pushQueue(Frame *frame)
    {
        const unsigned char r = frame->priorityRank();
        // low priority (rank 3) is bounded below the cap so TP_TX_PRIO_RESERVE slots stay available for a
        // higher-priority frame; system/urgent/normal may use the full cap.
        const size_t limit = (r == 3)
                                 ? (_maxQueueSize > TP_TX_PRIO_RESERVE ? _maxQueueSize - TP_TX_PRIO_RESERVE : 0)
                                 : _maxQueueSize;
        if (_txCount >= limit) return false;

        _queue[r].push(frame);
        ++_txCount;
        return true;
    }

    /**
     * @brief Checks if the transmitter is awaiting a response.
     *
     * This function returns the status of the transmitter's response waiting state.
     *
     * @return true if the transmitter is awaiting a response, false otherwise.
     */
    bool Transmitter::awaitResponse()
    {
        return _state == TX_AWAIT;
    }

    /**
     * @brief Returns the current size of the queue.
     *
     * This function retrieves the number of elements currently stored in the queue.
     *
     * @return size_t The number of elements in the queue.
     */
    size_t Transmitter::queueSize()
    {
        return _txCount;
    }

    /**
     * @brief Resets the Transmitter: drops the in-flight frame and the queue entries present on entry.
     *
     * Main-loop context only (handleReset, DataLinkLayer::reset, the TX/desync watchdogs). Every dropped
     * frame is reported through DataLinkLayer::droppedFrame() first, so the layer above can confirm it
     * negatively instead of waiting for its transport timeout. The callback runs OUTSIDE txLock: it calls
     * up into the stack and must not block the RX-context acknowledge path.
     *
     * @param notify false skips the callback (destructor only).
     */
    void Transmitter::reset(bool notify /* = true */)
    {
        // rxLock first, then txLock: the receiver takes them in that order (processReceviedByte holds
        // rxLock and calls sendAcknowledge -> txLock), so detaching in the same order cannot invert the
        // hierarchy. It matters because the receiver reads _frame via currentFrame() to match our TX echo,
        // and it does so entirely within one rxLock hold -- detaching under that lock means no snapshot of
        // the pointer can still be in use once we own it, so the delete below is safe without the lock.
        _dll.rxLock(true);
        _dll.txLock(true);
        Frame *frame = _frame;
        // _frame is not cleared when a frame completes: finalize() only sets TX_IDLE and processQueue()
        // deletes it on the next send. Reporting it unconditionally would emit a second, negative
        // L_Data.con for a telegram already confirmed positively -- a tunnel client sees two answers.
        const bool inFlight = (_state == TX_TRANSMIT || _state == TX_AWAIT);
        _frame = nullptr;
        _transmitPos = 0;
        _lastOffset = 0; // reset the mirror with _transmitPos: U_L_DataStart.req of the next frame resets the chip's offset (DS p.49)
        _state = TX_IDLE;
        resetWatchdogTimer();
        const size_t pending = _txCount;
        _dll.txUnlock();
        _dll.rxUnlock();

        if (frame != nullptr)
        {
            if (notify && inFlight) _dll.droppedFrame(*frame);
            delete frame;
        }

        // One frame per lock hold, bounded by the entry count taken above, so a callback that re-queues
        // cannot extend the loop. A frame the callback pushes is drained too while that budget remains
        // and stays queued once it is spent -- _txCount matches the queue contents either way.
        for (size_t i = 0; i < pending; ++i)
        {
            frame = nullptr;
            _dll.txLock(true);
            for (unsigned char r = 0; r < 4 && frame == nullptr; ++r)
                if (!_queue[r].empty())
                {
                    frame = _queue[r].front();
                    _queue[r].pop();
                    --_txCount;
                }
            _dll.txUnlock();

            if (frame == nullptr) break;
            if (notify) _dll.droppedFrame(*frame);
            delete frame;
        }
    }

    /**
     * @brief Sends an acknowledge message.
     *
     * This function sends an acknowledge message of the specified type. If the transmitter lock is
     * taken, the acknowledge is parked and flushAcknowledge() writes it as soon as the lock frees.
     * It is discarded once TPUART_ACK_WINDOW_US has elapsed -- past that the chip would apply it to
     * whatever frame it is receiving then, acknowledging a foreign telegram and suppressing its repetition.
     *
     * @param acknowledge The type of acknowledge message to send.
     */
    void Transmitter::sendAcknowledge(AcknowledgeType acknowledge)
    {
        if (_dll.txLock())
        {
            _dll._interface->write(U_ACK_REQ | acknowledge);
            _dll.txUnlock();
        }
        else
        {
            _cachedAcknowledge = U_ACK_REQ | acknowledge;
        }
    }

    void Transmitter::sendCachedAcknowledge()
    {
        if (!_cachedAcknowledge) return;

        if (_dll.txLock())
        {
            _dll._interface->write(_cachedAcknowledge);
            _cachedAcknowledge = 0;
            _dll.txUnlock();
        }
    }

    /**
     * @brief Sets the maximum size of the queue.
     *
     * This function sets the maximum number of elements that the queue can hold.
     *
     * @param size The maximum number of elements for the queue.
     */
    void Transmitter::setQueueSize(unsigned long size)
    {
        _maxQueueSize = size;
    }

    Frame *Transmitter::currentFrame()
    {
        return _frame;
    }

    bool Transmitter::isTransmitting()
    {
        return _state == TX_TRANSMIT;
    }

    void Transmitter::resetWatchdogTimer()
    {
        _time = millis();
    }

} // namespace TPUart
