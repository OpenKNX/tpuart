#pragma GCC optimize("O3")
#include "TPUart/Transmitter.h"
#include "TPUart/DataLinkLayer.h"

namespace TPUart
{
    const size_t MAX_QUEUE_SIZE = 50;
    const unsigned short MAX_WAIT_TIME = 60000;

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
        reset();
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
        if (_dll._receiver._invalid) return;
        if (_queue.empty()) return;

        if (_frame != nullptr)
        {
            delete _frame;
            _frame = nullptr;
            _transmitPos = 0;
            _lastOffset = 0; // reset the mirror with _transmitPos: U_L_DataStart.req of the next frame resets the chip's offset (DS p.49)
        }

        _frame = _queue.front();
        _queue.pop();
        _dll._statistics.incrementTxFrames();

        // Fallback if the frame is too big - Filtered on DLL, too
        if (_dll._bcuType == BCU_TPUART2 && _frame->size() > 64)
        {
            delete _frame;
            _frame = nullptr;
            return;
        }

        asm volatile("" ::: "memory");
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
        if (_queue.size() >= _maxQueueSize) return false;

        _queue.push(frame);
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
        return _queue.size();
    }

    /**
     * @brief Resets the Transmitter by clearing the message queue and resetting the state.
     *
     * This function performs the following actions:
     * - Empties the message queue and deletes each message.
     * - Resets the _awaitResponse flag to false.
     * - Ensures memory barriers are respected using inline assembly.
     * - Deletes the current frame if it exists.
     */
    void Transmitter::reset()
    {
        _dll.txLock(true);
        while (!_queue.empty())
        {
            delete _queue.front();
            _queue.pop();
        }

        if (_frame != nullptr)
        {
            delete _frame;
            _frame = nullptr;
            _transmitPos = 0;
            _lastOffset = 0; // reset the mirror with _transmitPos: U_L_DataStart.req of the next frame resets the chip's offset (DS p.49)
        }

        _state = TX_IDLE;
        resetWatchdogTimer();
        _dll.txUnlock();
    }

    /**
     * @brief Sends an acknowledge message.
     *
     * This function sends an acknowledge message of the specified type. If the
     * transmitter lock is acquired successfully, the acknowledge message is sent
     * immediately. Otherwise, the acknowledge message is cached for later
     * transmission.
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
