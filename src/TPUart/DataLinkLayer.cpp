#pragma GCC optimize("O3")
#include "TPUart/DataLinkLayer.h"
#include <stdarg.h>

// NCN thermal-warning (TW) handling tunables. Overridable via build flags (central TPUART flag config).
#ifndef TPUART_TW_HEARTBEAT_MS
#define TPUART_TW_HEARTBEAT_MS 60000 // while TW stays set: at most one "still set" line this often
#endif
#ifndef TPUART_TW_AUTORESET_AFTER_MS
#define TPUART_TW_AUTORESET_AFTER_MS 45000 // TPUART_NCN_TW_AUTORESET: let TW persist this long before the heal reset
#endif
#ifndef TPUART_TW_AUTORESET_COOLDOWN_MS
#define TPUART_TW_AUTORESET_COOLDOWN_MS 300000 // minimum spacing between two auto-resets (cross-episode backoff)
#endif
#ifndef TPUART_TW_RELAPSE_MS
#define TPUART_TW_RELAPSE_MS 8000 // TW still set this long after the reset => genuinely hot, not a latch
#endif

namespace TPUart
{
    SystemState &DataLinkLayer::getSystemState()
    {
        return _systemState;
    }

    BcuType DataLinkLayer::getBcuType() { return _bcuType; }
    uint8_t DataLinkLayer::getNcnAsr0() { return _ncnAsr0; }
    uint8_t DataLinkLayer::getNcnRevId() { return _ncnRevId; }
    bool DataLinkLayer::ncnRegValid() { return _ncnRegValid; }

    // Synchronous single-register read. ONLY valid inside the init busy-wait: the request reply is a
    // single header-less data byte, which is only unambiguous while no async bus traffic contends the UART.
    uint8_t DataLinkLayer::readRegisterBlocking(char readOpcode)
    {
        _interface->write(readOpcode); // read request = opcode only, no data byte
        unsigned long start = millis();
        do
        {
            int value = _interface->read();
            if (value >= 0) return (uint8_t)value; // first reply byte = register content
        } while ((millis() - start) < 20);
        return 0; // no reply (non-NCN chip / register absent / timeout)
    }

    // One-shot RevID + ASR0 snapshot during init. RevID (0x05) exists only on NCN5121/5130; on an
    // NCN5120 the read yields no valid code -> _ncnRevId stays 0 -> reported as "NCN5120". ASR0 (0x03)
    // is present on the whole NCN family and carries the rails + TW + latched TSD (thermal-shutdown) bit.
    // Registers per NCN5130/D Rev.8 p.56: ASR0 Table 20/21, RevID Table 22/23.
    void DataLinkLayer::readNcnRegisterSnapshot()
    {
        if (_bcuType != BCU_NCN5120) return; // register reads are NCN-family only
        _ncnRevId = readRegisterBlocking(U_INT_REG_RD_REQ_RID);
        _ncnAsr0 = readRegisterBlocking(U_INT_REG_RD_REQ_ASR0);
        _ncnRegValid = true;
    }

    void DataLinkLayer::begin(BcuType bcuType, Interface::Abstract *interface)
    {
        _bcuType = bcuType;
        _interface = interface;
        if (_interface->hasCallback()) _interface->registerCallback(std::bind(&DataLinkLayer::processReceviedByte, this));
        _initialized = true;
        tryInitialize();
    }

    void DataLinkLayer::tryInitialize()
    {
        if (_bcuState != BCU_UNINITIALIZED) return;
        _lastTryInitialize = millis();

        uint baudrates[2] = {19200, 38400};
        for (uint baudrate : baudrates)
        {
            if (_bcuType == BCU_TPUART2 && baudrate != 19200) continue;

            if (tryInitialize(baudrate))
            {
                setBCUState(BCU_CONNECTED, baudrate);
                _uReset = true;
                _bcuState = BCU_CONNECTED;
                return;
            }
        }
    }

    bool DataLinkLayer::tryInitialize(uint baudrate)
    {
        printMessage("Try Initialize %d", baudrate);

        _interface->end(); // End interface is already initialized
        _interface->begin(baudrate);
        _interface->write(U_RESET_REQ);
        unsigned long start = millis();
        do
        {
#ifdef ARDUINO_ARCH_ESP32
            vTaskDelay(pdMS_TO_TICKS(1));
#endif
            int value = _interface->read();
            if (value == -1) continue; // Queu is empty
            if (value == 0) continue;  // TPUart send zeros at the beginning
            // Serial.printf("%02X", value);

            // War direkt erfolgreich - Top
            if (value == U_RESET_IND)
            {
                // Safe here: synchronous, before the async RX pump owns the stream and before any
                // bus/tunnel traffic -> the header-less register replies cannot be mis-captured.
                readNcnRegisterSnapshot();
                return true;
            }
            break;
        }
        while (!((millis() - start) >= 50));
        return false;
    }

    void DataLinkLayer::reset()
    {
        if (_bcuState == BCU_UNINITIALIZED) return;
        printMessage("Reset BCU");

        rxLock(true);
        _transmitter.reset();
        _uState = 0;
        _uReset = false;
        _receiver.reset();
        _statistics.reset();
        _rxFrameBuffer.clear();
        _rxFrameBufferEntries = 0;
        _repetitionFilter.clear();
        _interface->flush();
        _bcuState = BCU_CONNECTED;

        _interface->write(U_RESET_REQ);
        _modeExtendedCRC = false;
        _lastDiscardedBytes = 0;
        _receiver._invalid = false;
        // Re-arm RX liveness on EVERY reset (all build flavors), so a reset on an already-quiet bus does
        // not make the very next process() count a spurious CONNECTED->DISCONNECTED.
        _receiver._lastReceivedTime = millis();
        // Seed the valid-protocol-rx liveness + reconnect throttle.
        _lastValidRx = millis();
        _lastReconnectAttempt = millis();
        _reconnectBackoff = 1000;
        rxUnlock();
    }

#ifdef TPUART_BCU_DEBUG
    void DataLinkLayer::forceDisconnect()
    {
        // Test hook (console: "bcu dis"): reproduce the terminal "BCU disconnected" on demand.
        // Latch _invalid and drop to DISCONNECTED; the auto-reconnect poke then recovers it automatically.
        printMessage("BCU force-disconnect (test)");
        rxLock(true);
        _receiver._invalid = true;
        setBCUState(BCU_DISCONNECTED);
        rxUnlock();
    }
#endif

    const char *DataLinkLayer::getBcuStateInfo()
    {
        switch (_bcuState)
        {
            case BCU_DISCONNECTED:
                return "Disconnected";
            case BCU_CONNECTED:
                return "Connected";
            case BCU_BUSMONITOR:
                return "Busmonitor";
            default:
                return "Uninitialized";
        }
    }

    int DataLinkLayer::getBaudrate()
    {
        return _baudrate;
    }

    void DataLinkLayer::processRxFrameBuffer()
    {
        if (!_rxFrameBufferEntries) return;

        char run = 0;
        while (_rxFrameBufferEntries && (TPUART_MAX_RXQUEUE_TIME_PER_LOOP == 0 || run < TPUART_MAX_RXQUEUE_TIME_PER_LOOP))
        {
            rxLock(true);
            const uint16_t bufferSize = _rxFrameBuffer.pop() + (_rxFrameBuffer.pop() << 8);
            const uint16_t frameSize = bufferSize - 3;

            // Bound what was a VLA (char frameData[frameSize]): the producer (pushRxFrameBuffer) never pushes
            // more than the rx search buffer holds, so a frameSize above it means the ring desynced or the
            // 2-byte length header underflowed (bufferSize < 3 -> frameSize wraps to ~65533). Either way the
            // read pointer is untrustworthy; sizing a stack array from it would overflow the stack. Drop the
            // whole buffer and resync (same reset idiom as the init/overflow paths) instead of indexing wild.
            if (frameSize > TPUART_RX_SEARCH_BUFFER_SIZE)
            {
                _rxFrameBuffer.clear();
                _rxFrameBufferEntries = 0;
                rxUnlock();
                _rxFrameBufferOverflow = true;
                _statistics.incrementRxFrameBufferOverflow();
                return;
            }

            char frameData[TPUART_RX_SEARCH_BUFFER_SIZE]; // fixed max; only the first frameSize bytes are written+read below

            for (size_t i = 0; i < frameSize; i++)
                frameData[i] = _rxFrameBuffer.pop();

            Frame frame(frameData, frameSize);

            frame.addFlags(_rxFrameBuffer.pop());
            asm volatile("" ::: "memory");
            _rxFrameBufferEntries = _rxFrameBufferEntries - 1;
            rxUnlock();

            run++;

            // Skip the repetition filter for busmon-only carriers (FCS-failed frames and 1-byte standalone
            // acknowledges): they must not pollute the per-source repetition map, are never delivered up,
            // and a 1-byte ack carrier has no valid size()/source() for the filter to read.
            if (!frame.isErrored() && !frame.isAckOnly())
            {
                bool alreadFound;
                alreadFound = _repetitionFilter.check(frame);
                if (frame.isRepeated() && alreadFound)
                {
                    frame.setFiltered();
                    _statistics.incrementRxRepetitions();
                }
            }

            // Complete TX Frame
            for (std::function<void(Frame &)> &callback : _callbacksReceivedFrame)
                callback(frame);
        }
    }

    void DataLinkLayer::pushRxFrameBuffer(Frame &frame)
    {
        _lastValidRx = millis(); // a parsed frame -> bus/chip alive -> liveness for auto-reconnect
        const unsigned short frameSize = frame.size();
        const unsigned short bufferSize = frameSize + 3;
        if (_rxFrameBuffer.available() <= bufferSize)
        {
            _rxFrameBufferOverflow = true;
            _statistics.incrementRxFrameBufferOverflow();
            return;
        }

        // Size (2 byte)
        _rxFrameBuffer.push(bufferSize & 0xFF);
        _rxFrameBuffer.push(bufferSize >> 8);

        // Frame
        for (size_t i = 0; i < frameSize; i++)
            _rxFrameBuffer.push(frame.data(i));

        // Flags (1 byte)
        _rxFrameBuffer.push(frame.flags());
        asm volatile("" ::: "memory");
        _rxFrameBufferEntries = _rxFrameBufferEntries + 1;
    }

    void DataLinkLayer::pushRxAcknByte(char value)
    {
        // A standalone L2 acknowledge (ACK/NAK/BUSY) is forwarded to the busmon as a 1-byte carrier. It
        // rides the normal rx-frame ring so the forward happens in loop context (processRxFrameBuffer),
        // and is tagged ACK_ONLY so the consumer short-circuits it to the busmon without ever calling
        // size()/isFrame()/cemi conversion (which would read past this single byte). Kept separate from
        // pushRxFrameBuffer, whose frame.size() would itself read past a 1-byte buffer.
        const unsigned short bufferSize = 1 + 3;
        if (_rxFrameBuffer.available() <= bufferSize)
        {
            _rxFrameBufferOverflow = true;
            _statistics.incrementRxFrameBufferOverflow();
            return;
        }
        _rxFrameBuffer.push(bufferSize & 0xFF);
        _rxFrameBuffer.push(bufferSize >> 8);
        _rxFrameBuffer.push(value);
        _rxFrameBuffer.push(TP_FRAME_FLAG_ACK_ONLY);
        asm volatile("" ::: "memory");
        _rxFrameBufferEntries = _rxFrameBufferEntries + 1;
    }

    DataLinkLayer::DataLinkLayer() : _transmitter(*this), _receiver(*this)
    {
#if defined(ARDUINO_ARCH_RP2040)
        mutex_init(&_rxLock);
        mutex_init(&_txLock);
#elif defined(ARDUINO_ARCH_ESP32)
        _rxLock = xSemaphoreCreateMutex();
        _txLock = xSemaphoreCreateMutex();
#endif
    }

    bool DataLinkLayer::rxLock(bool blocking /* = false */)
    {
#if defined(ARDUINO_ARCH_RP2040)
        if (blocking)
        {
            mutex_enter_blocking(&_rxLock);
            return true;
        }

        uint32_t owner;
        return mutex_try_enter(&_rxLock, &owner);
#elif defined(ARDUINO_ARCH_ESP32)
        TickType_t wait = blocking ? 0xFFFFFFFF : 0;
        return xSemaphoreTake(_rxLock, wait) == pdTRUE;
#else
        return true;
#endif
    }

    void DataLinkLayer::rxUnlock()
    {

#if defined(ARDUINO_ARCH_RP2040)
        mutex_exit(&_rxLock);
#elif defined(ARDUINO_ARCH_ESP32)
        xSemaphoreGive(_rxLock);
#endif
    }

    bool DataLinkLayer::txLock(bool blocking /* = false */)
    {
#if defined(ARDUINO_ARCH_RP2040)
        if (blocking)
        {
            mutex_enter_blocking(&_txLock);
            return true;
        }

        uint32_t owner;
        return mutex_try_enter(&_txLock, &owner);
#elif defined(ARDUINO_ARCH_ESP32)
        TickType_t wait = blocking ? 0xFFFFFFFF : 0;
        return xSemaphoreTake(_txLock, wait) == pdTRUE;
#else
        return true;
#endif
    }

    void DataLinkLayer::txUnlock()
    {
#if defined(ARDUINO_ARCH_RP2040)
        mutex_exit(&_txLock);
#elif defined(ARDUINO_ARCH_ESP32)
        xSemaphoreGive(_txLock);
#endif
    }

    void DataLinkLayer::end(bool deleteUart)
    {
        _initialized = false;
        _bcuState = BCU_UNINITIALIZED;
        if (deleteUart && _interface == nullptr)
        {
            delete _interface;
        }
    }

    /*
     * Callback for the interface to process the received byte.
     * Returns true if there are more bytes to process.
     */
    bool DataLinkLayer::processReceviedByte()
    {
        if (_bcuState == BCU_UNINITIALIZED) return false;

        return _receiver.processReceviedByte();
    }

    void DataLinkLayer::processTransmitByte()
    {
        if (_bcuState == BCU_UNINITIALIZED) return;

        _transmitter.processTransmitByte();
    }

    void DataLinkLayer::process()
    {
        if (!_initialized) return;
        if (_bcuState == BCU_UNINITIALIZED && _interface->available())
        {
            if (millis() - _lastTryInitialize > 1000) tryInitialize();
        }

        if (_bcuState == BCU_UNINITIALIZED) return;

        if (_bcuState == BCU_DISCONNECTED && millis() - _lastReconnectAttempt >= _reconnectBackoff)
        {
            // Active reconnect: the NCN went mute/desynced. Clean the parser, flush, poke U_RESET_REQ -- all
            // under rxLock, no UART teardown. The only way back to CONNECTED is receivedReset() on a genuine
            // U_RESET_IND (parsed unconditionally even while _invalid), so a wedged chip dribbling noise can't
            // look alive.
            _lastReconnectAttempt = millis();
            printMessage("BCU reconnect: U_RESET_REQ");
            rxLock(true);
            _receiver.reset();
            _interface->flush();
            _interface->write(U_RESET_REQ);
            rxUnlock();
            // Back off 1s -> 30s while the NCN stays absent (gentle polling, no U_RESET_REQ flood); resets to
            // 1s on a real reconnect and in reset(). Still recovers <=30s after the chip returns.
            _reconnectBackoff *= 2;
            if (_reconnectBackoff > 30000) _reconnectBackoff = 30000;
        }

        exitBusyModeTimer();
        processRequestState();

        handleReset();
        unsigned long start;

        start = millis();
        // if (!_interface->hasCallback())
        while (_interface->available())
            processReceviedByte();

        _receiver.process();
        _transmitter.processQueue();

        start = millis();
        while (_transmitter.isTransmitting())
        {
            processTransmitByte();
#ifdef ARDUINO_ARCH_ESP32
            taskYIELD(); // bytes go to the non-blocking HW TX FIFO; the TP wire serializes them anyway (no per-byte vTaskDelay tax)
#endif
            if (millis() - start > 20) break; // fairness/watchdog bound
        }

        processRxFrameBuffer();

        // The CON for the just-sent frame may have been parsed in this same process() pass; if TX is now idle,
        // pop+arm the NEXT queued frame now instead of stalling a full main-loop period. Still STRICTLY
        // one-in-flight: processQueue() returns unless _state==TX_IDLE -- no pipelining past an unconfirmed CON.
        _transmitter.processQueue();

        // Show state changes and errors
        showOverflowError();
        showStateError();
        showDiscardedError();
        showSystemState();
        showThermalWarning();

        // check nothing received
        const unsigned long last = _receiver._lastReceivedTime;
        const unsigned long lastValid = _lastValidRx;
        asm volatile("" ::: "memory"); // ensures that the last value is not updated again by the parallel task or interrupt after the query of millis()
        const unsigned long current = millis();
        // Disconnect only when BOTH are stale: no byte at all for 5s (bus dead) AND no valid protocol RX for
        // 30s (chip not answering the 1Hz state probe). A busy-but-healthy burst keeps the 5s timer fresh, so
        // it can't cause a spurious mid-burst disconnect + parser flush.
        if (_bcuState == BCU_CONNECTED && current - last > 5000UL && current - lastValid > 30000UL)
        {
            // printMessage("Time: %li - %li = %li", current, _receiver._lastReceivedTime, current - last);
            setBCUState(BCU_DISCONNECTED);
        }

        processWatchdog();
    }

    void DataLinkLayer::processWatchdog()
    {
        if (_bcuState == BCU_UNINITIALIZED) return;

        _transmitter.processWatchdog();
    }

    void DataLinkLayer::processRequestState()
    {
        if (_receiver._invalid) return;
        if ((millis() - _requestStateTimer < 1000)) return;

        requestState();

        _requestStateTimer = millis();
    }

    void DataLinkLayer::handleReset()
    {
        if (!_uReset) return;

        printMessage("Reset received");
        _uReset = false;

        applyConfiguration();
        requestState();
    }

    void DataLinkLayer::registerMessage(std::function<void(const char *, bool)> callback)
    {
        _callbackMessage = callback;
    }

    void DataLinkLayer::registerReceivedFrame(std::function<void(Frame &)> callback)
    {
        _callbacksReceivedFrame.push_back(callback);
    }

    void DataLinkLayer::registerCheckAcknowledge(std::function<AcknowledgeType(uint16_t, bool)> callback)
    {
        _callbackCheckAcknowledge = callback;
    }

    AcknowledgeType DataLinkLayer::checkAcknowledge(unsigned short destination, bool isGroupAddress)
    {
        if (!_callbackCheckAcknowledge) return ACK_None;

        return _callbackCheckAcknowledge(destination, isGroupAddress);
    }

    Statistics &DataLinkLayer::getStatistics()
    {
        return _statistics;
    }

    void DataLinkLayer::showOverflowError()
    {
        if (!(_rxShowOverflowTime == 0 || (millis() - _rxShowOverflowTime >= 1000))) return;
        _rxShowOverflowTime = millis();

        if (_rxSearchBufferOverflow)
        {
            _rxSearchBufferOverflow = false;
            printError("SearchBuffer Overflow!");
        }

        if (_rxFrameBufferOverflow)
        {
            _rxFrameBufferOverflow = false;
            printError("FrameBuffer Overflow!");
        }

        if (_rxInterfaceOverflow)
        {
            _rxInterfaceOverflow = false;
            printError("Interface Overflow!");
        }
    }

    void DataLinkLayer::setRepetitions(uint8_t nack, uint8_t busy)
    {
        _repetitions = (nack & 0b111) | ((busy & 0b111) << 4);
    }

    void DataLinkLayer::requestState()
    {
        if (_bcuState == BCU_UNINITIALIZED) return;
        if (_bcuState == BCU_BUSMONITOR) return;

        // printMessage("requestState");

        txLock(true);
        _interface->write(U_STATE_REQ);
        if (_bcuType == BCU_NCN5120) _interface->write(U_SYSTEM_STATE_REQ);
        txUnlock();
    }

    /*
     * Apply the configuration to the BCU depended on the chip type.
     *
     * The configuration is only applied if the chip is initialized and not in monitoring mode.
     */
    void DataLinkLayer::applyConfiguration()
    {
        if (_bcuState == BCU_UNINITIALIZED) return;
        if (_bcuState == BCU_BUSMONITOR) return;

        // printMessage("applyConfiguration");
        txLock(true);

        // Enable extednded CRC16
        if (_bcuType == BCU_NCN5120)
            _interface->write(U_NCN5120_CONFIGURE_REQ | U_NCN5120_CONFIGURE_CRC_CCITT_REQ);
        else if (_bcuType == BCU_TPUART2)
        {
            _interface->write(U_TPUART2_ACTIVATECRC_REQ);
            _modeExtendedCRC = true;
        }

        // Set Address for AutoACK Unicast
        {
            if (_ownAddress > 0)
            {
                char buffer[10];
                sprintf(buffer, "%u.%u.%u", (_ownAddress >> 12 & 0b1111), (_ownAddress >> 8 & 0b1111), (_ownAddress & 0b11111111));
                if (_bcuType == BCU_NCN5120) _interface->write(U_NCN5120_SET_ADDRESS_REQ);
                if (_bcuType == BCU_TPUART2) _interface->write(U_TPUART2_SET_ADDRESS_REQ);
                _interface->write((_ownAddress >> 8) & 0xFF);
                _interface->write(_ownAddress & 0xFF);
                if (_bcuType == BCU_NCN5120) _interface->write(0xFF); // Dummy Byte needed by NCN only
            }
        }

        // Abweichende Config
        if (_repetitions != 0b00110011)
        {
            if (_bcuType == BCU_NCN5120)
            {
                _interface->write(U_NCN5120_SET_REPETITION_REQ);
                _interface->write(_repetitions);
                _interface->write(0x0); // dummy, see NCN5120 datasheet
                _interface->write(0x0); // dummy, see NCN5120 datasheet
            }
            else if (_bcuType == BCU_TPUART2)
            {
                _interface->write(U_TPUART2_SET_REPETITION_REQ);
                _interface->write(((_repetitions & 0xF0) << 1) || (_repetitions & 0x0F));
            }
        }

        txUnlock();
    }

    /*
     * Print n message by using the registered callback.
     */
    void DataLinkLayer::printMessage(const char *format, ...)
    {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        if (_callbackMessage) _callbackMessage(buffer, false);
    }

    /*
     * Print an error message by using the registered callback.
     */
    void DataLinkLayer::printError(const char *format, ...)
    {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        if (_callbackMessage) _callbackMessage(buffer, true);
    }

    /*
     * push a new frame to the transmit queue.
     * if the transmit queue is full, the return is false.
     *
     * Important! The frame is not copied, only a pointer is stored.
     * The frame will only delete wenn the frame is send.
     * Is the queue full, the frame must be deleted by the caller.
     */
    bool DataLinkLayer::pushTransmitQueue(Frame *frame)
    {
        if (_bcuState == BCU_UNINITIALIZED) return false;
        if (_bcuState == BCU_BUSMONITOR) return false;

        if (_bcuType == BCU_TPUART2 && frame->size() > 64) return false;

        if (!_transmitter.pushQueue(frame)) return false;

        return true;
    }

    /*
     * Return the receiver object for direct access.
     */
    Receiver &DataLinkLayer::getReceiver()
    {
        return _receiver;
    }

    /*
     * Return the transmitter object for direct access.
     */
    Transmitter &DataLinkLayer::getTransmitter()
    {
        return _transmitter;
    }

    /*
     * This function allows controlling the power supply (VCC2) of the NCN5120.
     * Note that the setting survives a restart of the host system and a reset of the BCU.
     * Anyone using this function should therefore re-enable the power supply on boot of the host system.
     */
    bool DataLinkLayer::powerControl(bool state)
    {
        if (_bcuState == BCU_UNINITIALIZED) return false;
        // if (_bcuState == BCU_BUSMONITOR) return false;
        if (_bcuType != BCU_NCN5120) return false;

        rxLock(true); // Prevent sending ACKs during sending multi bytes multibytes
        _interface->write(U_INT_REG_WR_REQ_ACR0);
        if (state)
        {
            printMessage("VCC2 power enabled");
            _interface->write(ACR0_FLAG_DC2EN | ACR0_FLAG_V20VEN | ACR0_FLAG_XCLKEN | ACR0_FLAG_V20VCLIMIT);
        }
        else
        {
            printMessage("VCC2 power disabled");
            _interface->write(ACR0_FLAG_XCLKEN | ACR0_FLAG_V20VCLIMIT);
        }
        rxUnlock();

        requestState();
        return true;
    }

    /*
     * Activate or deactivate the stop mode. In stop mode, the NCN5120 stop sending frames to host controller.
     */
    bool DataLinkLayer::stopMode(bool state)
    {
        if (_bcuState == BCU_UNINITIALIZED) return false;
        if (_bcuType != BCU_NCN5120) return false;

        txLock(true);
        if (state)
        {
            printMessage("Stop mode enabled");
            _interface->write(U_STOP_MODE_REQ);
        }
        else
        {
            printMessage("Stop mode disabled");
            _interface->write(U_EXIT_STOP_MODE_REQ);
        }
        txUnlock();

        return true;
    }

    /*
     * Actiavte the budy mode. In busy mode, all frames will be rejected with BUSY.
     * The mode will be left after 700ms. This is done to prevent the mode from being accidentally activated.
     */
    bool DataLinkLayer::busyMode(bool state)
    {
        if (_bcuState == BCU_UNINITIALIZED) return false;

        txLock(true);
        if (state)
        {
            printMessage("Busy mode enabled for around 700ms");
            if (_bcuType == BCU_NCN5120) _interface->write(U_NCN5120_SET_BUSY_REQ);
            if (_bcuType == BCU_TPUART2) _interface->write(U_TPUART2_SET_BUSY_REQ);
            _busyMode = millis();
        }
        else
        {
            printMessage("Busy mode disabled");
            if (_bcuType == BCU_NCN5120) _interface->write(U_NCN5120_QUIT_BUSY_REQ);
            if (_bcuType == BCU_TPUART2) _interface->write(U_TPUART2_QUIT_BUSY_REQ);
            _busyMode = 0;
        }
        txUnlock();

        return true;
    }

    /*
     * This function disables the busy mode after approximately 700ms.
     * The reason for the 700ms is that TPUART2 also exits the mode in hardware after 700ms, ensuring the same behavior between both chip types.
     */
    void DataLinkLayer::exitBusyModeTimer()
    {
        if (!_busyMode) return;
        if ((millis() - _busyMode) < 700) return;

        busyMode(false);
    }

    /*
     * Show the current state of the TPUart.
     */
#ifndef TPUART_STATE_ERR_THROTTLE_MS
#define TPUART_STATE_ERR_THROTTLE_MS 10000 // min ms between repeated "TP Error" lines of the SAME error set
#endif

    void DataLinkLayer::showStateError()
    {
        if (!_uState) return;

        // Snapshot + clear first (minimise the race with receivedState in RX context), then COUNT every
        // event so the health counters / `bcu` report stay exact regardless of the log throttling below.
        const char errors = _uState;
        _uState = 0;
        if (errors & SLAVE_COLLISION) _statistics.incrementBcuSlaveCollisions();
        if (errors & RECEIVE_ERROR) _statistics.incrementBcuReceiveErrors();
        if (errors & TRANSMIT_ERROR) _statistics.incrementBcuTransmitErrors();
        if (errors & PROTOCOL_ERROR) _statistics.incrementBcuProtocolErrors();

        // RATE-LIMIT the log. Probing a non-existent individual address (an ETS-style `ftc scan` over empty
        // PAs) gets no L2-ACK, which the NCN surfaces as a U_State Protocol-Error on every un-acked TX -- so
        // the scan floods one line per probe. That is the EXPECTED, benign outcome of probing addresses that
        // do not exist. Collapse a repeated IDENTICAL error set to one line per window + a suppressed-count,
        // mirroring showThermalWarning(). A NEW error set logs at once, so a real RE/TE/SC is never hidden.
        const unsigned long now = millis();
        if (errors == _stateErrLast && (now - _stateErrLogged) < TPUART_STATE_ERR_THROTTLE_MS)
        {
            _stateErrSuppressed++;
            return;
        }

        std::string errorMessage = "TP Error:";
        if (errors & SLAVE_COLLISION) errorMessage += " Slave-Collision";
        if (errors & RECEIVE_ERROR) errorMessage += " Receive-Error";
        if (errors & TRANSMIT_ERROR) errorMessage += " Transmit-Error";
        if (errors & PROTOCOL_ERROR) errorMessage += " Protocol-Error";
        if (_stateErrSuppressed)
        {
            char tail[24];
            snprintf(tail, sizeof(tail), " (+%u more)", _stateErrSuppressed);
            errorMessage += tail;
        }
        // TEMPERATURE_WARNING is a persistent condition handled in showThermalWarning() (edge-logged) and
        // masked out of _uState in receivedState(), so it never reaches this per-event transient-error path.
        printError(errorMessage.c_str());
        _stateErrLast = errors;
        _stateErrLogged = now;
        _stateErrSuppressed = 0;
    }

    /*
     * NCN thermal-warning (TW). The 1Hz state poll re-reads the NCN's TW bit, so a raw log floods one line
     * per second for the whole hot/latched episode. Track it as a condition and edge-log it: once on the
     * rising edge, an optional heartbeat while it holds, once when it clears. Runs in main-loop context
     * (safe to log / reset); receivedState() only captures the raw bit.
     */
    void DataLinkLayer::showThermalWarning()
    {
        // On a dead/reconnecting bus there are no fresh samples -- drop the condition silently (the
        // disconnect is reported elsewhere) so a stale bit cannot drive heartbeats or an auto-reset.
        if (_bcuState != BCU_CONNECTED)
        {
            _twActive = false;
            _twSampled = false;
            _twRaw = false; // clear the raw bit too, else a reconnect re-logs a stale rising edge
            return;
        }

        const unsigned long now = millis();
        const bool tw = _twRaw;

        if (_twSampled)
        {
            _twSampled = false;
            if (tw) _statistics.incrementBcuTempWarnings(); // per-poll health metric (unchanged)
        }

        if (tw && !_twActive)
        {
            _twActive = true;
            _twSince = now;
            _twLastBeat = now;
#ifdef TPUART_NCN_TW_AUTORESET
            _twAutoResetDone = false;
            _twHotLogged = false;
#endif
            printError("NCN thermal warning (TW) set");
        }
        else if (!tw && _twActive)
        {
            _twActive = false;
            const unsigned long secs = (now - _twSince) / 1000;
#ifdef TPUART_NCN_TW_AUTORESET
            if (_twAutoResetDone)
                printMessage("NCN thermal warning cleared by auto-reset (was latched, %lus)", secs);
            else
#endif
                printMessage("NCN thermal warning cleared (%lus)", secs);
        }
        else if (tw && _twActive)
        {
            if (now - _twLastBeat >= TPUART_TW_HEARTBEAT_MS)
            {
                _twLastBeat = now;
                printMessage("NCN thermal warning still set (%lus)", (now - _twSince) / 1000);
            }
#ifdef TPUART_NCN_TW_AUTORESET
            twAutoResetCheck(now);
#endif
        }
    }

#ifdef TPUART_NCN_TW_AUTORESET
    /*
     * L2: if TW persists, issue ONE U_RESET (the same clean protocol reset as "bcu rst") to clear a
     * possibly-latched TW. Exactly one per episode plus a long cross-episode cooldown, so a genuinely-hot
     * chip cannot cause a reset loop: if TW returns after the reset it is real heat -> we log that verdict
     * and stop resetting until it clears and re-triggers (subject to the cooldown).
     */
    void DataLinkLayer::twAutoResetCheck(unsigned long now)
    {
        if (_twAutoResetDone)
        {
            // Already reset this episode: TW still set well after it => the chip is genuinely hot.
            if (!_twHotLogged && (now - _twLastAutoReset) >= TPUART_TW_RELAPSE_MS)
            {
                _twHotLogged = true;
                printError("NCN TW persists through U_RESET -> genuinely hot (reduce TX load / cool down)");
            }
            return;
        }
        if ((now - _twSince) < TPUART_TW_AUTORESET_AFTER_MS) return;                                    // let it persist first
        if (_twLastAutoReset && (now - _twLastAutoReset) < TPUART_TW_AUTORESET_COOLDOWN_MS) return;     // cross-episode backoff

        _twAutoResetDone = true;
        _twLastAutoReset = now;
        printMessage("NCN TW persists %lus -> U_RESET_REQ (auto-heal latch)", (now - _twSince) / 1000);
        reset(); // U_RESET_REQ + re-arm, identical to a manual "bcu rst"
    }
#endif

    /*
     * Show the number of Discard bytes.
     */
    void DataLinkLayer::showDiscardedError()
    {
        if (!_receiver._discardedBytes.size()) return;
        if ((millis() - _lastDiscardedMessage) < 1000) return;
        if (!(_receiver._invalid || millis() - _receiver._lastDiscarded > 100 || _receiver._discardedBytes.size() > 100)) return;
        if (!rxLock()) return;

        std::string buffer;
        size_t size = _receiver._discardedBytes.size();
        while (_receiver._discardedBytes.size())
        {
            char hexBuffer[4];
            sprintf(hexBuffer, " %02X", _receiver._discardedBytes.pop());
            buffer += hexBuffer;
        }

        printError(" %u Bytes are discarded! (%s )", size, buffer.c_str());

        _lastDiscardedBytes = _statistics.getRxDiscardedBytes();
        _lastDiscardedMessage = millis();

        rxUnlock();
    }

    /*
     * Show the current system state of the NCN5120.
     */
    void DataLinkLayer::showSystemState()
    {
        if (_bcuType != BCU_NCN5120) return;
        if (!_systemState.dirty()) return;

        printMessage(_systemState.print().c_str());
    }

    /*
     * Set the own address of the BCU.
     * This is needed so that the BCU independently acknowledges frames when they are addressed to its own address.
     */
    void DataLinkLayer::setOwnAddress(short address)
    {
        _ownAddress = address;
        applyConfiguration(); // apply new address, if datalinklayer is already initialized
    };

    void DataLinkLayer::setBCUState(BcuState state, int baudrate)
    {
        if (_bcuState == state) return;

        if (state == BCU_CONNECTED)
        {
            if (baudrate == 0)
                printMessage("BCU connected");
            else
            {
                _baudrate = baudrate;
                printMessage("BCU connected (Baudrate: %d)", baudrate);
            }

            _receiver._lastReceivedTime = millis();
        }
        else if (state == BCU_DISCONNECTED)
        {
            _statistics.incrementBcuDisconnects(); // genuine transition only (guarded above)
            printMessage("BCU disconnected");
        }
        else if (state == BCU_BUSMONITOR)
        {
            printMessage("BCU in monitor mode");
        }

        _bcuState = state;
    }

    BcuState DataLinkLayer::getBcuState()
    {
        return _bcuState;
    }

    bool DataLinkLayer::startMonitoring()
    {
        if (_bcuState == BCU_UNINITIALIZED) return false;
        if (_bcuState == BCU_BUSMONITOR) return true;

        txLock(true);
        _interface->write(U_BUSMON_REQ);
        txUnlock();
        _modeExtendedCRC = false;
        setBCUState(BCU_BUSMONITOR);
        return true;
    }

    bool DataLinkLayer::isMonitoring() const
    {
        return _bcuState == BCU_BUSMONITOR;
    }

    bool DataLinkLayer::isConnected() const
    {
        return _bcuState == BCU_CONNECTED;
    }

    void DataLinkLayer::receivedState(char state)
    {
        _lastValidRx = millis(); // valid protocol RX (1Hz state-probe reply) -> liveness for auto-reconnect
        // TW is a persistent condition, not a per-event error. Capture the raw bit here (this can run in
        // RX-callback context) BEFORE the clean-state early-return, so the falling edge is seen too, and
        // let process()->showThermalWarning() edge-log it in main-loop context.
        _twRaw = (state & TEMPERATURE_WARNING) != 0;
        _twSampled = true;

        if (state == U_STATE_IND) return;

        // printMessage("U_STATE_IND %02X", state);
        _uState |= (state ^ U_STATE_MASK) & ~TEMPERATURE_WARNING; // TW handled above; keep only transient errors
    }

    void DataLinkLayer::receivedReset()
    {
        // printMessage("U_RESET_IND");
        _lastValidRx = millis();
        _reconnectBackoff = 1000; // a real reconnect succeeded -> reset the poke backoff to fast
        _uReset = true;
        _modeExtendedCRC = false;
        _modeAutoAcknowlage = false;
        _receiver._invalid = false; // reset indication -> the stream restarts in sync, so clear the desync flag (the parser also clears it on the U_RESET_IND)
        _transmitter.reset();       // unsolicited chip reset -> abandon the stale in-flight TX (also clears _lastOffset)
        setBCUState(BCU_CONNECTED);
    }

    void DataLinkLayer::receivedConfiguration(char config)
    {
        _lastValidRx = millis();
        // printMessage("U_CONFIGURE_IND %02X", value);
        _modeAutoAcknowlage = config & AUTO_ACKNOWLEDGE;
        _modeExtendedCRC = config & CRC_CCITT;
    }

} // namespace TPUart
