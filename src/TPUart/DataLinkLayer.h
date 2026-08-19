#pragma once
#include <Arduino.h>
#include <deque>
#include <functional>
#include <vector>

#include "TPUart/Frame.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Receiver.h"
#include "TPUart/RingBuffer.h"
#include "TPUart/SearchBuffer.h"
#include "TPUart/Statistics.h"
#include "TPUart/SystemState.h"
#include "TPUart/Transmitter.h"
#include "TPUart/Types.h"

#ifdef ARDUINO_ARCH_ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

#ifndef TPUART_MAX_PROCESS_TIME_PER_LOOP
#define TPUART_MAX_PROCESS_TIME_PER_LOOP 30
#endif

#ifndef TPUART_MAX_RXQUEUE_TIME_PER_LOOP
#define TPUART_MAX_RXQUEUE_TIME_PER_LOOP 20
#endif

namespace TPUart
{
    class DataLinkLayer
    {
        friend class Receiver;
        friend class Transmitter;

      private:
        bool _initialized = false;
        char _repetitions = 0b00110011; // 0-3 Nack (Default 3) // 5-7 Busy (Default 3)
        short _ownAddress = 0;
        volatile bool _uReset = false;
        volatile char _uState = 0;
        volatile bool _modeAutoAcknowlage = false;
        volatile bool _modeExtendedCRC = false;

        unsigned long _busyMode = 0;
        unsigned long _requestStateTimer = 0;
        unsigned long _lastTryInitialize = 0;
        unsigned long _lastReconnectAttempt = 0; // BCU auto-reconnect: throttle the U_RESET_REQ poke
        unsigned long _lastValidRx = 0;          // BCU auto-reconnect: last valid-protocol RX (liveness)
        unsigned long _reconnectBackoff = 1000;  // BCU auto-reconnect: poke interval, 1s -> 30s while NCN absent
        unsigned long _lastDiscardedMessage = 0;
        unsigned long _invalidSince = 0;    // desync watchdog: millis() of the rising edge of _receiver._invalid
        unsigned long _lastDesyncReset = 0; // desync watchdog: cooldown anchor, keeps a broken chip from reset-looping
        unsigned long _lastDiscardedBytes = 0;
        volatile size_t _rxFrameBufferEntries = 0;
        volatile BcuType _bcuType;
        // NCN5121/5130 boot register snapshot (read once in tryInitialize; static/latched values).
        uint8_t _ncnAsr0 = 0;      // ASR0 @ boot: rails + TW + TSD (thermal-shutdown history)
        uint8_t _ncnRevId = 0;     // RevID register: [7:5] silicon rev, [4:0] part number (0 = none / NCN5120)
        bool _ncnRegValid = false; // true once the boot snapshot was taken
        volatile BcuState _bcuState = BCU_UNINITIALIZED;
        volatile int _baudrate = 0; // last negotiated BCU baudrate (19200/38400), stored on connect

        // Overflow
        volatile bool _rxSearchBufferOverflow = false;
        volatile bool _rxFrameBufferOverflow = false;
        volatile bool _rxInterfaceOverflow = false;
        volatile unsigned long _rxShowOverflowTime = 0;

        // Transient U_State errors (SC/RE/TE/PE): every event is COUNTED (see showStateError), but the LOG
        // is rate-limited. Probing a non-existent individual address (`ftc scan` over empty PAs) gets no
        // L2-ACK -> the NCN reports a Protocol-Error on every un-acked TX -> one line per probe otherwise.
        char _stateErrLast = 0;               // last-logged error-bit set (a NEW set logs immediately)
        unsigned long _stateErrLogged = 0;    // millis() of the last printed "TP Error" line
        unsigned int _stateErrSuppressed = 0; // events collapsed since that line

        // NCN thermal-warning (TW): receivedState() (RX-callback context) only captures the raw bit;
        // process()->showThermalWarning() edge-logs it (L1) and optionally auto-heals a latch (L2).
        volatile bool _twRaw = false;     // latest TW bit from the 1Hz state poll
        volatile bool _twSampled = false; // a fresh poll sample is pending (drives the per-poll counter)
        bool _twActive = false;           // debounced: TW condition currently active
        unsigned long _twSince = 0;       // rising-edge time of the current TW episode
        unsigned long _twLastBeat = 0;    // last "still set" heartbeat while TW holds
#ifdef TPUART_NCN_TW_AUTORESET
        bool _twAutoResetDone = false;      // one auto-heal U_RESET per episode
        bool _twHotLogged = false;          // logged the "persists through reset -> genuinely hot" verdict
        unsigned long _twLastAutoReset = 0; // cross-episode auto-reset cooldown anchor
#endif

        Interface::Abstract *_interface = nullptr;

#if defined(ARDUINO_ARCH_RP2040)
        mutex_t _rxLock;
        mutex_t _txLock;
        mutex_t _test;
#elif defined(ARDUINO_ARCH_ESP32)
        SemaphoreHandle_t _rxLock;
        SemaphoreHandle_t _txLock;
#endif

        Transmitter _transmitter;
        Receiver _receiver;
        RingBuffer _rxFrameBuffer;
        RepetitionFilter _repetitionFilter;
        Statistics _statistics;
        SystemState _systemState;

        std::vector<std::function<void(Frame &)>> _callbacksReceivedFrame;
        std::function<AcknowledgeType(unsigned short, bool)> _callbackCheckAcknowledge;
        std::function<void(const char *, bool error)> _callbackMessage;

        void connected(bool connected);
        // len = 0 -> from frame.size(). A truncated frame must pass the octets actually received.
        void pushRxFrameBuffer(Frame &frame, unsigned short len = 0);
        // Push a standalone L2 acknowledge octet as a 1-byte busmon-only carrier (monitor mode only).
        void pushRxAcknByte(char value);
        // void processIncompleteFrame();
        // void processWaitForAcknTimer();
        void processRxFrameBuffer();
        AcknowledgeType checkAcknowledge(unsigned short destination, bool isGroupAddress);
        bool rxLock(bool blocking = false);
        bool txLock(bool blocking = false);
        void rxUnlock();
        void txUnlock();
        void showOverflowError();
        void showStateError();
        void showDiscardedError();
        void showSystemState();
        void showThermalWarning();
#ifdef TPUART_NCN_TW_AUTORESET
        void twAutoResetCheck(unsigned long now);
#endif
        void processRequestState();

        void tryInitialize();
        bool tryInitialize(uint baudrate);
        // Synchronous single-register read; ONLY safe inside the init busy-wait (no async/bus traffic).
        uint8_t readRegisterBlocking(char readOpcode);
        // One-shot NCN RevID + ASR0 snapshot, taken during init (NCN family only).
        void readNcnRegisterSnapshot();

        void exitBusyModeTimer();
        void setBCUState(BcuState state, int baudrate = 0);

        void receivedConfiguration(char config);
        void receivedState(char state);
        void receivedReset();
        void processWatchdog();

      public:
        DataLinkLayer();

        void begin(BcuType bcuType, Interface::Abstract *Interface);
        void end(bool deleteUart = true);
        void reset();
#ifdef TPUART_BCU_DEBUG
        void forceDisconnect(); // test hook (console "bcu dis"): force the terminal BCU_DISCONNECTED state
#endif
#ifdef TPUART_BCU_MARKER
        // Bench hook (console "bcu marker on|off"): NCN frame-end MARKER, DS p.40. The parser does not
        // know 0xCB/0x13, so the device stops delivering frames while on; "off" resets the BCU.
        bool markerMode(bool state);
#endif
        void process();

        void registerMessage(std::function<void(const char *, bool)> callback);
        void registerReceivedFrame(std::function<void(Frame &)> callback);
        void registerCheckAcknowledge(std::function<AcknowledgeType(unsigned short, bool)> callback);

        bool processReceviedByte();
        void processTransmitByte();

        void setRepetitions(uint8_t nack, uint8_t busy);
        void requestState();
        void applyConfiguration();
        void handleReset();
        void printMessage(const char *message, ...);
        void printError(const char *message, ...);
        bool pushTransmitQueue(Frame *frame);

        Statistics &getStatistics();
        SystemState &getSystemState();
        Receiver &getReceiver();
        Transmitter &getTransmitter();

        // NCN family identity + boot register snapshot (see tryInitialize).
        BcuType getBcuType();
        uint8_t getNcnAsr0();
        uint8_t getNcnRevId();
        bool ncnRegValid();

        bool powerControl(bool state);
        bool stopMode(bool state);
        bool busyMode(bool state);
        void setOwnAddress(short address);
        bool startMonitoring();
        BcuState getBcuState();
        const char *getBcuStateInfo();
        int getBaudrate();
        bool isMonitoring() const;
        bool isConnected() const;
        // "Is the KNX bus actually usable?" for the tunnel connectionstate heartbeat: host<->chip link up AND,
        // on an NCN, the bus-voltage (VBUS) bit set. isConnected() alone misses bus-power loss on an
        // externally-powered NCN (the chip keeps answering while the bus is dead).
        bool busOperational();

        volatile uint _statsDurationMax = 0;
        volatile uint _statsDurationMin = 0;
        volatile uint _statsDuration = 0;
        volatile uint _statsDurationCount = 0;
    };

} // namespace TPUart
