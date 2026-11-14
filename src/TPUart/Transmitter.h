#pragma once
#include "TPUart/Frame.h"
#include "TPUart/Types.h"
#include <cstdint>

namespace TPUart
{
    class DataLinkLayer;

    static_assert(TPUART_TX_QUEUE_SIZE < 255, "TPUART_TX_QUEUE_SIZE must fit TxQueue's 8-bit node indices");

    // One FIFO per priority rank (0..3) over a shared pool of TPUART_TX_QUEUE_SIZE nodes, without heap
    // (std::queue's deque allocates a 512-octet block per instance). pushQueue() admits a frame only while fewer
    // than TPUART_TX_QUEUE_SIZE are queued, so push() always finds a free node.
    class TxQueue
    {
        static constexpr uint8_t kNone = 0xFF;
        Frame *_frame[TPUART_TX_QUEUE_SIZE];
        uint8_t _next[TPUART_TX_QUEUE_SIZE];
        uint8_t _head[4] = {kNone, kNone, kNone, kNone};
        uint8_t _tail[4] = {kNone, kNone, kNone, kNone};
        uint8_t _free = 0;

      public:
        TxQueue()
        {
            for (uint8_t i = 0; i < TPUART_TX_QUEUE_SIZE; ++i) _next[i] = (i + 1 < TPUART_TX_QUEUE_SIZE) ? (uint8_t)(i + 1) : kNone;
        }
        bool empty(uint8_t rank) const { return _head[rank] == kNone; }
        Frame *front(uint8_t rank) const { return _frame[_head[rank]]; }
        void pop(uint8_t rank)
        {
            const uint8_t node = _head[rank];
            _head[rank] = _next[node];
            if (_head[rank] == kNone) _tail[rank] = kNone;
            _next[node] = _free;
            _free = node;
        }
        void push(uint8_t rank, Frame *frame)
        {
            const uint8_t node = _free;
            _free = _next[node];
            _frame[node] = frame;
            _next[node] = kNone;
            if (_tail[rank] == kNone)
                _head[rank] = node;
            else
                _next[_tail[rank]] = node;
            _tail[rank] = node;
        }
    };

    class Transmitter
    {
        DataLinkLayer &_dll;
        volatile unsigned int _droppedAcknowledges = 0; // acknowledges dropped because their window elapsed (see sendAcknowledge)
        // One word, because two fields cannot be read consistently from two contexts: bits 0-7 hold the
        // parked acknowledge (0 = none), bits 8-31 the micros() it was parked at, truncated to 24 bits.
        volatile unsigned long _pendingAcknowledge = 0;
        size_t _transmitPos;
        unsigned char _lastOffset; // NCN5130 keeps the data-index offset until it is changed (DS p.42)
        volatile unsigned long _time;
        volatile TxState _state; // written by the RX context (UART IRQ or UART task, finalize), read by the main loop -> volatile (matches Receiver::_state)
        Frame *_frame = nullptr;
        volatile bool _attemptEnded = false; // a frame start followed the echo of the frame in TX_AWAIT; RX context sets, TX_AWAIT entry clears
        uint32_t _lastBackstopReset = 0; // throttles repeated backstop resets (a persistent echo-loss fault falls back to TPUART_TX_WATCHDOG_MS)

      public:
        TxQueue _queue;
        volatile size_t _txCount = 0;
        Transmitter(DataLinkLayer &dll);
        ~Transmitter();

        bool transmit(const char *data, size_t size);
        void finalize();
        void processWatchdog();

        bool pushQueue(Frame *frame);
        size_t queueSize();
        // notify=false suppresses the dropped-frame callback (destructor only, see Transmitter::reset).
        void reset(bool notify = true);
        void sendAcknowledge(AcknowledgeType acknowledge = ACK_None);
        void flushAcknowledge();

        bool processTransmitByte();
        Frame *currentFrame();
        bool isTransmitting();
        bool awaitResponse();
        void markAttemptEnded() { _attemptEnded = true; }
        bool attemptEnded() { return _attemptEnded; }
        unsigned int droppedAcknowledges();
        void processQueue();
        void resetWatchdogTimer();
    };

} // namespace TPUart
