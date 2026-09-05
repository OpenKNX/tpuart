#pragma once
#include "TPUart/Frame.h"
#include "TPUart/Types.h"
#include <cstdint>
#include <queue>

namespace TPUart
{
    class DataLinkLayer;

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
        unsigned long _maxQueueSize;
        volatile TxState _state; // H1: written by the UART task (finalize), read by the main loop -> volatile (matches Receiver::_state)
        Frame *_frame = nullptr;
        uint32_t _lastBackstopReset = 0; // throttles repeated backstop fast-resets (a persistent echo-loss fault degrades to the 60s cap)

      public:
        std::queue<Frame *> _queue[4];
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
        void setQueueSize(unsigned long size);

        void processTransmitByte();
        Frame *currentFrame();
        bool isTransmitting();
        bool awaitResponse();
        unsigned int droppedAcknowledges();
        void processQueue();
        void resetWatchdogTimer();
    };

} // namespace TPUart
