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
        volatile char _cachedAcknowledge;
        size_t _transmitPos;
        unsigned char _lastOffset; // NCN5130 keeps the data-index offset until it is changed (DS p.42)
        volatile unsigned long _time;
        unsigned long _maxQueueSize;
        volatile TxState _state; // H1: written by the UART task (finalize), read by the main loop -> volatile (matches Receiver::_state)
        Frame *_frame = nullptr;
        uint32_t _lastBackstopReset = 0; // throttles repeated backstop fast-resets (a persistent echo-loss fault degrades to the 60s cap)

      public:
        std::queue<Frame *> _queue[4];
        size_t _txCount = 0;
        Transmitter(DataLinkLayer &dll);
        ~Transmitter();

        bool transmit(const char *data, size_t size);
        void finalize();
        void processWatchdog();

        bool pushQueue(Frame *frame);
        size_t queueSize();
        void reset();
        void sendAcknowledge(AcknowledgeType acknowledge = ACK_None);
        void setQueueSize(unsigned long size);

        void processTransmitByte();
        Frame *currentFrame();
        bool isTransmitting();
        bool awaitResponse();
        void sendCachedAcknowledge();
        void processQueue();
        void resetWatchdogTimer();
    };

} // namespace TPUart
