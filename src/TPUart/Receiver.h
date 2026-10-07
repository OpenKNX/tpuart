#pragma once
#include "TPUart/Frame.h"
#include "TPUart/RepetitionFilter.h"
#include "TPUart/RingBuffer.h"
#include "TPUart/SearchBuffer.h"
#include "TPUart/Types.h"
#include <Arduino.h>

namespace TPUart
{
    class DataLinkLayer;

    class Receiver
    {
        // bool _frame = false;

        friend class DataLinkLayer;
        friend class Transmitter;

        DataLinkLayer &_dll;
        SearchBuffer _searchBuffer;
        RingBufferN<TPUART_DISCARD_LOG_SIZE> _discardedBytes; // hex-dump source of showDiscardedError(), first octets kept
        volatile unsigned long _lastDiscarded = 0;

        volatile RxState _state = RX_IDLE;
        // volatile bool _uReset = false;
        // volatile char _uState = 0;

        volatile size_t _awaitBytes = 1;
        volatile unsigned long _lastReceivedTime = 0;
        volatile uint8_t _unansweredProbes = 0; // U_STATE_REQ written since the last octet read from the chip

        void processSearchBufferFrame();
        void processSearchBufferInvalid(int x);
        void processSearchBufferAcknowledge();
        void processTimeout();
        void processCompleteFrame(bool acknowledge = false);
        void processSearchBufferTimeout();

        bool sufficientlyBytes();
        bool pushSearchBuffer(const char value);
        void processControlBytes();

      public:
        volatile bool _invalid = false;
        // Set when any octet of the frame currently being assembled arrived flagged by the chip; applied
        // to the frame when it is handed up, cleared with it.
        volatile bool _byteErrorInFrame = false;
        bool test = false;
        Receiver(DataLinkLayer &dll);

        void process();
        bool processReceviedByte();
        void processSearchBuffer();
        void reset();

        unsigned short getAwaitBytes();
        unsigned short getSearchBufferPosition();
    };
} // namespace TPUart