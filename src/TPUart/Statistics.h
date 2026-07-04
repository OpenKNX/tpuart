#pragma once

namespace TPUart
{
    class DataLinkLayer;

    class Statistics
    {
      private:
        volatile unsigned int _rxControlBytes;
        volatile unsigned int _rxDiscardedBytes;
        volatile unsigned int _rxFrameBufferOverflow;
        volatile unsigned int _rxFrames;
        volatile unsigned int _rxFrameBytes;
        volatile unsigned int _rxLastBusBytes;
        volatile unsigned long _rxLastBusLoadTime;
        volatile unsigned int _rxOverflowFrameBuffer;
        volatile unsigned int _rxOverflowInterface;
        volatile unsigned int _rxOverflowSearchBuffer;
        volatile unsigned int _rxReceivedBytes;
        volatile unsigned int _rxSearchBufferOverflow;
        volatile unsigned int _rxUartOverflow;
        volatile unsigned int _txFrames;
        volatile unsigned int _txOverflowFrameBuffer;
        volatile unsigned int _rxRepetitions;
        // BCU health counters — persist across BCU resets, cleared only on reboot. Deliberately NOT
        // zeroed by reset(): a watchdog "Reset BCU" calls _statistics.reset(), which would otherwise
        // wipe the very count it just incremented (and the CON-rescue/disconnect history with it).
        // In-class init gives them a defined zero at boot; reset() leaves them alone.
#ifdef TPUART_BCU_HEALTH
        volatile unsigned int _bcuResets = 0;
        volatile unsigned int _bcuDisconnects = 0;
        volatile unsigned int _bcuConRescues = 0;
        volatile unsigned int _bcuTempWarnings = 0; // NCN TEMPERATURE_WARNING (TW) count -- thermal health signal
        // NCN U_State error-bit counts (lifetime, persist across reset like the others) -- quantify bus/NCN health
        volatile unsigned int _bcuSlaveCollisions = 0; // SC
        volatile unsigned int _bcuReceiveErrors = 0;   // RE
        volatile unsigned int _bcuTransmitErrors = 0;  // TE
        volatile unsigned int _bcuProtocolErrors = 0;  // PE
#endif

      public:
        Statistics();

        void reset();

        void incrementRxOverflowFrameBuffer(int increment = 1);
        void incrementRxOverflowInterface(int increment = 1);
        void incrementRxOverflowSearchBuffer(int increment = 1);
        void incrementRxRepetitions(int increment = 1);
        void incrementRxFrames(int increment = 1);
        void incrementRxFrameBytes(int increment = 1);
        // void incrementRxControlBytes(int increment = 1);
        void incrementRxDiscardedBytes(int increment = 1);
        void incrementRxReceivedBytes(int increment = 1);
        void incrementRxSearchBufferOverflow(int increment = 1);
        void incrementRxFrameBufferOverflow(int increment = 1);
        void incrementRxUartOverflow(int increment = 1);
        void incrementTxOverflowFrameBuffer(int increment = 1);
        void incrementTxFrames(int increment = 1);
#ifdef TPUART_BCU_HEALTH
        void incrementBcuResets(int increment = 1);
        void incrementBcuDisconnects(int increment = 1);
        void incrementBcuConRescues(int increment = 1);
        void incrementBcuTempWarnings(int increment = 1);
        void incrementBcuSlaveCollisions(int increment = 1);
        void incrementBcuReceiveErrors(int increment = 1);
        void incrementBcuTransmitErrors(int increment = 1);
        void incrementBcuProtocolErrors(int increment = 1);
#else
        // TPUART_BCU_HEALTH off: no-op inlines -> call sites stay clean and compile to nothing
        void incrementBcuResets(int = 1) {}
        void incrementBcuDisconnects(int = 1) {}
        void incrementBcuConRescues(int = 1) {}
        void incrementBcuTempWarnings(int = 1) {}
        void incrementBcuSlaveCollisions(int = 1) {}
        void incrementBcuReceiveErrors(int = 1) {}
        void incrementBcuTransmitErrors(int = 1) {}
        void incrementBcuProtocolErrors(int = 1) {}
#endif

        unsigned int getRxRepetitions();
        unsigned int getRxFrameBytes();
        unsigned int getTxFrames();
        unsigned int getRxFrames();
        unsigned int getRxBusBytes();
        // unsigned int getRxControlBytes();
        unsigned int getRxDiscardedBytes();
        unsigned int getRxReceivedBytes();
        unsigned int getRxSearchBufferOverflow();
        unsigned int getRxFrameBufferOverflow();
        unsigned int getRxUartOverflow();
        unsigned int getBusLoad();
        unsigned int getTxOverflowFrameBuffer();
        unsigned int getRxOverflowFrameBuffer();
        unsigned int getRxOverflowInterface();
        unsigned int getRxOverflowSearchBuffer();
#ifdef TPUART_BCU_HEALTH
        unsigned int getBcuResets();
        unsigned int getBcuDisconnects();
        unsigned int getBcuConRescues();
        unsigned int getBcuTempWarnings();
        unsigned int getBcuSlaveCollisions();
        unsigned int getBcuReceiveErrors();
        unsigned int getBcuTransmitErrors();
        unsigned int getBcuProtocolErrors();
#else
        // TPUART_BCU_HEALTH off: return 0 so any API consumer still compiles/links
        unsigned int getBcuResets() { return 0; }
        unsigned int getBcuDisconnects() { return 0; }
        unsigned int getBcuConRescues() { return 0; }
        unsigned int getBcuTempWarnings() { return 0; }
        unsigned int getBcuSlaveCollisions() { return 0; }
        unsigned int getBcuReceiveErrors() { return 0; }
        unsigned int getBcuTransmitErrors() { return 0; }
        unsigned int getBcuProtocolErrors() { return 0; }
#endif
    };

} // namespace TPUart
