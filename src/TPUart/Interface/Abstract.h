#pragma once
#include <functional>

namespace TPUart
{
    namespace Interface
    {
        class Abstract
        {
          protected:
            bool _running = false;

          public:
            virtual void flush() = 0;
            virtual void begin(int baud) = 0;
            virtual void end() = 0;
            virtual bool available() = 0;
            virtual bool availableForWrite() = 0;
            virtual bool write(char value) = 0;
            virtual int read() = 0;
            virtual bool overflow() { return false; };
            /*
             * Did the octet just handed out by read() arrive flagged as defective? The NCN encodes a
             * bus-side bit error into its parity bit (9-bit UART mode, DS p.27) -- the only per-octet
             * error channel that survives bus monitor mode. Valid until the next read(); adapters
             * without access return false and may report coarser than per-octet.
             */
            virtual bool lastByteErrored() { return false; }
            virtual bool lastByteOverrun() { return false; }
            virtual unsigned char lastByteStatus() { return 0; } // bit0 FE, bit1 PE, bit2 BE, bit3 OE
            virtual bool hasCallback() { return false; }
            virtual void registerCallback(std::function<bool()> callback) {}
        };
    } // namespace Interface
} // namespace TPUart