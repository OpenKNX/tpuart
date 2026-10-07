#pragma once
#include <cstddef>
#include <cstdint>
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
            /** @brief Writes a block no acknowledge may split (a frame octet pair, a multi-octet request). */
            virtual bool writeGlued(const char *data, size_t length)
            {
                for (size_t i = 0; i < length; i++)
                    if (!write(data[i])) return false;
                return true;
            }
            /** @brief Sends an acknowledge ahead of queued octets; false = not supported, the caller writes it itself. */
            virtual bool writeUrgent(char) { return false; }
            /** @brief Octets writable without waiting; SIZE_MAX for an interface whose write() waits itself. */
            virtual size_t txFree() { return SIZE_MAX; }
            /** @brief Drops queued, unsent octets. */
            virtual void txClear() {}
            /** @brief Acknowledges writeUrgent() dropped because their window elapsed. */
            virtual unsigned int droppedUrgent() { return 0; }
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