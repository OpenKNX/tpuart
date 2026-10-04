#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "TPUart/Config.h"

namespace TPUart
{

    /**
     * @brief Byte ring of Capacity octets (one slot more is allocated to tell full from empty).
     * Defined in RingBuffer.cpp and instantiated there for the sizes in use.
     */
    template <size_t Capacity>
    class RingBufferN
    {
      public:
        RingBufferN();

        // Pushes one octet; false when full.
        bool push(char data);

        // Pops one octet into data; false when empty.
        bool pop(char &data);
        char pop();

        bool isEmpty() const;
        bool isFull() const;

        // Free slots.
        size_t available() const;
        size_t size() const;

        // Drops all octets.
        void clear();

      private:
        static constexpr size_t kSlots = Capacity + 1;

        // volatile: the ring is also written from the UART interrupt or task.
        volatile char buffer[kSlots];

        // head: next free slot, tail: next readable slot.
        volatile size_t head;
        volatile size_t tail;
    };

    // RX frame ring between the parser and process().
    using RingBuffer = RingBufferN<TPUART_RX_FRAME_BUFFER_SIZE>;
} // namespace TPUart
