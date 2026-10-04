#pragma GCC optimize("O3")
#include "TPUart/RingBuffer.h"

namespace TPUart
{

    template <size_t Capacity>
    RingBufferN<Capacity>::RingBufferN() : head(0), tail(0) {}

    // Full when the slot after head is tail.
    template <size_t Capacity>
    bool RingBufferN<Capacity>::push(char data)
    {
        size_t next = (head + 1) % kSlots;
        if (next == tail)
        {
            // Full: only kSlots - 1 elements can be stored
            return false;
        }
        buffer[head] = data;

        asm volatile("" ::: "memory");
        head = next;
        return true;
    }

    // Empty when head == tail.
    template <size_t Capacity>
    bool RingBufferN<Capacity>::pop(char &data)
    {
        if (isEmpty())
        {
            return false;
        }
        data = buffer[tail];
        asm volatile("" ::: "memory");
        tail = (tail + 1) % kSlots;
        return true;
    }

    template <size_t Capacity>
    char RingBufferN<Capacity>::pop()
    {
        if (isEmpty())
        {
#if defined(__cpp_exceptions) || defined(__EXCEPTIONS)
            throw std::out_of_range("Buffer is empty");
#endif
            return 0;
        }
        char value = buffer[tail];
        asm volatile("" ::: "memory");
        tail = (tail + 1) % kSlots;
        return value;
    }

    template <size_t Capacity>
    bool RingBufferN<Capacity>::isEmpty() const
    {
        return head == tail;
    }

    template <size_t Capacity>
    bool RingBufferN<Capacity>::isFull() const
    {
        return ((head + 1) % kSlots) == tail;
    }

    // Free slots; one slot is sacrificed, so at most kSlots - 1 elements fit.
    template <size_t Capacity>
    size_t RingBufferN<Capacity>::available() const
    {
        if (head >= tail)
            return kSlots - (head - tail) - 1;
        else
            return tail - head - 1;
    }

    // Octets stored.
    template <size_t Capacity>
    size_t RingBufferN<Capacity>::size() const
    {
        return kSlots - available() - 1;
    }

    template <size_t Capacity>
    void RingBufferN<Capacity>::clear()
    {
        head = 0;
        tail = 0;
    }

    template class RingBufferN<TPUART_RX_FRAME_BUFFER_SIZE>;

} // namespace TPUart
