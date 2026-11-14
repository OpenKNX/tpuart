#include "RepetitionFilter.h"
#include <Arduino.h>
#include <cstring>

namespace TPUart
{
    unsigned short RepetitionFilter::crc(Frame &frame)
    {
        // CRC-16/SPI-FUJITSU
        unsigned short crc = 0x1D0F;
        unsigned short polynomial = 0x1021;

        const size_t s = frame.size() - 1; // without checksum
        for (size_t i = 0; i < s; i++)
        {
            uint8_t data = (uint8_t)frame.data(i); // a signed char shifted left is undefined for octets >= 80h
            if (i == 0) data = data | 0b100000; // retry bit

            crc ^= (uint16_t)(data << 8);
            for (size_t j = 0; j < 8; j++)
                if (crc & 0x8000)
                    crc = (crc << 1) ^ polynomial;
                else
                    crc <<= 1;
        }
        return crc;
    }

    bool RepetitionFilter::check(Frame &frame)
    {
        const uint16_t source = frame.source();
        const uint16_t checksum = crc(frame);
        const unsigned long now = millis();

        std::size_t i = 0;
        while (i < _used && _entries[i].source != source) ++i;

        bool repetition = false;
        if (i < _used)
            repetition = _entries[i].checksum == checksum && (now - _entries[i].timestamp) <= TPUART_REPETITION_TTL_MS; // known source
        else if (_used < MAX_SIZE)
            ++_used; // new source, free slot: entry i (== old _used) is taken by the shift below
        else
            i = MAX_SIZE - 1; // new source, store full: the least recent entry is overwritten

        // Shift [0, i) one slot back and put the source in front, as the list did with erase + push_front.
        memmove(&_entries[1], &_entries[0], i * sizeof(CacheEntry));
        _entries[0] = {source, checksum, now};
        return repetition;
    }

    void RepetitionFilter::clear()
    {
        _used = 0;
    }

    std::size_t RepetitionFilter::size()
    {
        return _used;
    }

} // namespace TPUart