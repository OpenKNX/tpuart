#include "TPUart/Frame.h"
#include <Arduino.h>
#include <cstdint>
#include <list>
#include <unordered_map>

// #define TPUART_REPETITION_SIMPLE

namespace TPUart
{
    struct RepetitionFilterEntry
    {
        unsigned short checksum;
        unsigned long timestamp;
    };

    class RepetitionFilter
    {
      public:
        bool check(Frame &frame);
        void clear();
        std::size_t size();

      private:
        unsigned short crc(Frame &frame);
        unsigned long _avg = 0;
        unsigned long _avgCnt = 0;

#ifdef TPUART_REPETITION_SIMPLE
        // fast lookup: Source -> Checksum
        std::unordered_map<uint16_t, RepetitionFilterEntry> _entries;
#else
        static constexpr std::size_t MAX_SIZE = 50;
        // 03_02_02 2.4.2 p.39 suppresses only a repetition of the directly preceding frame, so an entry ages
        // out: without a bound, a repetition whose original this device missed was dropped forever whenever
        // that source's last frame carried the same payload -- the normal state of a cyclic sensor. The
        // worst-case legitimate repetition burst of a maximum extended frame is about 2.6 s.
        struct CacheEntry
        {
            uint16_t source;
            uint16_t checksum;
            unsigned long timestamp;
        };
        std::list<CacheEntry> _entries;
        std::unordered_map<uint16_t, std::list<CacheEntry>::iterator> _map;
#endif
    };
} // namespace TPUart