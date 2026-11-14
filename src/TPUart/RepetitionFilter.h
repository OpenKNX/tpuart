#include "TPUart/Config.h"
#include "TPUart/Frame.h"
#include <Arduino.h>
#include <cstdint>
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

#ifdef TPUART_REPETITION_SIMPLE
        // fast lookup: Source -> Checksum
        std::unordered_map<uint16_t, RepetitionFilterEntry> _entries;
#else
        static constexpr std::size_t MAX_SIZE = TPUART_REPETITION_FILTER_SIZE;
        // 03_02_02 2.4.2 p.39 suppresses only a repetition of the directly preceding frame, so an entry ages out after
        // TPUART_REPETITION_TTL_MS; the longest repetition burst of a maximum extended frame is about 2.6 s.
        struct CacheEntry
        {
            uint16_t source;
            uint16_t checksum;
            unsigned long timestamp;
        };
        // Fixed store without heap, ordered by last access: [0] is the most recent, [_used - 1] the least recent.
        CacheEntry _entries[MAX_SIZE];
        std::size_t _used = 0;
#endif
    };
} // namespace TPUart