#include "TPUart/RepetitionFilter.h"

#include "TPUart/Frame.h"

namespace TPUart
{

uint16_t RepetitionFilter::fingerprint(const Frame &frame)
{
    uint16_t crc = 0x1D0F;
    constexpr uint16_t POLYNOMIAL = 0x1021;

    // Nur über das, was wirklich ankam: bei einem abgeschnittenen Telegramm ist size() größer als die
    // vorhandenen Daten. Das Prüfsummen-Oktett bleibt außen vor.
    size_t length = frame.length();
    if (frame.size() <= length) length = frame.size() - 1;

    for (size_t i = 0; i < length; i++)
    {
        uint8_t value = frame.data(i);
        if (i == 0) value |= 0b100000; // Wiederholungs-Bit vereinheitlichen

        crc ^= (uint16_t)(value << 8);
        for (size_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ POLYNOMIAL) : (uint16_t)(crc << 1);
    }

    return crc;
}

bool RepetitionFilter::check(const Frame &frame)
{
    uint16_t source = frame.source();

    // 0.0.0 kennzeichnet einen freien Platz - ein solcher Absender wird nicht gemerkt, also nie gefiltert.
    if (source == 0) return false;

    uint16_t checksum = fingerprint(frame);
    _counter = (uint16_t)(_counter + 1);

    Entry *slot = nullptr;
    Entry *oldest = nullptr;
    bool seen = false;

    for (size_t i = 0; i < TPUART_REPETITION_FILTER_COUNT; i++)
    {
        Entry &entry = _entries[i];

        if (entry._source == 0)
        {
            if (slot == nullptr) slot = &entry;
            continue;
        }

        if (entry._source == source)
        {
            seen = (entry._checksum == checksum);
            slot = &entry; // derselbe Absender überschreibt immer seinen eigenen Eintrag
            break;
        }

        if (oldest == nullptr || (uint16_t)(_counter - entry._seen) > (uint16_t)(_counter - oldest->_seen)) oldest = &entry;
    }

    // Alles belegt und der Absender noch nicht dabei: der am längsten nicht mehr gesehene Absender
    // fällt raus. Das ist die einzige Art, wie ein Eintrag verschwindet - neue Telegramme verdrängen
    // alte, so wie die 50er-Liste der alten Library.
    if (slot == nullptr) slot = oldest;

    slot->_source = source;
    slot->_checksum = checksum;
    slot->_seen = _counter;

    return seen;
}

void RepetitionFilter::clear()
{
    for (size_t i = 0; i < TPUART_REPETITION_FILTER_COUNT; i++)
        _entries[i]._source = 0;
}

size_t RepetitionFilter::size() const
{
    size_t count = 0;
    for (size_t i = 0; i < TPUART_REPETITION_FILTER_COUNT; i++)
        if (_entries[i]._source != 0) count++;

    return count;
}

} // namespace TPUart
