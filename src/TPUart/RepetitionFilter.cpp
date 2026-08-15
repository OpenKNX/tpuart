#include "TPUart/RepetitionFilter.h"

#include <Arduino.h>

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
    uint16_t checksum = fingerprint(frame);
    uint32_t now = millis();

    Entry *slot = nullptr;
    Entry *oldest = &_entries[0];
    bool seen = false;

    for (size_t i = 0; i < TPUART_REPETITION_FILTER_COUNT; i++)
    {
        Entry &entry = _entries[i];

        if (!entry._used)
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

        if ((uint32_t)(now - entry._timestamp) > (uint32_t)(now - oldest->_timestamp)) oldest = &entry;
    }

    // Alles belegt und der Absender noch nicht dabei: der am längsten nicht mehr gesehene Absender
    // fällt raus. Das ist die einzige Art, wie ein Eintrag verschwindet - neue Telegramme verdrängen
    // alte, so wie die 50er-Liste der alten Library.
    if (slot == nullptr) slot = oldest;

    slot->_source = source;
    slot->_checksum = checksum;
    slot->_timestamp = now;
    slot->_used = true;

    return seen;
}

void RepetitionFilter::clear()
{
    for (size_t i = 0; i < TPUART_REPETITION_FILTER_COUNT; i++)
        _entries[i]._used = false;
}

size_t RepetitionFilter::size() const
{
    size_t count = 0;
    for (size_t i = 0; i < TPUART_REPETITION_FILTER_COUNT; i++)
        if (_entries[i]._used) count++;

    return count;
}

} // namespace TPUart
