#include "TPUart/Frame.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace TPUart
{

// Der Inhalt bleibt bewusst uninitialisiert: er wird gleich vollständig überschrieben, und 263 Byte
// vorsorglich zu nullen wäre in einem Pfad, der pro Telegramm läuft, verschenkte Zeit.
Frame::Frame(size_t length, uint8_t flags) : _length(length < TPUART_BUFFER_SIZE ? length : TPUART_BUFFER_SIZE), _flags(flags) {}

Frame::Frame(const uint8_t *data, size_t length, uint8_t flags) : Frame(length, flags)
{
    memcpy(_data, data, _length);
}

Frame::Frame(const char *data, size_t length) : Frame((const uint8_t *)data, length, 0) {}

uint8_t Frame::at(size_t pos) const
{
    return pos < _length ? _data[pos] : 0;
}

const char *Frame::data() const
{
    return (const char *)_data;
}

uint8_t Frame::data(size_t pos) const
{
    return at(pos);
}

uint8_t *Frame::buffer()
{
    return _data;
}

size_t Frame::length() const
{
    return _length;
}

uint8_t Frame::flags() const
{
    return _flags;
}

void Frame::addFlags(uint8_t flags)
{
    _flags |= flags;
}

void Frame::resetFlags()
{
    _flags = 0;
}

bool Frame::isExtended() const
{
    return (at(0) & L_DATA_MASK) == L_DATA_EXTENDED_IND;
}

bool Frame::isFrame() const
{
    return (at(0) & L_DATA_MASK) == L_DATA_STANDARD_IND || (at(0) & L_DATA_MASK) == L_DATA_EXTENDED_IND;
}

// Die 9 gegen 8 steht NUR HIER - sowohl sizeOf() als auch metadataSize() holen sie sich von hier, sonst
// gäbe es die Fallunterscheidung wieder zweimal.
static uint8_t metadataSizeOf(uint8_t control)
{
    return (control & L_DATA_MASK) == L_DATA_EXTENDED_IND ? 9 : 8;
}

uint8_t Frame::metadataSize() const
{
    return metadataSizeOf(at(0));
}

// Abgeleitet statt erneut aus dem Längenfeld gelesen: die Nutzlast ist genau das, was von der Gesamtlänge
// nach dem Rahmen übrig bleibt. So gibt es auch für die Herkunft der Länge nur eine Stelle.
uint8_t Frame::apduSize() const
{
    return (uint8_t)(size() - metadataSize());
}

// Die Zahlen: ein Standard-Telegramm hat 8 Oktetts Rahmen (Steuerbyte, Quelle 2, Ziel 2,
// Adresstyp/Hopcount/Länge, TPCI, Prüfsumme) plus die APDU aus dem unteren Nibble von Byte 5; ein Extended
// hat 9 - es bringt ein eigenes Längen-Oktett mit - plus die APDU aus Byte 6.
uint16_t Frame::sizeOf(const uint8_t *data, size_t available)
{
    if (available < 6) return 0; // ohne Byte 5 ist nicht einmal die Standard-Länge lesbar

    uint8_t metadata = metadataSizeOf(data[0]);

    if (metadata == 9)
    {
        if (available < 7) return 0; // das Längen-Oktett des Extended steht erst in Byte 6
        return (uint16_t)(metadata + data[6]);
    }

    return (uint16_t)(metadata + (data[5] & 0x0F));
}

// DER UMWEG ÜBER DAS KOPF-ARRAY ist kein Zierrat, sondern der Unterschied zwischen zwei Verträgen: at()
// liefert jenseits von length() eine 0, size() beantwortet damit "wie lang SOLLTE es sein" auch bei einem
// Fragment. sizeOf() meldet dort stattdessen 0 ("noch nicht entscheidbar"), und das wäre hier gefährlich -
// cemiSize() rechnet size() - 1, und mit einer 0 entstünde ein 1-Byte-malloc, in das cemiData() dann
// hineinschriebe. Sieben Bytes auf dem Stack, und size() läuft nicht je empfangenem Byte.
uint16_t Frame::size() const
{
    uint8_t header[7] = {at(0), at(1), at(2), at(3), at(4), at(5), at(6)};

    return sizeOf(header, sizeof(header));
}

uint16_t Frame::source() const
{
    return isExtended() ? (uint16_t)((at(2) << 8) + at(3)) : (uint16_t)((at(1) << 8) + at(2));
}

uint16_t Frame::destination() const
{
    return isExtended() ? (uint16_t)((at(4) << 8) + at(5)) : (uint16_t)((at(3) << 8) + at(4));
}

bool Frame::isGroupAddress() const
{
    return isExtended() ? (at(1) >> 7) & 0b1 : (at(5) >> 7) & 0b1;
}

bool Frame::isRepeated() const
{
    return !(at(0) & 0b100000);
}

uint8_t Frame::calcCRC8() const
{
    uint16_t last = size() > 0 ? (uint16_t)(size() - 1) : 0;
    if (last > _length) last = (uint16_t)_length; // abgeschnitten - nicht über das Empfangene hinaus lesen

    uint8_t checksum = 0;
    for (uint16_t i = 0; i < last; i++)
        checksum ^= _data[i];

    return (uint8_t)~checksum;
}

bool Frame::isValid() const
{
    // DAS URTEIL DES EMPFÄNGERS ZUERST. Er weiß beim Setzen des Flags Dinge, die den ausgelieferten Bytes
    // nicht mehr anzusehen sind - etwa dass eine verifizierte Pause mitten in das Telegramm fiel. Es zu
    // übergehen hieße, ein bekannt kaputtes Telegramm womöglich für gut zu erklären. Und es steht vorn,
    // weil ein so markiertes Telegramm dann nicht einmal den Durchlauf kostet.
    if (_flags & TP_FRAME_FLAG_INVALID) return false;

    // Steuerbyte. Alles, was kein L_Data ist, hat auf dem Sendeweg nichts verloren - insbesondere kein
    // Poll (0xF0), dessen Länge ganz anders zustande kommt.
    if (!isFrame()) return false;

    // Länge. size() leitet sie aus dem Kopf ab; stimmt sie nicht überein, ist das Telegramm abgeschnitten
    // oder das Längenfeld verfälscht. Die Mindestlänge ist damit miterledigt: ein Fragment unter sechs
    // Bytes kann die abgeleitete Größe gar nicht erreichen.
    if (_length != size()) return false;

    return calcCRC8() == _data[_length - 1];
}

bool Frame::isInvalid() const
{
    return !isValid();
}

bool Frame::isFiltered() const
{
    return (_flags & TP_FRAME_FLAG_FILTERED) != 0;
}

void Frame::setFiltered()
{
    _flags |= TP_FRAME_FLAG_FILTERED;
}

bool Frame::isTransmitted() const
{
    return (_flags & TP_FRAME_FLAG_TX) != 0;
}

void Frame::setTransmitted()
{
    _flags |= TP_FRAME_FLAG_TX;
}

bool Frame::isDataCon() const
{
    return (_flags & TP_FRAME_FLAG_DATA_CON) != 0;
}

bool Frame::isAddressed() const
{
    return (_flags & TP_FRAME_FLAG_ADDRESSED) != 0;
}

bool Frame::isAck() const
{
    return (_flags & TP_FRAME_FLAG_ACK) != 0;
}

bool Frame::isNack() const
{
    return (_flags & TP_FRAME_FLAG_ACK_NACK) != 0;
}

bool Frame::isBusy() const
{
    return (_flags & TP_FRAME_FLAG_ACK_BUSY) != 0;
}

void Frame::setAcknowledge(AckType acknowledge)
{
    _flags &= ~(TP_FRAME_FLAG_ADDRESSED | TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_BUSY | TP_FRAME_FLAG_ACK_NACK);

    if (acknowledge == AckType::None) return;

    // ADDRESSED kommt hier dazu, und nur hier: diese Überladung ist ausdrücklich die Quittung, die WIR
    // gesendet haben. Die Bits selbst liefert acknowledgeFlags() - dieselbe Übersetzung wie im Receiver.
    _flags |= (uint8_t)(TP_FRAME_FLAG_ADDRESSED | acknowledgeFlags(acknowledge));
}

void Frame::setAcknowledge(bool busy, bool nack)
{
    _flags |= TP_FRAME_FLAG_ACK;
    if (busy) _flags |= TP_FRAME_FLAG_ACK_BUSY;
    if (nack) _flags |= TP_FRAME_FLAG_ACK_NACK;
}

// Übernommen aus der alten Library, Rechnung unverändert: +2 Bytes Kopf (0x29, 0x00), beim
// Standard-Telegramm zusätzlich +1, weil Länge und Hop-Count dort auf zwei Bytes auseinandergezogen
// werden; -1 für die entfallende Prüfsumme.
uint16_t Frame::cemiSize() const
{
    return (uint16_t)(size() + (isExtended() ? 2 : 3) - 1);
}

uint8_t *Frame::cemiData() const
{
    uint8_t *buffer = (uint8_t *)malloc(cemiSize());
    if (buffer == nullptr) return nullptr;

    // NUR SO WEIT KOPIEREN, WIE WIRKLICH EMPFANGEN WURDE. size() kommt aus dem Längenbyte, und das kann
    // bei einem abgeschnittenen oder verfälschten Telegramm weit über _length liegen - dann stünde im
    // cEMI-Puffer der uninitialisierte Rest von _data. Über den Puffer hinaus ginge es nicht (die
    // Rechnung in cemiSize() passt), aber der Aufrufer bekäme Datenmüll als Nutzlast geliefert. Der Rest
    // bleibt genullt, damit die Länge stimmt und der Inhalt wenigstens definiert ist.
    //
    // Wer ein Telegramm weiterverarbeitet, ohne isValid() zu prüfen, bekommt hier also Nullen statt
    // Zufall - die Prüfung ersetzt das nicht.
    memset(buffer, 0, cemiSize());

    buffer[0] = 0x29; // L_Data.ind
    buffer[1] = 0x00; // keine Zusatzinformation

    if (isExtended())
    {
        size_t count = size() - 1; // -1: ohne Prüfsumme
        if (count > _length) count = _length;

        memcpy(buffer + 2, _data, count);
    }
    else
    {
        buffer[2] = at(0);
        buffer[3] = (uint8_t)(at(5) & 0xF0); // Kontrollfeld 2: Adresstyp und Hop-Count
        buffer[8] = (uint8_t)(at(5) & 0x0F); // Länge, im Standard-Frame nur ein Nibble

        size_t count = _length > 1 ? _length - 1 : 0;
        memcpy(buffer + 4, _data + 1, count < 4 ? count : 4); // Quelle und Ziel

        count = _length > 6 ? _length - 6 : 0;
        size_t apdu = (size_t)buffer[8] + 1;
        memcpy(buffer + 9, _data + 6, count < apdu ? count : apdu);
    }

    return buffer;
}

std::string Frame::humanSource() const
{
    uint16_t value = source();
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%02i.%02i.%03i", (value >> 12 & 0b1111), (value >> 8 & 0b1111), (value & 0b11111111));
    return buffer;
}

std::string Frame::humanDestination() const
{
    uint16_t value = destination();
    char buffer[16];

    if (isGroupAddress())
        snprintf(buffer, sizeof(buffer), "%02i/%02i/%03i", (value >> 11 & 0b11111), (value >> 8 & 0b111), (value & 0b11111111));
    else
        snprintf(buffer, sizeof(buffer), "%02i.%02i.%03i", (value >> 12 & 0b1111), (value >> 8 & 0b1111), (value & 0b11111111));

    return buffer;
}

std::string Frame::printFrame() const
{
    std::string result;
    result.reserve(48 + _length * 3);

    result.append(humanSource());
    result.append(" -> ");
    result.append(humanDestination());
    result.append(" [");
    // Die Reihenfolge der Buchstaben ist reine Darstellung und hat nichts mit der Bitbelegung zu tun.
    // 'D' ist isAddressed(), nicht "von uns gesendet" - das ist das 'T' darüber.
    // DATA_CON hat in dieser Ausgabe bewusst keinen Buchstaben.
    result.push_back(isTransmitted() ? 'T' : '_'); // von uns gesendet
    result.push_back(isAddressed() ? 'D' : '_');   // für uns bestimmt (und deshalb von uns quittiert)
    result.push_back(isInvalid() ? 'I' : '_');     // kaputt
    result.push_back(isExtended() ? 'E' : '_');    // Extended
    result.push_back(isRepeated() ? 'R' : '_');    // Wiederholung
    result.push_back(isFiltered() ? 'F' : '_');    // gefiltert
    result.push_back(isBusy() ? 'B' : '_');        // Quittung BUSY
    result.push_back(isNack() ? 'N' : '_');        // Quittung NACK
    result.push_back(isAck() ? 'A' : '_');         // Quittung liegt vor
    result.append("] ( ");

    // Ausgegeben wird, was WIRKLICH ankam - bei einem abgeschnittenen Frame weniger als size().
    char hex[4];
    for (size_t i = 0; i < _length; i++)
    {
        if (i) result.push_back(' ');
        snprintf(hex, sizeof(hex), "%02X", _data[i]);
        result.append(hex);
    }

    result.append(" ) [");
    // Nur die tatsächlich empfangene Länge. Ein abgeschnittenes Telegramm hatte hier einmal zusätzlich die
    // Soll-Länge stehen - die Unterscheidung "abgeschnitten" gegenüber "Prüfsumme falsch" ist aber keine,
    // auf die jemand anders reagiert: kaputt ist kaputt, und beide tragen INVALID.
    result.append(std::to_string(_length));
    result.append("]");

    return result;
}

} // namespace TPUart
