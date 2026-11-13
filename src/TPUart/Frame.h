#pragma once
#include <stddef.h>
#include <stdint.h>

#include <string>

#include "TPUart/Types.h"

namespace TPUart
{

// Ein Telegramm - API des übrigen KNX-Stacks, Namen und Semantik wie in der alten Library. Abweichungen:
//  - besitzt seine Daten in einem festen Array (kein Heap, gefahrlos kopierbar), gedacht für den Stack
//  - uint8_t statt char
//  - length() ist die empfangene Länge, size() die laut Kopf; alle Zugriffe sind gegen length() abgesichert
//  - isValid() nutzt das INVALID-Flag des Receivers und prüft zusätzlich selbst
//  - ohne checkCRC16*, awaitDestination()/awaitSize()
class Frame
{
  private:
    uint8_t _data[TPUART_BUFFER_SIZE];
    size_t _length; // tatsächlich empfangene Bytes - kann kleiner als size() sein
    uint8_t _flags;

    // Grenzgeprüfter Zugriff - ein abgeschnittenes Frame liest nicht über length() hinaus.
    uint8_t at(size_t pos) const;

  public:
    // Frame der gewünschten Länge (gekappt auf TPUART_BUFFER_SIZE); befüllt über buffer().
    explicit Frame(size_t length, uint8_t flags = 0);

    // Kopiert die Daten.
    Frame(const uint8_t *data, size_t length, uint8_t flags = 0);

    // KOMPAT: Signatur der alten Library, kopiert ebenfalls.
    Frame(const char *data, size_t length);

    // `const char *` ist Pflicht: OFM-Network schreibt `const char *d = frame.data()`. Deshalb heißt der
    // Schreibzugriff buffer().
    const char *data() const;

    uint8_t data(size_t pos) const;

    // Schreibzugriff, genau length() Bytes.
    uint8_t *buffer();

    // Tatsächlich empfangene Bytes - nicht size().
    size_t length() const;

    uint8_t flags() const;
    void addFlags(uint8_t flags);
    void resetFlags();

    bool isExtended() const;
    bool isFrame() const;

    // Metadaten am Anfang des Frames plus die Prüfsumme am Ende.
    uint8_t metadataSize() const;

    uint8_t apduSize() const;

    // Länge laut Kopf. Wird nie 0 - Aufrufer rechnen size() - 1.
    uint16_t size() const;

    // Dieselbe Ableitung auf rohen Bytes (Receiver im Tick, Sendewarteschlange) - die einzige Stelle mit
    // dieser Rechnung. 0 = noch nicht entscheidbar. Kann TPUART_BUFFER_SIZE überschreiten (LG 255). Nur L_Data.
    static uint16_t sizeOf(const uint8_t *data, size_t available);

    uint16_t source() const;
    uint16_t destination() const;
    bool isGroupAddress() const;
    bool isRepeated() const;

    // CRC-8/GSM-A: XOR über alles vor der Prüfsumme, invertiert.
    uint8_t calcCRC8() const;

    // Intakt: kein INVALID-Flag, L_Data-Steuerbyte, Länge wie laut Kopf, Prüfsumme stimmt. Beides nötig -
    // das Flag trägt Wissen des Empfängers, die Rechnung gilt für selbst gebaute Telegramme. Nur aus loop().
    bool isValid() const;
    bool isInvalid() const;

    bool isFiltered() const;
    void setFiltered();

    bool isTransmitted() const;
    void setTransmitted();

    // Nur bei selbst gesendeten Telegrammen: die BCU hat den Versand bestätigt (L_Data.con).
    bool isDataCon() const;

    // Wir haben dieses Frame selbst quittiert - im Unterschied zu einer bloß beobachteten Quittung.
    bool isAddressed() const;

    bool isAck() const;
    bool isNack() const;
    bool isBusy() const;

    // Eigene Quittung: ADDRESSED plus ACK (+BUSY/NACK).
    void setAcknowledge(AckType acknowledge);

    // Eine auf dem Bus BEOBACHTETE Quittung: ohne ADDRESSED, denn sie kam nicht von uns.
    void setAcknowledge(bool busy, bool nack);

    // KOMPAT: cEMI-Sicht für den knx-Stack. cemiData() liefert einen malloc-Puffer, den der Aufrufer freigibt.
    // Standard-Telegramme werden ins Extended-Format umsortiert, die Prüfsumme fällt weg.
    uint16_t cemiSize() const;
    uint8_t *cemiData() const;

    // Allozieren über std::string - nur aus dem Hauptkontext.
    std::string humanSource() const;
    std::string humanDestination() const;
    std::string printFrame() const;
};

} // namespace TPUart
