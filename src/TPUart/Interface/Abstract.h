#pragma once

#include <stddef.h>
#include <stdint.h>

namespace TPUart
{
namespace Interface
{


// UART-Schnittstelle. Destruktor und overflow() haben ihren Rumpf in Abstract.cpp - dort liegt die vtable.
class Abstract
{
  public:
    virtual ~Abstract();

    virtual void begin(uint32_t baud) = 0;
    virtual void end() = 0;

    // Verwirft ungelesene empfangene Bytes, nicht blockierend - anders als Stream::flush().
    virtual void flush() = 0;

    // Anzahl bereitliegender Bytes - der Aufrufer erkennt daran einen Rückstand.
    virtual size_t available() = 0;
    virtual int read() = 0;

    // Bytes, die write() JETZT ohne Blockieren annimmt. Muss stimmen, nicht zu klein und nicht zu groß -
    // siehe TPUART_TX_INTERFACE_BUFFER.
    virtual size_t availableForWrite() = 0;

    // Blockiert nicht; der Aufrufer prüft vorher availableForWrite().
    virtual bool write(char value) = 0;

    virtual bool overflow();
};

} // namespace Interface
} // namespace TPUart
