#pragma once
#include <stdint.h>

#include <string>

#include "TPUart/Types.h"

namespace TPUart
{

// System-Status des NCN512x: Folgebyte eines U_SystemStat.ind, nur als Antwort auf requestState().
// ACHTUNG: SYSTEM_STAT_MODE_NORMAL muss zur Maske 0x03 passen, sonst wird normalMode() nie wahr.
// Nur Hauptkontext, daher kein volatile.
class SystemState
{
  private:
    uint8_t _state = 0;
    bool _valid = false;
    bool _dirty = false;

  public:
    void update(uint8_t state);

    // false: der Chip hat noch nie geantwortet - alles andere ist dann bedeutungslos.
    bool isValid() const;

    uint8_t raw() const;

    // Einmalig true nach einer Änderung.
    bool dirty();

    bool v20v() const;
    bool vdd2() const;
    bool vbus() const;
    bool vfilt() const;
    bool xtal() const;
    bool thermalWarning() const;

    uint8_t mode() const;
    bool normalMode() const;
    bool stopMode() const;
    bool syncMode() const;
    bool powerupMode() const;

    const char *modeString() const;

    // Nur aus dem Hauptkontext (std::string). Aufgeführt werden die gesetzten Flags.
    std::string print() const;
};

} // namespace TPUart
