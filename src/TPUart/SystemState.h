#pragma once
#include <stdint.h>

#include <string>

#include "TPUart/Types.h"

namespace TPUart
{

// Der System-Status des NCN512x: das Folgebyte eines U_SystemStat.ind (0x4B). Er kommt ausschließlich als
// Antwort auf ein U_SystemState.req, also auf requestState() hin - von selbst meldet der Chip ihn nie.
//
// Übernommen aus der alten Library, mit zwei Änderungen:
//   - EIN FEHLER IST KORRIGIERT: dort war SYSTEM_STATE_MODE_NORMAL als 0x33 definiert, verglichen wurde
//     aber gegen mode(), das mit 0x03 maskiert - normalMode() konnte damit nie true werden. modeString()
//     hat es richtig gemacht, weil es direkt gegen 0x03 verglich.
//   - _valid kam dazu: ohne Antwort ist _state 0, was sonst als "Power-UP, alles aus" gelesen würde.
//
// Kein volatile: geschrieben wird ausschließlich aus DataLinkLayer::loop() (Hauptkontext), nicht aus tick().
class SystemState
{
  private:
    uint8_t _state = 0;
    bool _valid = false;
    bool _dirty = false;

  public:
    void update(uint8_t state);

    // Solange false, hat der Chip noch nie geantwortet - alle Abfragen unten sind dann bedeutungslos.
    bool isValid() const;

    uint8_t raw() const;

    // Meldet einmalig, dass sich der Status geändert hat (oder erstmals eintraf), und setzt sich dabei
    // zurück. Damit lässt sich im Hauptloop ohne eigenen Vergleich auf Änderungen reagieren.
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

    // Fordert Speicher an (std::string) - nur aus dem Hauptkontext aufrufen, nie aus tick().
    // Aufgeführt werden die Flags, die gesetzt sind; ein fehlendes VDD2 heißt also: Regler ist aus.
    std::string print() const;
};

} // namespace TPUart
