#include "TPUart/SystemState.h"

namespace TPUart
{

void SystemState::update(uint8_t state)
{
    if (!_valid || _state != state) _dirty = true;

    _state = state;
    _valid = true;
}

bool SystemState::isValid() const
{
    return _valid;
}

uint8_t SystemState::raw() const
{
    return _state;
}

bool SystemState::dirty()
{
    bool dirty = _dirty;
    _dirty = false;
    return dirty;
}

bool SystemState::v20v() const
{
    return (_state & SYSTEM_STAT_V20V) != 0;
}

bool SystemState::vdd2() const
{
    return (_state & SYSTEM_STAT_VDD2) != 0;
}

bool SystemState::vbus() const
{
    return (_state & SYSTEM_STAT_VBUS) != 0;
}

bool SystemState::vfilt() const
{
    return (_state & SYSTEM_STAT_VFILT) != 0;
}

bool SystemState::xtal() const
{
    return (_state & SYSTEM_STAT_XTAL) != 0;
}

bool SystemState::thermalWarning() const
{
    return (_state & SYSTEM_STAT_TW) != 0;
}

uint8_t SystemState::mode() const
{
    return _state & SYSTEM_STAT_MODE_MASK;
}

bool SystemState::normalMode() const
{
    return mode() == SYSTEM_STAT_MODE_NORMAL;
}

bool SystemState::stopMode() const
{
    return mode() == SYSTEM_STAT_MODE_STOP;
}

bool SystemState::syncMode() const
{
    return mode() == SYSTEM_STAT_MODE_SYNC;
}

bool SystemState::powerupMode() const
{
    return mode() == SYSTEM_STAT_MODE_POWERUP;
}

const char *SystemState::modeString() const
{
    switch (mode())
    {
        case SYSTEM_STAT_MODE_NORMAL:
            return "Normal";
        case SYSTEM_STAT_MODE_STOP:
            return "Stop";
        case SYSTEM_STAT_MODE_SYNC:
            return "Sync";
        default:
            return "Power-UP";
    }
}

std::string SystemState::print() const
{
    if (!_valid) return "SystemState: unbekannt (noch keine Antwort der BCU)";

    std::string message = "SystemState: ";
    message += modeString();
    message += " (";
    if (v20v()) message += " V20V";
    if (vdd2()) message += " VDD2";
    if (vbus()) message += " VBUS";
    if (vfilt()) message += " VFILT";
    if (xtal()) message += " XTAL";
    if (thermalWarning()) message += " TW";
    message += " )";

    return message;
}

} // namespace TPUart
