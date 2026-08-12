#pragma once
#include <string>

namespace TPUart
{
    class SystemState
    {
      private:
        volatile char _state = 0;
        volatile bool _dirty = false;
        volatile bool _seen = false; // set once any state reply has been parsed (never on a non-NCN chip)

      public:
        bool v20v();
        bool vdd2();
        bool vbus();
        bool vfilt();
        bool xtal();
        bool thermalWarning();
        char mode();
        const char * modeString();
        std::string print();
        bool normalMode();
        bool stopMode();
        bool syncMode();
        bool powerupMode();

        void update(char state);
        bool dirty();
        bool seen(); // has any system-state reply been parsed? (false on non-NCN chips -> no VBUS telemetry)
    };

} // namespace TPUart
