#pragma once

// ARDUINO-ERSATZ FÜR DEN NATIVEN LAUF - und der ist überraschend klein, weil die Library aus Arduino
// GENAU ZWEI FUNKTIONEN benutzt: millis() und micros(). Sonst nichts. Die plattformgebundenen Teile
// schalten sich selbst ab (Interface/RP2040.cpp und ESP32.cpp stehen ganz hinter ihren ARDUINO_ARCH_*,
// TickDriver.cpp hat einen else-Zweig, der supported() auf false setzt).
//
// DIESE DATEI WIRD NUR IN DER NATIVEN ENV GEFUNDEN, weil nur sie den Ordner über -I in den Suchpfad
// nimmt. In den Hardware-Envs liegt das echte Arduino.h davor - der Schim ist dort unsichtbar.
//
// SIE IST HEADER-ONLY, und das ist Absicht: läge hier eine .cpp, würde PlatformIO sie in JEDER Env
// mitübersetzen (der Testordner wird rekursiv gebaut) und die Hardware-Envs bekämen ein zweites
// millis(). Der Zählerstand steckt deshalb in einem funktionslokalen static.
//
// DIE UHR STEHT UND WIRD GESTELLT. Das ist der eigentliche Gewinn des nativen Laufs: die Testvorrichtung
// dreht sie in festen Schritten weiter, statt auf echte Zeit zu warten - aus fünf Sekunden
// Verbindungsabbruch wird ein Schleifendurchlauf.
//
// WAS DAMIT NICHT MEHR GEPRÜFT WIRD: das Zeitverhalten selbst. Auf der Hardware misst die Sammlung echte
// Fristen gegen eine echte Uhr, und genau daran ist schon ein Fehler aufgefallen, den eine gestellte Uhr
// wegdefiniert (die Zustellkosten des Telegramm-Callbacks verschoben die Pausenerkennung). Der native
// Lauf ist deshalb eine ZUSÄTZLICHE Absicherung der Protokolllogik, kein Ersatz für die Hardwareläufe.

#include <stddef.h>
#include <stdint.h>

// Der einzige Zeitgeber. Beide Seiten müssen ihn sehen - die Library für die Pausenerkennung und das
// Dummy-Interface für seinen Fahrplan -, sonst laufen Skript und Auswertung auseinander und das Ergebnis
// wird stillschweigend falsch statt rot.
inline uint32_t &tpuartNativeClockUs()
{
    static uint32_t now = 0;
    return now;
}

inline uint32_t micros()
{
    return tpuartNativeClockUs();
}

inline uint32_t millis()
{
    return tpuartNativeClockUs() / 1000;
}

// Wartende Funktionen drehen die Uhr weiter, statt zu blockieren - sonst stünde ein delay() im Testcode
// still und nichts käme voran.
inline void delay(uint32_t ms)
{
    tpuartNativeClockUs() += ms * 1000;
}

inline void delayMicroseconds(uint32_t us)
{
    tpuartNativeClockUs() += us;
}
