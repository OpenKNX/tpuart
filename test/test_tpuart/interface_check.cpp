// ÜBERSETZUNGS- UND LINKTEST FÜR DIE INTERFACES - kein Test im Unity-Sinn, hier steht keine einzige
// Zusicherung. Die Datei hat trotzdem einen Zweck, und es ist derselbe, den früher die drei
// Produktions-Envs (pico_uart, pico_serial, esp32_uart) samt src/main.cpp erfüllten: dass sich jedes
// Interface dieser Library überhaupt instanziieren, mit einem DataLinkLayer verdrahten und linken lässt.
//
// Weshalb das nicht von selbst mitgeprüft wird, obwohl der Testbau src/ vollständig übersetzt:
//
//   ArduinoSerial ist ein TEMPLATE und steht komplett im Header. Ohne Aufrufer entsteht davon kein
//   einziger Rumpf - ein Tippfehler in write() oder eine Signatur, die nicht zu Abstract passt, fiele
//   erst beim ersten Verbraucher auf, also im Fremdprojekt. Genau das ist hier abgestellt: die
//   ausdrückliche Instanziierung unten erzeugt JEDE Methode, nicht nur die benutzten.
//
//   RP2040 und ESP32 werden als .cpp mitübersetzt, aber ihre Vollständigkeit gegenüber Abstract
//   entscheidet sich erst am Instanziieren. Fehlt ein override, ist die Klasse abstrakt und der
//   Compiler schweigt bis dahin.
//
// Beide Plattformen bringen ihren eigenen Zweig mit, ausgewählt über ARDUINO_ARCH_*, nicht über ein
// TPUART_INTERFACE_*-Flag: die alten Envs prüften je Lauf genau ein Interface, hier laufen alle
// Interfaces der jeweiligen Plattform in einem Bau mit.
#include <Arduino.h>

#include "TPUart/DataLinkLayer.h"
#include "TPUart/Interface/ArduinoSerial.h"

// Auf beiden Plattformen ist Serial1 ein echter UART - auf dem RP2040 SerialUART aus arduino-pico, auf
// dem ESP32 HardwareSerial. decltype statt eines festen Typnamens ist Absicht und der Grund, dass diese
// Klasse ein Template ist: welche Serial-Klasse eine Plattform mitbringt, weiß nur der Aufrufer.
template class TPUart::Interface::ArduinoSerial<decltype(Serial1)>;

#ifdef ARDUINO_ARCH_RP2040
#include "TPUart/Interface/RP2040.h"
#endif

#ifdef ARDUINO_ARCH_ESP32
#include "TPUart/Interface/ESP32.h"
#endif

// NIE AUFGERUFEN, und das ist der Punkt: der Compiler übersetzt den Rumpf vollständig, der Linker löst
// jedes Symbol darin auf, aber kein Konstruktor läuft und kein Interface fasst Hardware an. Ein globales
// Objekt stattdessen würde auf dem RP2040 beim Start einen DMA-Channel beanspruchen, den die echten Tests
// dann nicht mehr bekommen.
void tpuartInterfaceLinkCheck()
{
    TPUart::Interface::ArduinoSerial<decltype(Serial1)> serialInterface(Serial1);
    TPUart::DataLinkLayer serialDatalink(serialInterface);
    serialDatalink.begin(TPUart::BcuType::BCU_TPUART2);
    serialDatalink.process();
    serialDatalink.end();

#ifdef ARDUINO_ARCH_RP2040
    // Dieselbe Belegung, die src/main.cpp benutzte: RX an GPIO 1, TX an GPIO 0, uart0.
    TPUart::Interface::RP2040 picoInterface(1, 0, uart0);
    TPUart::DataLinkLayer picoDatalink(picoInterface);
    picoDatalink.begin(TPUart::BcuType::BCU_NCN5120);
    picoDatalink.process();
    picoDatalink.end();
#endif

#ifdef ARDUINO_ARCH_ESP32
    // Die Pins sind hier reine Platzhalter - geprüft wird das Übersetzen, nicht die Verdrahtung. Welche
    // Pins auf einem Modul überhaupt frei sind, steht im Kommentar an uart_set_pin() im Interface.
    TPUart::Interface::ESP32 espInterface(37, 5, UART_NUM_1);
    TPUart::DataLinkLayer espDatalink(espInterface);
    espDatalink.begin(TPUart::BcuType::BCU_NCN5120);
    espDatalink.process();
    espDatalink.end();
#endif
}
