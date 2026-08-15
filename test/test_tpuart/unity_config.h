// EIGENE UNITY-KONFIGURATION, und der einzige Grund dafür steht in unityOutputComplete().
//
// PlatformIO erzeugt sich diese Datei sonst selbst (siehe platformio/test/runners/unity.py) und legt
// dabei fest, wohin Unity schreibt. Deren Fassung endet mit:
//
//     void unityOutputComplete(void) { Serial.end(); }
//
// Auf Plattformen mit echtem UART ist das harmlos. Auf dem RP2040 ist Serial der USB-CDC, und Serial.end()
// meldet das GERÄT VOM BUS AB - mitten im letzten Handgriff von UNITY_END(). Die Folgen waren genau die
// beobachteten: der Testläufer bekommt die Schlusszeile nicht mehr ("ClearCommError failed" nach 23 von 24
// Fällen), und der nächste Upload findet keine Schnittstelle mehr ("Please specify upload_port"), bis der
// Pico neu gesteckt wird. Beim ESP32 hängt Serial an einem echten UART bzw. am USB-Seriell-Wandler, dort
// fällt es nicht auf - der Fehler gehört also zur Kombination aus nativem USB und dieser Vorgabe.
//
// Liegt eine unity_config.h im Testordner, benutzt PlatformIO sie und übersetzt seine eigene Fassung NICHT
// mit. Damit fehlen dann auch die vier Ausgabefunktionen; die stehen deshalb in test_main.cpp.

#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifndef NULL
#ifndef __cplusplus
#define NULL (void *)0
#else
#define NULL 0
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    void unityOutputStart(unsigned long);
    void unityOutputChar(unsigned int);
    void unityOutputFlush(void);
    void unityOutputComplete(void);

#define UNITY_OUTPUT_START() unityOutputStart((unsigned long)115200)
#define UNITY_OUTPUT_CHAR(c) unityOutputChar(c)
#define UNITY_OUTPUT_FLUSH() unityOutputFlush()
#define UNITY_OUTPUT_COMPLETE() unityOutputComplete()

#ifdef __cplusplus
}
#endif

#endif // UNITY_CONFIG_H
