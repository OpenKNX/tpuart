#pragma once
#include <stddef.h>
#include <stdint.h>

namespace TPUart
{

// Größtmögliches Frame: Extended = 9 Byte Metadaten + bis zu 254 Byte APDU. Im Längenbyte eines
// Extended-Frames ist der Wert 255 reserviert, die APDU wird also nie länger als 254 - ein LG von 255
// ist damit kein gültiges Frame, sondern ein Fehlerfall (siehe Größenprüfung in Receiver::processFrameByte).
//
// Steht hier und nicht bei einer der beiden Hälften, weil beide dieselbe Größe brauchen: der Empfangspuffer
// des Receivers und der Sendepuffer des Transmitters.
constexpr size_t TPUART_BUFFER_SIZE = 263;

// Zustand des Empfangs - beschreibt ausschließlich, was gerade EINLÄUFT. Fertige Sequenzen gibt es hier
// bewusst nicht mehr als Zustand: sobald etwas vollständig ist, wandert es sofort in den RX-Ringpuffer und
// der Zustand geht direkt weiter. Es muss also nichts abgeholt werden, und nichts kann den Empfang
// aufhalten.
enum class RxState : uint8_t
{
    Idle,     // nichts angefangen
    Frame,    // Frame läuft ein
    FrameAck, // Frame ist da, es fehlt noch die Antwort (Quittung im Busmonitor bzw. L_Data.con)
    Control,  // Steuerbyte-Sequenz läuft (einziger Fall: U_SystemStat.ind, 2. Byte fehlt noch)
    Poll,     // Poll-Telegramm läuft ein - Grenze aus dem Slot-Count, siehe L_POLL_DATA_IND
    Resync,   // Position im Bytestrom unbekannt - alles wird verworfen, bis eine verifizierte Pause kommt
};

// Zustand des TELEGRAMM-Versands - und nur das. Weder das Acknowledge zu einem einlaufenden Frame noch ein
// Steuercode an die BCU ist hier ein Zustand: beide sind einzelne Byte-Folgen, die zwischendurch rausgehen,
// ohne einen laufenden Sendevorgang zu betreffen. Für Steuercodes gilt das erst seit sie ihre eigene
// Warteschlange haben - vorher belegte ein wartender Steuercode diesen Zustand und blockierte damit alles
// andere. Da Empfang und Telegrammversand nie gleichzeitig laufen, können sich RxState und TxState nicht
// ins Gehege kommen.
enum class TxState : uint8_t
{
    Idle,     // bereit, ein Telegramm aus der Warteschlange zu holen
    Transmit, // Sendepuffer der BCU wird befüllt (U_L_DataStart/Cont/End), ein Oktett je Tick
    Await,    // Telegramm ist raus, warte auf die Bestätigung der BCU (L_Data.con)
};

// Zustand der Verbindung zur BCU, wie BcuState in der alten Library. Nicht gespeichert, sondern aus den
// vorhandenen Flags abgeleitet - so gibt es keinen zweiten Schreiber für ein Feld, das Tick und Hauptkontext
// beide anfassen müssten.
enum class BcuState : uint8_t
{
    Uninitialized, // begin() nicht gerufen, oder die Baudratenerkennung läuft noch (nie verbunden gewesen)
    Connected,     // die BCU antwortet
    Disconnected,  // war verbunden, antwortet aber nicht mehr - die Erkennung läuft wieder
};

// DLL services (device is transparent) - erstes Byte entscheidet, ob eine Sequenz ein Frame ist.
constexpr uint8_t L_DATA_STANDARD_IND = 0x90;
constexpr uint8_t L_DATA_EXTENDED_IND = 0x10;
constexpr uint8_t L_DATA_MASK = 0xD3;

// Poll-Telegramm: ein Master fragt mit EINEM Telegramm bis zu 15 Geräte ab, und jedes antwortet mit einem
// einzelnen Byte in seinem Slot - ohne eigenen Rahmen und ohne Quittung. Auf dem Bus sieht das so aus
// (NCN Figure 56, Siemens Fig. 23): Control, Source 2, Poll-Adresse 2, Slot Count, Prüfsumme, danach die
// Slots. Die Prüfsumme deckt nur den Kopf ab, die Slots liegen dahinter.
//
// ZUM HOST GEHT DAVON REGULÄR NUR DAS STEUERBYTE. Siemens TP-UART 2 S. 32 / 2+ S. 33 wörtlich: "From a
// L_PollData-request only the Controlbyte is transmitted to the host if the TP-UART is a polling slave.
// If the TP-UART is polling master the complete polling frame is transmitted to the host as well if a
// collision is detected during sending the polling master frame." In Figure 56 stehen die übrigen Bytes
// deshalb auch in der Spalte "KNX Bus", nicht in der Host-Spalte. Was der Chip durchreicht, wenn er weder
// Master noch Slave ist, sagt kein Datenblatt.
//
// DER PARSER DECKT BEIDE FÄLLE AB (RxState::Poll, Receiver::processPollByte): kommt nichts nach, schließt
// die Pause die Sequenz ab; kommt der ganze Zyklus, wird er anhand des Slot-Counts abgezählt und ohne
// Pause übersprungen. Als 1-Byte-Steuerbyte darf es keinesfalls durchgehen - die Folgebytes würden dann
// als Anfang neuer Sequenzen gelesen, und ein Source-High-Byte von 0x10 (Master 1.0.x) sieht wie ein
// Extended-Frame-Start aus, mit einem ungewollten U_Ackn.req auf eine Fantasie-Zieladresse als Folge.
constexpr uint8_t L_POLL_DATA_IND = 0xF0;

// Kopflänge eines Poll-Telegramms auf dem Bus (Steuerbyte, Quelle 2, Poll-Adresse 2, Slot Count,
// Prüfsumme) und die höchste plausible Slot-Zahl. Die Slotnummern gehen laut U_PollingState.req über
// 0...14 (NCN Table 12 S. 33, Siemens Fig. 3), mehr als 15 Slots kann es also nicht geben - ein größerer
// Wert im Slot-Count-Byte heißt, dass die Länge nicht stimmt.
constexpr uint8_t L_POLL_DATA_HEADER_SIZE = 7;
constexpr uint8_t L_POLL_DATA_MAX_SLOTS = 15;

// Acknowledge-Dienste. Die Quittung vom Bus erreicht den Host nur im Busmonitor-Modus (Datenblatt S. 34
// und Figure 35). Erkennungsmuster dort: x x 0 0 x x 0 0. Die beiden Bit-Paare sind invertiert zu lesen -
// gesetzte Maskenbits bedeuten "nicht busy" bzw. "nicht nack" (TP1-Codes: 0xCC = ACK, 0x0C = NACK,
// 0xC0 = BUSY).
constexpr uint8_t L_ACKN_IND = 0x00;
constexpr uint8_t L_ACKN_MASK = 0x33;
constexpr uint8_t L_ACKN_BUSY_MASK = 0x0C;
constexpr uint8_t L_ACKN_NACK_MASK = 0xC0;
constexpr uint8_t L_DATA_CON = 0x0B;
constexpr uint8_t L_DATA_CON_MASK = 0x7F;

// Dienste an den Chip (NCN5130 Table 12, "Services from Host Controller")
constexpr uint8_t U_RESET_REQ = 0x01;
constexpr uint8_t U_STATE_REQ = 0x02;
constexpr uint8_t U_SYSTEM_STATE_REQ = 0x0D; // nur NCN512x
constexpr uint8_t U_STOP_MODE_REQ = 0x0E;    // nur NCN512x
constexpr uint8_t U_EXIT_STOP_MODE_REQ = 0x0F;

// Eigene Adresse und Wiederholungszähler. Beide Dienste gibt es auf beiden Chips - mit unterschiedlichen
// Opcodes, unterschiedlicher Sequenzlänge UND unterschiedlichem Bitlayout des Zählerbytes.
//
// ACHTUNG bei 0x28: beim TPUART2 ist das U_SetAddress, beim NCN512x dagegen U_IntRegWr.req (28-2B, Zugriff
// auf die internen Register - 0x29 ist dort unser ACR0-Schreibzugriff). Dieselbe Zahl, völlig andere
// Bedeutung; die Verwechslung würde am NCN in ein Register schreiben.
constexpr uint8_t U_NCN5120_SET_ADDRESS_REQ = 0xF1;    // + AddrHigh + AddrLow + Dummy = 4 Byte
constexpr uint8_t U_NCN5120_SET_REPETITION_REQ = 0xF2; // + Zähler + 2 Dummy = 4 Byte
constexpr uint8_t U_TPUART2_SET_ADDRESS_REQ = 0x28;    // + AddrHigh + AddrLow = 3 Byte
constexpr uint8_t U_TPUART2_SET_REPETITION_REQ = 0x24; // U_MxRstCnt + Zähler = 2 Byte

// Das Zählerbyte, und hier lohnt der genaue Blick - die beiden Chips legen den BUSY-Zähler anders:
//   NCN5130 (Figure 37):  0 b b b 0 n n n   Busy in Bit 6-4, "Bit 3 und 7 müssen null sein"
//   TPUART2 (Fig. 20):    c c c 0 0 c c c   Busy in Bit 7-5, Nack in Bit 2-0
// Der Nack-Zähler liegt bei beiden in Bit 2-0. Die alte Library hielt einen NCN-formatierten Wert vor und
// rechnete ihn für den TPUART2 um - mit `||` statt `|`, wodurch dort immer 0 oder 1 herauskam. Hier werden
// die beiden Zähler getrennt gehalten und je Chip zusammengesetzt; damit gibt es kein Zwischenformat, in
// dem sich so ein Fehler verstecken könnte.
constexpr uint8_t U_NCN5120_REPETITION_BUSY_SHIFT = 4;
constexpr uint8_t U_TPUART2_REPETITION_BUSY_SHIFT = 5;
constexpr uint8_t U_REPETITION_COUNTER_MASK = 0x07; // 0...7 laut beiden Datenblättern

// Busy-Modus: derselbe Zweck, aber unterschiedliche Opcodes je Chip.
constexpr uint8_t U_NCN5120_SET_BUSY_REQ = 0x03;
constexpr uint8_t U_NCN5120_QUIT_BUSY_REQ = 0x04;
constexpr uint8_t U_TPUART2_SET_BUSY_REQ = 0x21;
constexpr uint8_t U_TPUART2_QUIT_BUSY_REQ = 0x22;

// Interne Register des NCN512x. ACR0 steuert die Spannungsregler; geschrieben wird als 2-Byte-Sequenz
// (Opcode, Wert), die ununterbrochen rausgehen muss.
constexpr uint8_t U_INT_REG_WR_REQ_ACR0 = 0x29;
constexpr uint8_t ACR0_FLAG_V20VEN = 0x40;     // 20V-Regler an
constexpr uint8_t ACR0_FLAG_DC2EN = 0x20;      // VCC2 (DC-DC) an
constexpr uint8_t ACR0_FLAG_XCLKEN = 0x10;     // Taktausgang für externe Bausteine
constexpr uint8_t ACR0_FLAG_TRIGEN = 0x08;     // Trigger-Ausgang
constexpr uint8_t ACR0_FLAG_V20VCLIMIT = 0x04; // Strombegrenzung des 20V-Reglers

// Busmonitor: alles vom Bus wird ungefiltert an den Host durchgereicht, Acknowledge-Frames eingeschlossen.
// Der Chip ist dabei transparent und quittiert selbst nichts. Verlassen lässt sich der Zustand laut
// Datenblatt (S. 36, Figure 35) AUSSCHLIESSLICH über den Reset-Service - es gibt kein "Busmon aus".
constexpr uint8_t U_BUSMON_REQ = 0x05;

// Telegrammversand (Table 12, Figure 42-44). Jedes Oktett wird einzeln übergeben, mit vorangestelltem
// Positionsbyte: das erste mit U_L_DataStart.req, jedes weitere mit U_L_DataCont.req - beide sind
// 0x80 | Position, eine Fallunterscheidung braucht es also nicht. Das LETZTE Byte ist die Prüfsumme und
// bekommt U_L_DataEnd.req; erst damit startet der Chip die Übertragung auf den Bus.
//
// Im Positionsbyte stecken nur 6 Bit. Ein Extended-Frame ist aber bis zu 263 Byte lang, es braucht also
// 9 Bit: U_L_DataOffset.req liefert die oberen 3. Der Chip merkt sich den Offset, bis ein neuer kommt -
// gesendet wird er deshalb nur bei Änderung.
constexpr uint8_t U_L_DATA_START_REQ = 0x80;
constexpr uint8_t U_L_DATA_END_REQ = 0x40;
constexpr uint8_t U_L_DATA_OFFSET_REQ = 0x08;
constexpr uint8_t U_L_DATA_POSITION_MASK = 0x3F;

// U_Ackn.req = 0x10 | n<<2 | b<<1 | a. Muss noch WÄHREND des laufenden Frames gesendet werden - der Chip
// legt den Immediate-Acknowledge unmittelbar nach dem Checksum-Byte selbst auf den Bus (Figures 50-52).
constexpr uint8_t U_ACKN_REQ = 0x10;
constexpr uint8_t U_ACKN_REQ_NACK = 0x04;
constexpr uint8_t U_ACKN_REQ_BUSY = 0x02;
constexpr uint8_t U_ACKN_REQ_ADDRESSED = 0x01;

// Flags eines Telegramms - das Byte ist die API zum übrigen KNX-Stack, die BITBELEGUNG LIEGT ALSO FEST
// und darf nicht umsortiert werden. Ein Verbraucher, der das Byte numerisch liest statt über die
// Zugriffsmethoden von Frame, bekäme sonst stillschweigend etwas anderes.
// Bit 4 war lange als ECHO reserviert und unbenutzt; dort sitzt jetzt INVALID. Damit ist das Byte voll -
// ein weiteres Flag bräuchte ein zweites Byte im Warteschlangeneintrag.
//
// Zu den Quittungs-Bits: ACK heißt "dieses Telegramm wurde quittiert" - unabhängig davon, von wem. Das
// kann in drei Situationen bekannt sein: wir haben selbst quittiert, wir haben die Quittung im Busmonitor
// auf dem Bus beobachtet, oder sie kam als Antwort auf ein von uns gesendetes Telegramm. ADDRESSED kommt
// nur im ersten Fall dazu und trennt ihn von den anderen beiden.
constexpr uint8_t TP_FRAME_FLAG_TX = 0b10000000;        // Von uns selbst gesendet
constexpr uint8_t TP_FRAME_FLAG_DATA_CON = 0b01000000;  // Mit L_Data.con beantwortet
constexpr uint8_t TP_FRAME_FLAG_FILTERED = 0b00100000;  // Soll vom Gerät gefiltert werden
constexpr uint8_t TP_FRAME_FLAG_INVALID = 0b00010000;   // Frame ist kaputt: CRC falsch, abgeschnitten oder Länge korrupt
constexpr uint8_t TP_FRAME_FLAG_ADDRESSED = 0b00001000; // Von diesem Gerät verarbeitet - wir haben quittiert
constexpr uint8_t TP_FRAME_FLAG_ACK_BUSY = 0b00000100;  // Quittung war BUSY
constexpr uint8_t TP_FRAME_FLAG_ACK_NACK = 0b00000010;  // Quittung war NACK
constexpr uint8_t TP_FRAME_FLAG_ACK = 0b00000001;       // Quittung liegt vor

// Chip-Typ der BCU (Bus Coupling Unit) - bestimmt, welche Baudraten bei der Verbindungsaufnahme überhaupt
// in Frage kommen (siehe DataLinkLayer::begin()).
enum class BcuType : uint8_t
{
    Tpuart2, // Siemens 5WG1117-2AB12 "TPUart 2" - fest 19200 Baud
    Ncn5120, // OnSemi NCN5120/NCN5121/NCN5130 - 19200 oder 38400 Baud

    // KOMPAT: Schreibweise der alten Library. Gleiche Werte, damit vorhandener Aufrufercode unverändert
    // übersetzt. Kann weg, sobald knx/OGM-Common umgestellt sind.
    BCU_TPUART2 = Tpuart2,
    BCU_NCN5120 = Ncn5120,
};

enum class AckType : uint8_t
{
    None = 0, // kein U_Ackn.req senden - Frame geht uns nichts an
    Addressed = U_ACKN_REQ_ADDRESSED,
    Busy = U_ACKN_REQ_ADDRESSED | U_ACKN_REQ_BUSY,
    Nack = U_ACKN_REQ_ADDRESSED | U_ACKN_REQ_NACK,

    // KOMPAT: Schreibweise der alten Library, gleiche Werte.
    ACK_None = None,
    ACK_Addressed = Addressed,
    ACK_Busy = Busy,
    ACK_Nack = Nack,
};

// KOMPAT: hieß in der alten Library AcknowledgeType.
using AcknowledgeType = AckType;

// Übersetzt eine Quittung in die zugehörigen Frame-Flags. ADDRESSED kommt NICHT von hier - das setzt der
// Aufrufer nur dazu, wenn die Quittung von uns selbst stammt (eine beobachtete trägt es nicht).
//
// Steht hier, weil es eine reine Übersetzung zwischen zwei Dingen aus dieser Datei ist - AckType oben, die
// TP_FRAME_FLAG_* darüber. Vorher gab es sie zweimal: als dateilokalen Helfer im Receiver und ausgeschrieben
// in Frame::setAcknowledge().
uint8_t acknowledgeFlags(AckType acknowledge);

// Steuerdienste, geräte-/chip-spezifisch
constexpr uint8_t U_RESET_IND = 0x03;
constexpr uint8_t U_STATE_MASK = 0x07;
constexpr uint8_t U_STATE_IND = 0x07;
constexpr uint8_t U_CONFIGURE_IND = 0x01;
constexpr uint8_t U_CONFIGURE_MASK = 0x83;

// Bits im U_Configure.ind (NCN5130 Table 13: "0 b aa ap c m 0 1"). Damit meldet der Chip, welche
// Betriebsarten aktiv sind - insbesondere bestätigt er hierüber das Aktivieren der Auto-Quittung.
constexpr uint8_t U_CONFIGURE_AUTO_ACKNOWLEDGE = 0x20;
constexpr uint8_t U_CONFIGURE_AUTO_POLLING = 0x10;
constexpr uint8_t U_CONFIGURE_CRC_CCITT = 0x08;
constexpr uint8_t U_CONFIGURE_MARKER = 0x04;
constexpr uint8_t U_SYSTEM_STAT_IND = 0x4B; // nur NCN5120
constexpr uint8_t U_STOP_MODE_IND = 0x2B;   // nur NCN5120
constexpr uint8_t U_FRAME_END_IND = 0xCB;
constexpr uint8_t U_FRAME_STATE_IND = 0x13;
constexpr uint8_t U_FRAME_STATE_MASK = 0x17;

// Fehlerbits im U_State.ind. Die unteren drei Bits identifizieren nur den Dienst (U_STATE_MASK), alles
// darüber sind Fehleranzeigen des Chips - sie stehen für ein SEIT DER LETZTEN ABFRAGE aufgetretenes
// Ereignis, nicht für einen Dauerzustand, und sind nach dem Melden beim Chip wieder gelöscht.
constexpr uint8_t U_STATE_SLAVE_COLLISION = 0x80;
constexpr uint8_t U_STATE_RECEIVE_ERROR = 0x40;
constexpr uint8_t U_STATE_TRANSMIT_ERROR = 0x20;
constexpr uint8_t U_STATE_PROTOCOL_ERROR = 0x10;
constexpr uint8_t U_STATE_TEMPERATURE_WARNING = 0x08;

// Bits im Folgebyte des U_SystemStat.ind. Belegung wie in der alten Library.
constexpr uint8_t SYSTEM_STAT_V20V = 0x80;  // 20V-Linearregler im normalen Arbeitsbereich
constexpr uint8_t SYSTEM_STAT_VDD2 = 0x40;  // DC2-Regler (VCC2) im normalen Arbeitsbereich
constexpr uint8_t SYSTEM_STAT_VBUS = 0x20;  // Busspannung im normalen Arbeitsbereich
constexpr uint8_t SYSTEM_STAT_VFILT = 0x10; // Spannung am Pufferkondensator in Ordnung
constexpr uint8_t SYSTEM_STAT_XTAL = 0x08;  // Quarzoszillator läuft im Sollbereich
constexpr uint8_t SYSTEM_STAT_TW = 0x04;    // Übertemperatur-Warnung
constexpr uint8_t SYSTEM_STAT_MODE_MASK = 0x03;
constexpr uint8_t SYSTEM_STAT_MODE_POWERUP = 0x00;
constexpr uint8_t SYSTEM_STAT_MODE_SYNC = 0x01;
constexpr uint8_t SYSTEM_STAT_MODE_STOP = 0x02;
constexpr uint8_t SYSTEM_STAT_MODE_NORMAL = 0x03;

// Klartextname eines Steuerbytes, nullptr wenn keiner der bekannten Dienste passt. Gedacht für Diagnose-
// ausgaben: ein unbekanntes Steuerbyte ist ein Hinweis darauf, dass der Bytestrom fehlgedeutet wird.
const char *controlServiceName(uint8_t value);

} // namespace TPUart
