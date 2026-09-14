#pragma once
#include <stddef.h>
#include <stdint.h>

namespace TPUart
{

// Größtes Frame: Extended mit 9 Byte Kopf + 254 Byte APDU (LG 255 ist reserviert).
constexpr size_t TPUART_BUFFER_SIZE = 263;

// Zustand des Empfangs. Fertige Sequenzen gehen sofort in den Ringpuffer.
enum class RxState : uint8_t
{
    Idle,     // nichts angefangen
    Frame,    // Frame läuft ein
    FrameAck, // Frame ist da, es fehlt noch die Antwort (Quittung im Busmonitor bzw. L_Data.con)
    Control,  // Steuerbyte-Sequenz läuft (einziger Fall: U_SystemStat.ind, 2. Byte fehlt noch)
    Poll,     // Poll-Telegramm läuft ein - Grenze aus dem Slot-Count, siehe L_POLL_DATA_IND
    Resync,   // Position im Bytestrom unbekannt - alles wird verworfen, bis eine verifizierte Pause kommt
};

// Zustand des Telegrammversands. Acknowledge und Steuercodes sind kein Zustand.
enum class TxState : uint8_t
{
    Idle,     // bereit, ein Telegramm aus der Warteschlange zu holen
    Transmit, // Sendepuffer der BCU wird befüllt (U_L_DataStart/Cont/End), ein Oktett je Tick
    Await,    // Telegramm ist raus, warte auf die Bestätigung der BCU (L_Data.con)
};

// Zustand der Verbindung zur BCU. Wer welchen Übergang schreibt: DataLinkLayer::_bcuState.
enum class BcuState : uint8_t
{
    Uninitialized, // begin() nicht gerufen, oder nach end()
    Searching,     // Baudratensuche - das Interface gehört dem Hauptkontext
    Identifying,   // verbunden, der Chip wird bestimmt (nur NCN, einmal je begin())
    Connected,     // die BCU antwortet
    BusMonitor,    // nach U_Busmon.req - verlassen nur per Reset
    Disconnected,  // war verbunden, antwortet nicht mehr - der Tick verbindet neu
};

// DLL services (device is transparent) - erstes Byte entscheidet, ob eine Sequenz ein Frame ist.
constexpr uint8_t L_DATA_STANDARD_IND = 0x90;
constexpr uint8_t L_DATA_EXTENDED_IND = 0x10;
constexpr uint8_t L_DATA_MASK = 0xD3;

// Rang der Priorität aus den Bits 3-2 des Steuerbytes. ACHTUNG: nicht die Reihenfolge der Rohwerte -
// roh 0 System = Rang 0, roh 1 Normal = Rang 2, roh 2 Urgent = Rang 1, roh 3 Low = Rang 3 (wie knx_types.h).
uint8_t telegramPriorityRank(uint8_t control);

// Anzahl der Prioritätsklassen; nur für die niedrigste gilt die Reserve im Sendepuffer.
constexpr uint8_t TP_PRIORITY_COUNT = 4;
constexpr uint8_t TP_PRIORITY_LOW = 3;

// Poll-Telegramm: Control, Source 2, Poll-Adresse 2, Slot Count, Prüfsumme, danach die Slots. Zum Host geht
// regulär nur das Steuerbyte; der Parser deckt auch den ganzen Zyklus ab (RxState::Poll). Nie als
// 1-Byte-Steuerbyte behandeln - die Folgebytes sähen sonst wie neue Frames aus.
constexpr uint8_t L_POLL_DATA_IND = 0xF0;

// Kopflänge eines Poll-Telegramms und die höchste mögliche Slot-Zahl (Slots 0...14).
constexpr uint8_t L_POLL_DATA_HEADER_SIZE = 7;
constexpr uint8_t L_POLL_DATA_MAX_SLOTS = 15;

// Quittung vom Bus, nur im Busmonitor. Die Bit-Paare sind invertiert: gesetzt heißt "nicht busy/nack".
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

// Adresse und Wiederholungszähler: auf beiden Chips, aber mit anderen Opcodes, Längen und Bitlayouts.
// ACHTUNG: 0x28 ist beim TPUART2 U_SetAddress, beim NCN512x U_IntRegWr.req.
constexpr uint8_t U_NCN5120_SET_ADDRESS_REQ = 0xF1;    // + AddrHigh + AddrLow + Dummy = 4 Byte
constexpr uint8_t U_NCN5120_SET_REPETITION_REQ = 0xF2; // + Zähler + 2 Dummy = 4 Byte
constexpr uint8_t U_TPUART2_SET_ADDRESS_REQ = 0x28;    // + AddrHigh + AddrLow = 3 Byte
constexpr uint8_t U_TPUART2_SET_REPETITION_REQ = 0x24; // U_MxRstCnt + Zähler = 2 Byte

// Zählerbyte: Nack in Bit 2-0 bei beiden; Busy in Bit 6-4 (NCN) bzw. Bit 7-5 (TPUART2).
constexpr uint8_t U_NCN5120_REPETITION_BUSY_SHIFT = 4;
constexpr uint8_t U_TPUART2_REPETITION_BUSY_SHIFT = 5;
constexpr uint8_t U_REPETITION_COUNTER_MASK = 0x07; // 0...7 laut beiden Datenblättern

// Busy-Modus: derselbe Zweck, aber unterschiedliche Opcodes je Chip.
constexpr uint8_t U_NCN5120_SET_BUSY_REQ = 0x03;
constexpr uint8_t U_NCN5120_QUIT_BUSY_REQ = 0x04;
constexpr uint8_t U_TPUART2_SET_BUSY_REQ = 0x21;
constexpr uint8_t U_TPUART2_QUIT_BUSY_REQ = 0x22;

// Interne Register - nur NCN512x. Lesen über 38-3F: RevID (0x3D) antwortet, obwohl das Datenblatt 38-3B nennt.
// ACHTUNG: 0x28 ist beim TPUART2 U_SetAddress.req.
constexpr uint8_t U_INT_REG_WR_REQ = 0x28;
constexpr uint8_t U_INT_REG_WR_ADDRESS_MASK = 0x03;
constexpr uint8_t U_INT_REG_RD_REQ = 0x38;
constexpr uint8_t U_INT_REG_RD_ADDRESS_MASK = 0x07;

// Registernummern laut NCN5130/D Table 14.
constexpr uint8_t NCN_REG_WD = 0x00;    // Watchdog
constexpr uint8_t NCN_REG_ACR0 = 0x01;  // Analog Control 0 - Spannungsregler, siehe NCN_ACR0_FLAG_*
constexpr uint8_t NCN_REG_ACR1 = 0x02;  // Analog Control 1 - Spannungsüberwachung
constexpr uint8_t NCN_REG_ASR0 = 0x03;  // Analog Status 0 - nur lesbar
constexpr uint8_t NCN_REG_REVID = 0x05; // Revision ID - nur 5121/5130

// Resetwerte, an denen bcuChip() den Chip erkennt.
constexpr uint8_t NCN_WD_RESET = 0x0F;
constexpr uint8_t NCN_ACR1_RESET_5120 = 0x00;
constexpr uint8_t NCN_ACR1_RESET_5121_5130 = 0x60;

// RevID: [7:5] Silizium-Revision, [4:0] Teilenummer.
constexpr uint8_t NCN_REVID_PART_MASK = 0x1F;
constexpr uint8_t NCN_REVID_REVISION_SHIFT = 5;
constexpr uint8_t NCN_PART_5130 = 0x0C;
constexpr uint8_t NCN_PART_5121 = 0x0D;

constexpr uint8_t NCN_ACR0_FLAG_V20VEN = 0x40;     // 20V-Regler an
constexpr uint8_t NCN_ACR0_FLAG_DC2EN = 0x20;      // VCC2 (DC-DC) an
constexpr uint8_t NCN_ACR0_FLAG_XCLKEN = 0x10;     // Taktausgang für externe Bausteine
constexpr uint8_t NCN_ACR0_FLAG_TRIGEN = 0x08;     // Trigger-Ausgang
constexpr uint8_t NCN_ACR0_FLAG_V20VCLIMIT = 0x04; // Strombegrenzung des 20V-Reglers

// Busmonitor: alles vom Bus inklusive Quittungen geht an den Host. Verlassen nur per Reset.
constexpr uint8_t U_BUSMON_REQ = 0x05;

// Telegrammversand: jedes Oktett mit Positionsbyte (Start/Cont = 0x80 | Position), das letzte - die
// Prüfsumme - mit U_L_DataEnd.req, das die Übertragung startet. Positionen über 63 brauchen
// U_L_DataOffset.req (nur NCN512x), gesendet nur bei Änderung.
constexpr uint8_t U_L_DATA_START_REQ = 0x80;
constexpr uint8_t U_L_DATA_END_REQ = 0x40;
constexpr uint8_t U_L_DATA_OFFSET_REQ = 0x08;
constexpr uint8_t U_L_DATA_POSITION_MASK = 0x3F;

// U_Ackn.req = 0x10 | n<<2 | b<<1 | a. Muss noch während des laufenden Frames gesendet werden.
constexpr uint8_t U_ACKN_REQ = 0x10;
constexpr uint8_t U_ACKN_REQ_NACK = 0x04;
constexpr uint8_t U_ACKN_REQ_BUSY = 0x02;
constexpr uint8_t U_ACKN_REQ_ADDRESSED = 0x01;

// Flags eines Telegramms. Das Byte ist API zum KNX-Stack - die Bitbelegung liegt fest.
// ACK: es liegt eine Quittung vor, egal von wem. ADDRESSED: wir haben selbst quittiert.
constexpr uint8_t TP_FRAME_FLAG_TX = 0b10000000;        // Von uns selbst gesendet
constexpr uint8_t TP_FRAME_FLAG_DATA_CON = 0b01000000;  // Mit L_Data.con beantwortet
constexpr uint8_t TP_FRAME_FLAG_FILTERED = 0b00100000;  // Soll vom Gerät gefiltert werden
constexpr uint8_t TP_FRAME_FLAG_INVALID = 0b00010000;   // Frame ist kaputt: CRC falsch, abgeschnitten oder Länge korrupt
constexpr uint8_t TP_FRAME_FLAG_ADDRESSED = 0b00001000; // Von diesem Gerät verarbeitet - wir haben quittiert
constexpr uint8_t TP_FRAME_FLAG_ACK_BUSY = 0b00000100;  // Quittung war BUSY
constexpr uint8_t TP_FRAME_FLAG_ACK_NACK = 0b00000010;  // Quittung war NACK
constexpr uint8_t TP_FRAME_FLAG_ACK = 0b00000001;       // Quittung liegt vor

// Chip-Typ der BCU - bestimmt die Baudraten, die bei der Verbindungsaufnahme probiert werden.
enum class BcuType : uint8_t
{
    Tpuart2, // Siemens 5WG1117-2AB12 "TPUart 2" - fest 19200 Baud
    Ncn5120, // OnSemi NCN5120/NCN5121/NCN5130 - 19200 oder 38400 Baud

    // KOMPAT: Schreibweise der alten Library.
    BCU_TPUART2 = Tpuart2,
    BCU_NCN5120 = Ncn5120,
};

// Verbauter Chip. Unknown: noch nicht verbunden oder nicht identifiziert.
enum class BcuChip : uint8_t
{
    Unknown = 0,
    Tpuart2,
    Ncn5120,
    Ncn5121,
    Ncn5130,
};

// Klartextname ("NCN5130"), "Unknown" wenn nicht identifiziert.
const char *bcuChipName(BcuChip chip);

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

// Quittung -> Frame-Flags. ADDRESSED setzt der Aufrufer nur bei eigener Quittung dazu.
uint8_t acknowledgeFlags(AckType acknowledge);

// Steuerdienste, geräte-/chip-spezifisch
constexpr uint8_t U_RESET_IND = 0x03;
constexpr uint8_t U_STATE_MASK = 0x07;
constexpr uint8_t U_STATE_IND = 0x07;
constexpr uint8_t U_CONFIGURE_IND = 0x01;
constexpr uint8_t U_CONFIGURE_MASK = 0x83;

// Bits im U_Configure.ind (NCN5130 Table 13).
constexpr uint8_t U_CONFIGURE_AUTO_ACKNOWLEDGE = 0x20;
constexpr uint8_t U_CONFIGURE_AUTO_POLLING = 0x10;
constexpr uint8_t U_CONFIGURE_CRC_CCITT = 0x08;
constexpr uint8_t U_CONFIGURE_MARKER = 0x04;
constexpr uint8_t U_SYSTEM_STAT_IND = 0x4B; // nur NCN5120
constexpr uint8_t U_STOP_MODE_IND = 0x2B;   // nur NCN5120
constexpr uint8_t U_FRAME_END_IND = 0xCB;
constexpr uint8_t U_FRAME_STATE_IND = 0x13;
constexpr uint8_t U_FRAME_STATE_MASK = 0x17;

// Fehlerbits im U_State.ind - je ein Ereignis seit der letzten Abfrage, kein Dauerzustand.
constexpr uint8_t U_STATE_SLAVE_COLLISION = 0x80;
constexpr uint8_t U_STATE_RECEIVE_ERROR = 0x40;
constexpr uint8_t U_STATE_TRANSMIT_ERROR = 0x20;
constexpr uint8_t U_STATE_PROTOCOL_ERROR = 0x10;
constexpr uint8_t U_STATE_TEMPERATURE_WARNING = 0x08;

// Bits im Folgebyte des U_SystemStat.ind.
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

// Klartextname eines Steuerbytes, nullptr wenn unbekannt.
const char *controlServiceName(uint8_t value);

} // namespace TPUart
