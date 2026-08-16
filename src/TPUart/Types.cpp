#include "TPUart/Types.h"

namespace TPUart
{

// Siehe den Kommentar an der Deklaration - die Tabelle dreht die Rohwerte auf die Rangfolge, Normal und
// Urgent tauschen dabei die Plätze.
uint8_t telegramPriorityRank(uint8_t control)
{
    static const uint8_t rank[TP_PRIORITY_COUNT] = {0, 2, 1, 3};

    return rank[(control >> 2) & 0x03];
}

uint8_t acknowledgeFlags(AckType acknowledge)
{
    switch (acknowledge)
    {
        case AckType::Addressed:
            return TP_FRAME_FLAG_ACK;
        case AckType::Busy:
            return TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_BUSY;
        case AckType::Nack:
            return TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_NACK;
        default:
            return 0;
    }
}

// Die Reihenfolge ist wichtig - die Maskendienste sind unspezifischer als die festen Werte darüber und
// würden diese sonst mit abdecken.
const char *controlServiceName(uint8_t value)
{
    if (value == U_RESET_IND) return "U_Reset.ind";
    if (value == U_SYSTEM_STAT_IND) return "U_SystemStat.ind";
    if (value == U_STOP_MODE_IND) return "U_StopMode.ind";
    if (value == U_FRAME_END_IND) return "U_FrameEnd.ind";

    // Kein Steuerdienst, sondern der Anfang eines Poll-Telegramms - hier steht es trotzdem, weil der
    // Eintrag denselben Weg nimmt (Frame::isFrame() ist für 0xF0 falsch). Ohne diesen Namen meldete
    // handleControlEntry() ein sauber erkanntes Poll als "Unknown control byte F0", also als Fehler.
    if (value == L_POLL_DATA_IND) return "L_Poll_Data.ind";
    if ((value & U_STATE_MASK) == U_STATE_IND) return "U_State.ind";
    if ((value & U_FRAME_STATE_MASK) == U_FRAME_STATE_IND) return "U_FrameState.ind";
    if ((value & U_CONFIGURE_MASK) == U_CONFIGURE_IND) return "U_Configure.ind";
    if ((value & L_DATA_CON_MASK) == L_DATA_CON) return "L_Data.con";
    if ((value & L_ACKN_MASK) == L_ACKN_IND) return "L_Ackn.ind";

    return nullptr;
}

} // namespace TPUart
