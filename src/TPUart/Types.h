#pragma once
// services Host -> Controller :
// internal commands, device specific
#define U_RESET_REQ 0x01
#define U_STATE_REQ 0x02
#define U_BUSMON_REQ 0x05
#define U_L_DATA_OFFSET_REQ 0x08 //-0x0C
#define U_SYSTEM_STATE_REQ 0x0D
#define U_STOP_MODE_REQ 0x0E
#define U_EXIT_STOP_MODE_REQ 0x0F
#define U_ACK_REQ 0x10 //-0x17
#define U_ACK_REQ_NACK 0x04
#define U_ACK_REQ_BUSY 0x02
#define U_ACK_REQ_ADDRESSED 0x01
#define U_POLLING_STATE_REQ 0xE0

// Only on NCN51xx available
#define U_NCN5120_CONFIGURE_REQ 0x18
#define U_NCN5120_CONFIGURE_MARKER_REQ 0x1
#define U_NCN5120_CONFIGURE_CRC_CCITT_REQ 0x2
#define U_NCN5120_CONFIGURE_AUTO_POLLING_REQ 0x4
#define U_NCN5120_SET_REPETITION_REQ 0xF2
#define U_NCN5120_SET_ADDRESS_REQ 0xF1
#define U_NCN5120_SET_BUSY_REQ 0x03
#define U_NCN5120_QUIT_BUSY_REQ 0x04
// Only on TPUart2
#define U_TPUART2_SET_REPETITION_REQ 0x24
#define U_TPUART2_ACTIVATECRC_REQ 0x25
#define U_TPUART2_SET_ADDRESS_REQ 0x28
#define U_TPUART2_SET_BUSY_REQ 0x21
#define U_TPUART2_QUIT_BUSY_REQ 0x22

// knx transmit data commands
#define U_L_DATA_START_REQ 0x80
#define U_L_DATA_CONT_REQ 0x80 //-0xBF
#define U_L_DATA_END_REQ 0x40  //-0x7F

// serices to host controller

// DLL services (device is transparent)
#define L_DATA_STANDARD_IND 0x90
#define L_DATA_EXTENDED_IND 0x10
#define L_DATA_MASK 0xD3
#define L_POLL_DATA_IND 0xF0

// acknowledge services (device is transparent in bus monitor mode)
#define L_ACKN_IND 0x00
#define L_ACKN_MASK 0x33
#define L_ACKN_BUSY_MASK 0x0C
#define L_ACKN_NACK_MASK 0xC0
#define L_DATA_CON 0x0B
#define L_DATA_CON_MASK 0x7F

// control services, device specific
#define U_RESET_IND 0x03
#define U_STATE_MASK 0x07
#define U_STATE_IND 0x07
#define SLAVE_COLLISION 0x80
#define RECEIVE_ERROR 0x40
#define TRANSMIT_ERROR 0x20
#define PROTOCOL_ERROR 0x10
#define TEMPERATURE_WARNING 0x08
#define U_FRAME_STATE_IND 0x13
#define U_FRAME_STATE_MASK 0x17
#define PARITY_BIT_ERROR 0x80
#define CHECKSUM_LENGTH_ERROR 0x40
#define TIMING_ERROR 0x20
#define U_CONFIGURE_IND 0x01
#define U_CONFIGURE_MASK 0x83
#define AUTO_ACKNOWLEDGE 0x20
#define AUTO_POLLING 0x10
#define CRC_CCITT 0x08
#define FRAME_END_WITH_MARKER 0x40
#define U_FRAME_END_IND 0xCB
#define U_STOP_MODE_IND 0x2B
#define U_SYSTEM_STAT_IND 0x4B

/*
 * NCN51xx Register handling
 */
// write internal registers
#define U_INT_REG_WR_REQ_WD 0x28
#define U_INT_REG_WR_REQ_ACR0 0x29
#define U_INT_REG_WR_REQ_ACR1 0x2A
#define U_INT_REG_WR_REQ_ASR0 0x2B
// read internal registers
#define U_INT_REG_RD_REQ_WD 0x38
#define U_INT_REG_RD_REQ_ACR0 0x39
#define U_INT_REG_RD_REQ_ACR1 0x3A
#define U_INT_REG_RD_REQ_ASR0 0x3B
// Revision ID register (0x05): opcode 0x38|0x05. Present on NCN5121/NCN5130 only (NCN5120 has no 0x05).
// NCN5130/D Rev.8 p.56 Table 22/23 (Revision ID Register): [7:5] silicon rev, [4:0] part number.
#define U_INT_REG_RD_REQ_RID 0x3D
// Analog Status Register 0 (ASR0, reg 0x03) bit masks: [6]V20V [5]VDD2 [4]VBUS [3]VFILT [2]XTAL [1]TW [0]TSD.
// NCN5130/D Rev.8 p.56 Table 20/21 (Analog Status Register); bit 7 reserved.
#define ASR0_V20V 0x40
#define ASR0_VDD2 0x20
#define ASR0_VBUS 0x10
#define ASR0_VFILT 0x08
#define ASR0_XTAL 0x04
#define ASR0_TW 0x02
#define ASR0_TSD 0x01
// Analog Control Register 0 - Bit values
#define ACR0_FLAG_V20VEN 0x40
#define ACR0_FLAG_DC2EN 0x20
#define ACR0_FLAG_XCLKEN 0x10
#define ACR0_FLAG_TRIGEN 0x08
#define ACR0_FLAG_V20VCLIMIT 0x04

/*
 * TPUART_BUSMON_INTEGRITY (opt-in, define it in the product's build flags)
 *
 * In bus monitor mode the chip is transparent: no U_FrameEnd.ind, no CRC, no byte stuffing (DS Table 11 +
 * p.36), so every gap in the capture has to be detected and reported host-side. With this defined:
 *   - a truncated telegram is forwarded to the monitor instead of being swept into the discard ring,
 *   - a poll telegram is counted out and forwarded instead of being swept byte by byte,
 *   - the chip's per-octet bit-error flag is collected (NCN 9-bit UART parity encoding, DS p.27).
 * They fill the cEMI busmonitor Lost and Bit-error flags (03_06_03 4.1.5.8.1). Platforms without a
 * per-octet error channel (ArduinoSerial) simply never set the bit-error flag.
 */

namespace TPUart
{
    typedef enum
    {
        BCU_TPUART2, // SIEMENS 5WG1117-2AB12 TPUart 2
        BCU_NCN5120  // OnSemi NCN5120, NCN5121, NCN5130
    } BcuType;

    typedef enum
    {
        ACK_None = 0x0,
        ACK_Addressed = U_ACK_REQ_ADDRESSED,
        ACK_Busy = U_ACK_REQ_ADDRESSED | U_ACK_REQ_BUSY,
        ACK_Nack = U_ACK_REQ_ADDRESSED | U_ACK_REQ_NACK,
    } AcknowledgeType;

    typedef enum
    {
        BCU_UNINITIALIZED,
        BCU_CONNECTED,
        BCU_DISCONNECTED,
        BCU_BUSMONITOR,
    } BcuState;

    typedef enum
    {
        TX_IDLE,
        TX_TRANSMIT,
        TX_AWAIT
    } TxState;

    typedef enum
    {
        RX_IDLE,
        RX_FRAME,
        RX_FRAME_DESTINATION,
        RX_FRAME_SIZE,
        RX_FRAME_COMPLETE,
        RX_FRAME_WAIT_ACKN,
    } RxState;
} // namespace TPUart