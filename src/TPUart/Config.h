/**
 * @file        Config.h
 * @brief       Build-time tunables of the TPUart library, each overridable with a -D build flag
 * @details     Set them as build flags only: several size class members, so a value differing between translation units changes the layout.
 * @copyright   Copyright (c) 2025-2026, Marco Scholl, Erkan Çolak (erkan@colak.de)
 *              Licensed under GNU GPL v3.0
 */
#pragma once
#include <climits>

// ---- Receive path ----

// Frame ring between the RX parser and process(), in octets; one entry is the frame plus 4 (2 size, 2 flags).
// Filled by the parser from process(), and from the UART IRQ on RP2040 or the UART task on ESP32.
#ifndef TPUART_RX_FRAME_BUFFER_SIZE
#define TPUART_RX_FRAME_BUFFER_SIZE 6000
#endif

// Discarded octets kept for the hex dump of showDiscardedError(); later ones are only counted (dump shows "N+").
#ifndef TPUART_DISCARD_LOG_SIZE
#define TPUART_DISCARD_LOG_SIZE 256
#endif

// Parser buffer, in octets.
#ifndef TPUART_RX_SEARCH_BUFFER_SIZE
#define TPUART_RX_SEARCH_BUFFER_SIZE 300 // Frame::size() max 9 + 255 = 264, + 2 CRC + 1 ACK/CON, rest reserve
#endif

// Frames handed to the receive callbacks per process() pass, 0 = unbounded. A frame count, not a time.
#ifndef TPUART_MAX_RXQUEUE_TIME_PER_LOOP
#define TPUART_MAX_RXQUEUE_TIME_PER_LOOP 20
#endif

// RX inter-byte gap that ends a frame, and the idle-bus quiet window, in ms.
#ifndef TPUART_RX_TIMEOUT
#define TPUART_RX_TIMEOUT 5
#endif

// Quiet window before a missing L_Data.con of our own echo is declared lost, in ms. It measures the gap since the
// last received octet; the chip repeats after at least 50 (NACK) or 150 (BUSY) bit times of bus idle (NCN5120 DS p.33).
#ifndef TPUART_TX_CON_QUIET_MS
#define TPUART_TX_CON_QUIET_MS 200
#endif

// busOperational() reports the bus down when the Nth 1 Hz state probe goes out with the N-1 before it unanswered
// (default 3: about 2-3 s after the last octet from the chip). Counted per probe sent, so a main-loop stall is not silence.
#ifndef TPUART_BUS_SILENT_PROBES
#define TPUART_BUS_SILENT_PROBES 3
#endif

// Ack age limit in us from the ack decision, assumed right after octet 6; at 2432, 120 us below 2552 are decision-latency
// reserve. No age check after write() on ESP32, ArduinoSerial, or RP2040 without the TX ring (OPENKNX_DUALCORE or no DMA).
#ifndef TPUART_ACK_WINDOW_US
#define TPUART_ACK_WINDOW_US 2432
#endif

// ---- Repetition filter ----

// Age after which a remembered frame no longer suppresses its repetition, in ms.
#ifndef TPUART_REPETITION_TTL_MS
#define TPUART_REPETITION_TTL_MS 3000
#endif

// Sources remembered at once (RepetitionFilter::MAX_SIZE).
#ifndef TPUART_REPETITION_FILTER_SIZE
#define TPUART_REPETITION_FILTER_SIZE 50
#endif

// ---- Transmit path ----

// Frames the transmit queue accepts; a compile-time limit (Transmitter::pushQueue).
#ifndef TPUART_TX_QUEUE_SIZE
#define TPUART_TX_QUEUE_SIZE 50
#endif

// Queue slots kept free of low-priority frames (rank 3) for higher-priority ones.
#ifndef TP_TX_PRIO_RESERVE
#define TP_TX_PRIO_RESERVE 6
#endif

// The TX pump in process() stops once it has run longer than this, in ms.
#ifndef TPUART_TX_PUMP_MAX_MS
#define TPUART_TX_PUMP_MAX_MS 20
#endif

// A transmission awaiting its confirm this long resets the BCU (Transmitter::processWatchdog), in ms.
#ifndef TPUART_TX_WATCHDOG_MS
#define TPUART_TX_WATCHDOG_MS 60000
#endif

// The unprefixed names stop the build with #error instead of being ignored.
#if defined(TX_BACKSTOP_MS)
#error "TX_BACKSTOP_MS is renamed to TPUART_TX_BACKSTOP_MS"
#endif
#if defined(TX_BACKSTOP_COOLDOWN_MS)
#error "TX_BACKSTOP_COOLDOWN_MS is renamed to TPUART_TX_BACKSTOP_COOLDOWN_MS"
#endif
#if defined(TX_BACKSTOP_INVALID_MS)
#error "TX_BACKSTOP_INVALID_MS is renamed to TPUART_TX_BACKSTOP_INVALID_MS"
#endif
#ifndef TPUART_TX_BACKSTOP_MS
#define TPUART_TX_BACKSTOP_MS 8000 // above the worst-case CON latency (263-octet frame, 9600 baud, 3+3 repetitions)
#endif
#ifndef TPUART_TX_BACKSTOP_COOLDOWN_MS
#define TPUART_TX_BACKSTOP_COOLDOWN_MS 30000 // spacing after a backstop reset during which neither backstop fires (TPUART_TX_WATCHDOG_MS still does)
#endif
#ifndef TPUART_TX_BACKSTOP_INVALID_MS
#define TPUART_TX_BACKSTOP_INVALID_MS 12000 // latched-_invalid on a quiet bus: longer window, give _invalid a chance to clear first
#endif

// ---- Recovery and logging ----

#ifndef TPUART_DESYNC_RECOVER_MS
#define TPUART_DESYNC_RECOVER_MS 20000 // _invalid set this long while CONNECTED resets the BCU (DataLinkLayer::processWatchdog)
#endif
#ifndef TPUART_DESYNC_COOLDOWN_MS
#define TPUART_DESYNC_COOLDOWN_MS 60000 // minimum spacing between two desync resets
#endif
#ifndef TPUART_STATE_ERR_THROTTLE_MS
#define TPUART_STATE_ERR_THROTTLE_MS 10000 // min ms between repeated "TP Error" lines of the SAME error set
#endif

// NCN thermal-warning (TW) handling.
#ifndef TPUART_TW_HEARTBEAT_MS
#define TPUART_TW_HEARTBEAT_MS 60000 // while TW stays set: at most one "still set" line this often
#endif
#ifndef TPUART_TW_AUTORESET_AFTER_MS
#define TPUART_TW_AUTORESET_AFTER_MS 45000 // TPUART_NCN_TW_AUTORESET: let TW persist this long before the heal reset
#endif
#ifndef TPUART_TW_AUTORESET_COOLDOWN_MS
#define TPUART_TW_AUTORESET_COOLDOWN_MS 300000 // minimum spacing between two auto-resets (cross-episode backoff)
#endif
#ifndef TPUART_TW_RELAPSE_MS
#define TPUART_TW_RELAPSE_MS 8000 // TW still set this long after the reset => chip is hot, not latched
#endif

// Wait for U_Reset.ind per baud-rate probe in tryInitialize(), in ms.
#ifndef TPUART_RESET_PROBE_MS
#define TPUART_RESET_PROBE_MS 50
#endif

// Wait budgets of the NCN identification (TPUART_BCU_REGISTER_INFO), in ms: worst case 50 + 4 x 20 + 50 = 180,
// once per tryInitialize(), before received octets are processed.
#ifndef NCN_STOP_MODE_TIMEOUT
#define NCN_STOP_MODE_TIMEOUT 50
#endif
#ifndef NCN_RESET_IND_TIMEOUT
#define NCN_RESET_IND_TIMEOUT 50
#endif
#ifndef NCN_REG_READ_TIMEOUT
#define NCN_REG_READ_TIMEOUT 20
#endif
#ifndef NCN_DRAIN_GUARD
#define NCN_DRAIN_GUARD 64 // bounded discard of a late reply before the next request
#endif

// ---- Interfaces ----

// RP2040 DMA RX ring: 2^exp entries (see Interface/RP2040.h).
#ifndef TPUART_RP2040_DMA_BUFFER_EXP
#define TPUART_RP2040_DMA_BUFFER_EXP 11
#endif

// RP2040 transmit ring the UART IRQ drains one octet at a time (DMA RX only, FIFOs off), in octets; a power of two.
#ifndef TPUART_RP2040_TX_RING_SIZE
#define TPUART_RP2040_TX_RING_SIZE 256
#endif

// Octets the ESP32 UART task parses at most per UART event.
#ifndef TPUART_ESP32_DRAIN_GUARD
#define TPUART_ESP32_DRAIN_GUARD 2048
#endif
#ifndef TPUART_ESP32_TASK_STACK_SIZE
#define TPUART_ESP32_TASK_STACK_SIZE 4096 // bytes; the recursive RX parser (processSearchBuffer) runs on this stack
#endif
// ESP-IDF UART driver rings in octets and its event queue length. RX must exceed the 128-octet hardware FIFO;
// TX is 0 (blocking writes) or also above 128.
#ifndef TPUART_ESP32_RX_BUFFER_SIZE
#define TPUART_ESP32_RX_BUFFER_SIZE 1024
#endif
#ifndef TPUART_ESP32_TX_BUFFER_SIZE
#define TPUART_ESP32_TX_BUFFER_SIZE 1024
#endif
#ifndef TPUART_ESP32_EVENT_QUEUE_SIZE
#define TPUART_ESP32_EVENT_QUEUE_SIZE 64
#endif

// ---- Compile-time consistency ----

// Largest parser load: Frame::size() 9 + 255 = 264 (LG 255 is the escape code, a valid frame has at most 263),
// + 2 extended-CRC octets + the trailing ACK/CON octet.
static_assert(TPUART_RX_SEARCH_BUFFER_SIZE >= 264 + 2 + 1, "TPUART_RX_SEARCH_BUFFER_SIZE cannot hold a maximum frame");
// Every entry the parser can push must fit into an empty frame ring (push needs available() > len + 4).
static_assert(TPUART_RX_FRAME_BUFFER_SIZE >= TPUART_RX_SEARCH_BUFFER_SIZE + 4 + 1, "TPUART_RX_FRAME_BUFFER_SIZE cannot hold one maximum entry");
static_assert(TPUART_DISCARD_LOG_SIZE > 0, "TPUART_DISCARD_LOG_SIZE must be positive");
// The count is 1 between each probe and its answer; 3 tolerates one lost answer.
static_assert(TPUART_BUS_SILENT_PROBES >= 3 && TPUART_BUS_SILENT_PROBES < 255, "TPUART_BUS_SILENT_PROBES must be in 3..254");
// Index mask and uint16_t indices; one free slot tells full from empty, and the largest glued block is 4 octets.
static_assert(TPUART_RP2040_TX_RING_SIZE >= 8 && TPUART_RP2040_TX_RING_SIZE <= 32768 && (TPUART_RP2040_TX_RING_SIZE & (TPUART_RP2040_TX_RING_SIZE - 1)) == 0, "TPUART_RP2040_TX_RING_SIZE must be a power of two in 8..32768");
// processRxFrameBuffer() counts the frames in a char.
static_assert(TPUART_MAX_RXQUEUE_TIME_PER_LOOP >= 0 && TPUART_MAX_RXQUEUE_TIME_PER_LOOP <= CHAR_MAX, "TPUART_MAX_RXQUEUE_TIME_PER_LOOP exceeds its char counter");
static_assert(TPUART_REPETITION_FILTER_SIZE > 0, "TPUART_REPETITION_FILTER_SIZE must be positive");
// Leaves at least one slot to low-priority frames (Transmitter kQueueLimitLow).
static_assert(TP_TX_PRIO_RESERVE >= 0 && TP_TX_PRIO_RESERVE < TPUART_TX_QUEUE_SIZE, "TP_TX_PRIO_RESERVE must leave room for low-priority frames");
// Outside 64..2552 the parked or RP2040 path cannot hold the window: below 64 every parked ack drops. 2552 = 3698 (KNX ack
// start per 03_02_02, the NCN DS gives no U_Ackn.req deadline: LG-0 frame, 41 bit times - 1 RX char) - 2 RP2040 TX chars.
static_assert(TPUART_ACK_WINDOW_US >= 64 && TPUART_ACK_WINDOW_US <= 2552, "TPUART_ACK_WINDOW_US must be in 64..2552");
// drainInterface() counts the discarded octets in a uint8_t.
static_assert(NCN_DRAIN_GUARD >= 1 && NCN_DRAIN_GUARD <= 255, "NCN_DRAIN_GUARD exceeds its uint8_t counter");
// uart_driver_install() refuses smaller rings, and ESP32.cpp does not check its return.
static_assert(TPUART_ESP32_RX_BUFFER_SIZE > 128, "TPUART_ESP32_RX_BUFFER_SIZE must exceed the 128-octet UART FIFO");
static_assert(TPUART_ESP32_TX_BUFFER_SIZE == 0 || TPUART_ESP32_TX_BUFFER_SIZE > 128, "TPUART_ESP32_TX_BUFFER_SIZE must be 0 or exceed the 128-octet UART FIFO");
