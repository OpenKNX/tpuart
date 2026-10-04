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

#ifndef TPUART_RX_SEARCH_BUFFER_SIZE
#define TPUART_RX_SEARCH_BUFFER_SIZE 300 // 263 + 37 reserv for frame errors errors and control bytes
#endif

// (TPUART_MAX_PROCESS_TIME_PER_LOOP removed: it was never referenced anywhere, so setting it did nothing.
//  Bounding the RX drain in process() was considered and deliberately NOT added -- it would trade a loop
//  stall for a UART FIFO overflow, and which of the two is worse is a measurement, not a guess.
//  TPUART_MAX_RXQUEUE_TIME_PER_LOOP below IS live, see DataLinkLayer.cpp.)
#ifndef TPUART_MAX_RXQUEUE_TIME_PER_LOOP
#define TPUART_MAX_RXQUEUE_TIME_PER_LOOP 20
#endif

// RX inter-byte gap that ends a frame, and the idle-bus quiet window, in ms.
#ifndef TPUART_RX_TIMEOUT
#define TPUART_RX_TIMEOUT 5
#endif

// Quiet window before a missing L_Data.con is declared lost. Not TPUART_RX_TIMEOUT, which guards an
// intra-frame gap: the chip's own retransmission needs 30 bit times of acknowledge window plus 50 idle
// (NACK) or 150 (BUSY) before the first retry octet, and hundreds of ms for a full 3+3 sequence.
// The 8 s TX backstop remains the outer net.
#ifndef TPUART_TX_CON_QUIET_MS
#define TPUART_TX_CON_QUIET_MS 200
#endif

// How long a parked acknowledge stays writable, past which it would land on the NEXT frame.
// 03_02_02 at 9600 baud = 104.167us per bit time. awaitDestination() is 6, so the decision falls once
// octet 5 is in = bus time 13*5+11 = 76; the acknowledge must start at 13N-2+15 = 13N+13 (2.2.7 p.32),
// leaving 13N-63 bit times. The bound is the shortest standard frame, LG=0 -> N=8 (a T_Connect to a
// tunnel address, which the chip does not auto-acknowledge): 41 bit times = 4271us, minus the 11-bit
// host-UART character that carried octet 5 (573us at the 19200 strap) = 3698us. 3000 keeps margin for
// chip forwarding and loop jitter and covers requestState() (2 bytes, ~1.1ms). It deliberately does NOT
// cover applyConfiguration() (up to ~10 bytes, ~5.7ms) -- an acknowledge parked during that init/reset
// path is discarded rather than sent late onto the following frame.
#ifndef TPUART_ACK_WINDOW_US
#define TPUART_ACK_WINDOW_US 3000
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

#ifndef TP_TX_PRIO_RESERVE
#define TP_TX_PRIO_RESERVE 6 // slots kept free of low-priority frames so a higher-priority frame (console/
                             // management telegram) is never starved by a bulk low-priority upload burst
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
#define TPUART_TX_BACKSTOP_MS 8000 // > worst-case legit CON latency (263B ext frame, 9600 baud, NCN 3+3 repeats)
#endif
#ifndef TPUART_TX_BACKSTOP_COOLDOWN_MS
#define TPUART_TX_BACKSTOP_COOLDOWN_MS 30000 // after a fast reset, only the 60s cap may fire for this long
#endif
#ifndef TPUART_TX_BACKSTOP_INVALID_MS
#define TPUART_TX_BACKSTOP_INVALID_MS 12000 // latched-_invalid on a quiet bus: longer window, give _invalid a chance to clear first
#endif

// ---- Recovery and logging ----

#ifndef TPUART_DESYNC_RECOVER_MS
#define TPUART_DESYNC_RECOVER_MS 20000 // _invalid latched this long without clearing => unrecoverable on its own
#endif
#ifndef TPUART_DESYNC_COOLDOWN_MS
#define TPUART_DESYNC_COOLDOWN_MS 60000 // minimum spacing between two desync resets
#endif
#ifndef TPUART_STATE_ERR_THROTTLE_MS
#define TPUART_STATE_ERR_THROTTLE_MS 10000 // min ms between repeated "TP Error" lines of the SAME error set
#endif

// NCN thermal-warning (TW) handling tunables. Overridable via build flags (central TPUART flag config).
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
#define TPUART_TW_RELAPSE_MS 8000 // TW still set this long after the reset => genuinely hot, not a latch
#endif

// Wait for U_Reset.ind per baud-rate probe in tryInitialize(), in ms.
#ifndef TPUART_RESET_PROBE_MS
#define TPUART_RESET_PROBE_MS 50
#endif

// Init-only busy-wait budgets for the identification. Worst case 50 + 4 x 20 + 50 = 180 ms, once per
// BCU connect, entirely before the RX pump goes live.
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

// Octets the ESP32 UART task parses at most per UART event.
#ifndef TPUART_ESP32_DRAIN_GUARD
#define TPUART_ESP32_DRAIN_GUARD 2048
#endif
#ifndef TPUART_ESP32_TASK_STACK_SIZE
#define TPUART_ESP32_TASK_STACK_SIZE 4096 // recursive RX parser (processSearchBuffer) + VLAs -- 2048 is too tight
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
// processRxFrameBuffer() counts the frames in a char.
static_assert(TPUART_MAX_RXQUEUE_TIME_PER_LOOP >= 0 && TPUART_MAX_RXQUEUE_TIME_PER_LOOP <= CHAR_MAX, "TPUART_MAX_RXQUEUE_TIME_PER_LOOP exceeds its char counter");
static_assert(TPUART_REPETITION_FILTER_SIZE > 0, "TPUART_REPETITION_FILTER_SIZE must be positive");
// Leaves at least one slot to low-priority frames (Transmitter kQueueLimitLow).
static_assert(TP_TX_PRIO_RESERVE >= 0 && TP_TX_PRIO_RESERVE < TPUART_TX_QUEUE_SIZE, "TP_TX_PRIO_RESERVE must leave room for low-priority frames");
// drainInterface() counts the discarded octets in a uint8_t.
static_assert(NCN_DRAIN_GUARD >= 1 && NCN_DRAIN_GUARD <= 255, "NCN_DRAIN_GUARD exceeds its uint8_t counter");
// uart_driver_install() refuses smaller rings, and ESP32.cpp does not check its return.
static_assert(TPUART_ESP32_RX_BUFFER_SIZE > 128, "TPUART_ESP32_RX_BUFFER_SIZE must exceed the 128-octet UART FIFO");
static_assert(TPUART_ESP32_TX_BUFFER_SIZE == 0 || TPUART_ESP32_TX_BUFFER_SIZE > 128, "TPUART_ESP32_TX_BUFFER_SIZE must be 0 or exceed the 128-octet UART FIFO");
