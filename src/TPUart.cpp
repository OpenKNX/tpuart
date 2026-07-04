#include "TPUart.h"

// Build marker: if you see this during compile, the LOCAL lib/TPUart clone is being used
// (not the version pulled by knx/library.json). Remove once the local setup is confirmed.
#pragma message("==================  TPUart: LOCAL lib/TPUart clone in use  ==================")

// Performance switch: drain the whole UART RX ring per wakeup instead of 1 byte/event
// (fewer task wakeups, no ~1000 B/s drain ceiling). Default off = stock behaviour.
#ifdef TPUART_RX_DRAIN_FAST
#pragma message(">>>  TPUart: RX DRAIN FAST = ON  <<<")
#endif

// Self-healing BCU reconnect (auto "bcu rst" when the NCN goes mute/desynced). Default off = stock.
#ifdef TPUART_BCU_AUTORECONNECT
#pragma message(">>>  TPUart: BCU AUTO-RECONNECT = ON  <<<")
#endif

// TX throughput: taskYIELD instead of vTaskDelay(1) in the TX byte-pump + pop next queued frame in the
// same loop pass after the CON. Raises the IP->TP forward rate toward the 9600-baud floor. Default off.
#ifdef TPUART_TX_FAST
#pragma message(">>>  TPUart: TX FAST = ON  <<<")
#endif

// Fast-reset backstop for the residual transient lost-CON: cuts the 60s TX-stall to ~few s
// when the receiver is healthy-idle but the TX is stranded in TX_AWAIT. Default off = stock 60s cap.
#ifdef TPUART_BCU_BACKSTOP
#pragma message(">>>  TPUart: BCU BACKSTOP = ON  <<<")
#endif

// BCU health counters (resets/disconnects/con-rescues/temp-warn/SC-RE-TE-PE) shown in the 'bcu' report. Default off.
#ifdef TPUART_BCU_HEALTH
#pragma message(">>>  TPUart: BCU HEALTH = ON  <<<")
#endif

// Test/diagnosis only: console "bcu dis" hook + watchdog/CON-rescue traces. Default off.
#ifdef TPUART_BCU_DEBUG
#pragma message(">>>  TPUart: BCU DEBUG = ON  <<<")
#endif
