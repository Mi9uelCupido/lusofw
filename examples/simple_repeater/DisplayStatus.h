#pragma once
#include <stdint.h>

// LiPo discharge curve: mV → percentage (10-point lookup table with interpolation)
static inline uint8_t voltToBattPct(uint16_t mv) {
  static const uint16_t MV[]  = { 4200, 4060, 3980, 3900, 3800, 3700, 3600, 3500, 3370, 3200, 3000 };
  static const uint8_t  PCT[] = {  100,   90,   80,   70,   60,   50,   40,   30,   20,   10,    0  };
  if (mv >= MV[0])  return 100;
  if (mv <= MV[10]) return 0;
  for (int i = 0; i < 10; i++) {
    if (mv >= MV[i + 1]) {
      uint16_t range_mv  = MV[i]  - MV[i + 1];
      uint8_t  range_pct = PCT[i] - PCT[i + 1];
      return PCT[i + 1] + (uint8_t)((uint32_t)(mv - MV[i + 1]) * range_pct / range_mv);
    }
  }
  return 0;
}

// Live status filled by MyMesh every loop iteration and consumed by UITask.
// Zero-copy: MyMesh writes, UITask reads — no locking needed on single-threaded nRF52.
struct RepeaterStatus {
  uint32_t uptime_secs;
  uint32_t pkts_recv;
  uint32_t pkts_sent;
  float    last_snr;
  uint16_t batt_mv;
  uint8_t  batt_pct;         // 0-100 from LiPo curve; 255 = invalid/no battery
  float    temperature;
  int8_t   tx_power_cfg;     // configured value (_prefs.tx_power_dbm)
  int8_t   tx_power_eff;     // effective after TPC / temp / batt reductions
  int8_t   tpc_offset_dbm;   // current TPC offset (0 or negative)
  bool     batt_tx_reduced;
  bool     temp_tx_reduced;
  uint8_t  neighbour_count;
  uint32_t flood_accepted;
  uint32_t flood_rejected;
  bool     isolated;          // isolation detection triggered recently
  // ── new fields ──────────────────────────────────────────────────────────
  uint32_t pkt_rate_recv;     // packets received in the last minute
  uint32_t pkt_rate_sent;     // packets sent in the last minute
  float    duty_cycle_pct;    // TX duty cycle % over last 60-min rolling window
  uint32_t reboot_count;      // persistent boot counter (survives power cycles)
  bool     rtc_valid;
  uint8_t  rtc_hour;
  uint8_t  rtc_min;
  bool     tx_active;          // TX queue has pending packets right now
  uint32_t last_rx_secs_ago;   // seconds since last packet received
  uint8_t  hourly_dc_pct[12];  // last 12h TX duty cycle: 0-100 = 0.0-10.0%, oldest first
};
