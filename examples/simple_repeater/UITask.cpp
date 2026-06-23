#include "UITask.h"
#include <Arduino.h>
#include <helpers/CommonCLI.h>
#include <math.h>

// 'meshcore', 128x13px
static const uint8_t meshcore_logo [] PROGMEM = {
    0x3c, 0x01, 0xe3, 0xff, 0xc7, 0xff, 0x8f, 0x03, 0x87, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe,
    0x3c, 0x03, 0xe3, 0xff, 0xc7, 0xff, 0x8e, 0x03, 0x8f, 0xfe, 0x3f, 0xfe, 0x1f, 0xff, 0x1f, 0xfe,
    0x3e, 0x03, 0xc3, 0xff, 0x8f, 0xff, 0x0e, 0x07, 0x8f, 0xfe, 0x7f, 0xfe, 0x1f, 0xff, 0x1f, 0xfc,
    0x3e, 0x07, 0xc7, 0x80, 0x0e, 0x00, 0x0e, 0x07, 0x9e, 0x00, 0x78, 0x0e, 0x3c, 0x0f, 0x1c, 0x00,
    0x3e, 0x0f, 0xc7, 0x80, 0x1e, 0x00, 0x0e, 0x07, 0x1e, 0x00, 0x70, 0x0e, 0x38, 0x0f, 0x3c, 0x00,
    0x7f, 0x0f, 0xc7, 0xfe, 0x1f, 0xfc, 0x1f, 0xff, 0x1c, 0x00, 0x70, 0x0e, 0x38, 0x0e, 0x3f, 0xf8,
    0x7f, 0x1f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x0e, 0x38, 0x0e, 0x3f, 0xf8,
    0x7f, 0x3f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x1e, 0x3f, 0xfe, 0x3f, 0xf0,
    0x77, 0x3b, 0x87, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xfc, 0x38, 0x00,
    0x77, 0xfb, 0x8f, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xf8, 0x38, 0x00,
    0x73, 0xf3, 0x8f, 0xff, 0x0f, 0xff, 0x1c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x78, 0x7f, 0xf8,
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfe, 0x3c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x3c, 0x7f, 0xf8,
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfc, 0x3c, 0x0e, 0x1f, 0xf8, 0xff, 0xf8, 0x70, 0x3c, 0x7f, 0xf8,
};

// ─── Helpers ────────────────────────────────────────────────────────────────

static void formatUptime(char* buf, uint32_t secs) {
  uint32_t d = secs / 86400;
  uint32_t h = (secs % 86400) / 3600;
  uint32_t m = (secs % 3600) / 60;
  if (d > 0)
    sprintf(buf, "%ud %02uh %02um", (unsigned)d, (unsigned)h, (unsigned)m);
  else
    sprintf(buf, "%uh %02um %02us", (unsigned)h, (unsigned)m, (unsigned)(secs % 60));
}

static void formatPktCount(char* buf, uint32_t n) {
  if      (n >= 1000000) sprintf(buf, "%uM", (unsigned)(n / 1000000));
  else if (n >= 1000)    sprintf(buf, "%uK", (unsigned)(n / 1000));
  else                   sprintf(buf, "%u",  (unsigned)n);
}

// ─── Status helpers ─────────────────────────────────────────────────────────

const char* UITask::txModeStr() const {
  if (!_status) return "----";
  int reductions = (_status->batt_tx_reduced ? 1 : 0) +
                   (_status->temp_tx_reduced ? 1 : 0) +
                   (_status->tpc_offset_dbm != 0 ? 1 : 0);
  if (reductions == 0)                 return "FULL";
  if (_status->batt_tx_reduced && _status->temp_tx_reduced) return "RED";
  if (_status->batt_tx_reduced)        return "BATT";
  if (_status->temp_tx_reduced)        return "HOT";
  if (_status->tpc_offset_dbm != 0)   return "TPC";
  return "RED";
}

DisplayDriver::Color UITask::txModeColor() const {
  if (!_status) return DisplayDriver::LIGHT;
  if (_status->batt_tx_reduced || _status->temp_tx_reduced) return DisplayDriver::ORANGE;
  if (_status->tpc_offset_dbm != 0) return DisplayDriver::YELLOW;
  return DisplayDriver::GREEN;
}

DisplayDriver::Color UITask::battColor() const {
  if (!_status || _status->batt_pct == 255) return DisplayDriver::LIGHT;
  if (_status->batt_pct >= 50) return DisplayDriver::GREEN;
  if (_status->batt_pct >= 20) return DisplayDriver::YELLOW;
  return DisplayDriver::RED;
}

DisplayDriver::Color UITask::tempColor() const {
  if (!_status || isnan(_status->temperature)) return DisplayDriver::LIGHT;
  if (_status->temperature < 60.0f) return DisplayDriver::GREEN;
  if (_status->temperature < 72.0f) return DisplayDriver::YELLOW;
  return DisplayDriver::RED;
}

// ─── Bottom bar: time (left) + page dots (centre) ───────────────────────────

void UITask::renderTimeAndDots() {
  // UTC time on the left (only when RTC is valid)
  if (_status && _status->rtc_valid) {
    char ttmp[8];
    snprintf(ttmp, sizeof(ttmp), "%02u:%02u", _status->rtc_hour, _status->rtc_min);
    _display->setColor(DisplayDriver::LIGHT);
    _display->setTextSize(1);
    _display->setCursor(0, 57);
    _display->print(ttmp);
  }

  // Page dots centred
  const int dot_w = 6, dot_h = 5, gap = 4;
  int total = DISPLAY_PAGES * dot_w + (DISPLAY_PAGES - 1) * gap;
  int sx = (128 - total) / 2;
  for (int i = 0; i < DISPLAY_PAGES; i++) {
    int x = sx + i * (dot_w + gap);
    _display->setColor(DisplayDriver::LIGHT);
    if (i == _page) {
      _display->fillRect(x, 57, dot_w, dot_h);
    } else {
      _display->drawRect(x, 57, dot_w, dot_h);
    }
  }
}

// ─── Page 0: Radio config ────────────────────────────────────────────────────

void UITask::renderPage0() {
  char tmp[48];
  _display->setTextSize(1);

  // Node name (left) — ellipsized if too long
  _display->setColor(DisplayDriver::GREEN);
  _display->drawTextEllipsized(0, 0, 110, _node_prefs->node_name);

  // Frequency + SF
  _display->setColor(DisplayDriver::YELLOW);
  snprintf(tmp, sizeof(tmp), "%.3f MHz  SF%d", _node_prefs->freq, _node_prefs->sf);
  _display->setCursor(0, 12);
  _display->print(tmp);

  // BW + CR
  snprintf(tmp, sizeof(tmp), "BW:%.1f  CR:%d", _node_prefs->bw, _node_prefs->cr);
  _display->setCursor(0, 23);
  _display->print(tmp);

  // Effective TX power + mode
  if (_status) {
    _display->setColor(txModeColor());
    snprintf(tmp, sizeof(tmp), "TX: %ddBm  [%s]", _status->tx_power_eff, txModeStr());
    _display->setCursor(0, 34);
    _display->print(tmp);

    // Reboot counter (right-aligned, same line as version)
    _display->setColor(DisplayDriver::LIGHT);
    snprintf(tmp, sizeof(tmp), "Rst:%lu", _status->reboot_count);
    _display->drawTextRightAlign(128, 46, tmp);
  }

  // lusofw version
  _display->setColor(DisplayDriver::LIGHT);
  snprintf(tmp, sizeof(tmp), "lusofw %s", _version_info);
  _display->drawTextEllipsized(0, 46, 90, tmp);

  renderTimeAndDots();
}

// ─── Page 1: Network stats ───────────────────────────────────────────────────

void UITask::renderPage1() {
  char tmp[48], ra[10], ta[10];
  _display->setTextSize(1);

  if (!_status) {
    _display->setColor(DisplayDriver::LIGHT);
    _display->setCursor(0, 20);
    _display->print("No data");
    renderTimeAndDots();
    return;
  }

  // Packets RX / TX + receive rate
  _display->setColor(DisplayDriver::LIGHT);
  formatPktCount(ra, _status->pkts_recv);
  formatPktCount(ta, _status->pkts_sent);
  snprintf(tmp, sizeof(tmp), "RX:%-5s TX:%-5s %lu/m",
           ra, ta, _status->pkt_rate_recv);
  _display->setCursor(0, 0);
  _display->print(tmp);

  // Neighbours + last SNR
  snprintf(tmp, sizeof(tmp), "Nbrs:%-3d  SNR:%+.1fdB",
           _status->neighbour_count, _status->last_snr);
  _display->setCursor(0, 12);
  _display->print(tmp);

  // Uptime + duty cycle
  char upt[20];
  formatUptime(upt, _status->uptime_secs);
  if (_status->duty_cycle_pct > 0.0f) {
    DisplayDriver::Color dc_color = (_status->duty_cycle_pct < 8.0f)
                                    ? DisplayDriver::GREEN : DisplayDriver::ORANGE;
    _display->setColor(DisplayDriver::LIGHT);
    snprintf(tmp, sizeof(tmp), "Up:%s", upt);
    _display->setCursor(0, 23);
    _display->print(tmp);
    _display->setColor(dc_color);
    snprintf(tmp, sizeof(tmp), "DC:%.1f%%", _status->duty_cycle_pct);
    _display->drawTextRightAlign(128, 23, tmp);
  } else {
    _display->setColor(DisplayDriver::LIGHT);
    snprintf(tmp, sizeof(tmp), "Up: %s", upt);
    _display->setCursor(0, 23);
    _display->print(tmp);
  }

  // Flood filter efficiency
  uint32_t total_ff = _status->flood_accepted + _status->flood_rejected;
  if (total_ff > 0) {
    uint8_t pct = (uint8_t)(_status->flood_rejected * 100 / total_ff);
    snprintf(tmp, sizeof(tmp), "FF:+%lu -%lu (%u%%)",
             _status->flood_accepted, _status->flood_rejected, pct);
    _display->setColor(pct > 80 ? DisplayDriver::ORANGE : DisplayDriver::LIGHT);
  } else {
    snprintf(tmp, sizeof(tmp), "FF: no data yet");
    _display->setColor(DisplayDriver::LIGHT);
  }
  _display->setCursor(0, 34);
  _display->print(tmp);

  renderTimeAndDots();
}

// ─── Page 2: Health / diagnostics ───────────────────────────────────────────

void UITask::renderPage2() {
  char tmp[48];
  _display->setTextSize(1);

  if (!_status) {
    _display->setColor(DisplayDriver::LIGHT);
    _display->setCursor(0, 20);
    _display->print("No data");
    renderTimeAndDots();
    return;
  }

  // Battery voltage + percentage + status
  _display->setColor(battColor());
  if (_status->batt_mv >= 1000 && _status->batt_pct != 255) {
    float v   = _status->batt_mv / 1000.0f;
    uint8_t p = _status->batt_pct;
    const char* bst = (p >= 50) ? "OK" : (p >= 20) ? "LOW" : "CRIT";
    snprintf(tmp, sizeof(tmp), "Batt: %.2fV %3u%%  [%s]", v, p, bst);
  } else {
    snprintf(tmp, sizeof(tmp), "Batt: ---  [N/A]");
  }
  _display->setCursor(0, 0);
  _display->print(tmp);

  // Battery bar (12px tall, full width 128px)
  if (_status->batt_pct != 255) {
    uint8_t p = _status->batt_pct;
    int bar_w = (int)(p * 128 / 100);
    // Background (empty bar outline)
    _display->setColor(DisplayDriver::LIGHT);
    _display->drawRect(0, 9, 128, 3);
    // Fill (coloured)
    _display->setColor(battColor());
    if (bar_w > 2) _display->fillRect(1, 10, bar_w - 2, 1);
  }

  // MCU temperature
  _display->setColor(tempColor());
  if (!isnan(_status->temperature) && _status->temperature > -40.0f) {
    const char* tst = (_status->temperature < 60.0f) ? "OK" :
                      (_status->temperature < 72.0f) ? "HOT" : "CRIT";
    snprintf(tmp, sizeof(tmp), "Temp: %.1fC  [%s]", _status->temperature, tst);
  } else {
    snprintf(tmp, sizeof(tmp), "Temp: ---  [N/A]");
  }
  _display->setCursor(0, 15);
  _display->print(tmp);

  // Effective TX power + mode
  _display->setColor(txModeColor());
  snprintf(tmp, sizeof(tmp), "TX: %ddBm  [%s]", _status->tx_power_eff, txModeStr());
  _display->setCursor(0, 27);
  _display->print(tmp);

  // Isolation warning or healthy status
  if (_status->isolated) {
    _display->setColor(DisplayDriver::RED);
    _display->setCursor(0, 39);
    _display->print("! ISOLATED - no traffic");
  } else {
    _display->setColor(DisplayDriver::GREEN);
    _display->setCursor(0, 39);
    _display->print("Mesh: OK");
  }

  renderTimeAndDots();
}

// ─── begin / loop ───────────────────────────────────────────────────────────

void UITask::begin(NodePrefs* node_prefs, const char* build_date, const char* firmware_version) {
  _node_prefs = node_prefs;
  _prevBtnState = HIGH;
  _auto_off = millis() + AUTO_OFF_MILLIS;

  // Store version, strip git hash suffix (e.g. "v1.15.0-abc123" → "v1.15.0")
  char ver[24];
  strncpy(ver, firmware_version, sizeof(ver) - 1);
  ver[sizeof(ver) - 1] = 0;
  char* dash = strchr(ver, '-');
  if (dash) *dash = 0;
  snprintf(_version_info, sizeof(_version_info), "%s", ver);

  _display->turnOn();

  // Boot screen
  _display->startFrame();
  _display->setColor(DisplayDriver::BLUE);
  _display->drawXbm(0, 3, meshcore_logo, 128, 13);
  _display->setColor(DisplayDriver::LIGHT);
  _display->setTextSize(1);
  _display->drawTextCentered(64, 22, _version_info);
  _display->setColor(DisplayDriver::GREEN);
  _display->drawTextCentered(64, 35, "< Repeater >");
  _display->endFrame();
}

void UITask::loop() {
#ifdef PIN_USER_BTN
  if (millis() >= _next_read) {
    int btnState = digitalRead(PIN_USER_BTN);
    if (btnState != _prevBtnState) {
      if (btnState == USER_BTN_PRESSED) {
        if (!_display->isOn()) {
          // Display was off: wake it up, stay on current page
          _display->turnOn();
        } else {
          // Display was on: advance to next page
          _page = (_page + 1) % DISPLAY_PAGES;
          // Pause auto-rotate so user can read the page they chose
          _manual_until = millis() + MANUAL_PAUSE_MILLIS;
        }
        // Any button press resets the auto-off timer
        _auto_off = millis() + AUTO_OFF_MILLIS;
      }
      _prevBtnState = btnState;
    }
    _next_read = millis() + 50; // poll at 20 Hz for snappy response
  }
#endif

  if (_display->isOn()) {
    // Auto-rotate pages (paused after manual button press)
    if (millis() > _manual_until && millis() >= _next_rotate) {
      _page = (_page + 1) % DISPLAY_PAGES;
      _next_rotate = millis() + AUTO_ROTATE_MILLIS;
    }

    // Render at ~4 Hz
    if (millis() >= _next_refresh) {
      _display->startFrame();
      _display->setTextSize(1);

      if (millis() < BOOT_SCREEN_MILLIS) {
        // Keep boot screen until BOOT_SCREEN_MILLIS has elapsed
        _display->setColor(DisplayDriver::BLUE);
        _display->drawXbm(0, 3, meshcore_logo, 128, 13);
        _display->setColor(DisplayDriver::LIGHT);
        _display->drawTextCentered(64, 22, _version_info);
        _display->setColor(DisplayDriver::GREEN);
        _display->drawTextCentered(64, 35, "< Repeater >");
      } else {
        switch (_page) {
          case 0: renderPage0(); break;
          case 1: renderPage1(); break;
          case 2: renderPage2(); break;
          default: _page = 0; renderPage0(); break;
        }
      }

      _display->endFrame();
      _next_refresh = millis() + 250; // 4 Hz refresh
    }

    // Auto-off
    if (millis() > _auto_off) {
      _display->turnOff();
    }
  }
}
