#pragma once

#include <helpers/ui/DisplayDriver.h>
#include <helpers/CommonCLI.h>
#include "DisplayStatus.h"

#ifndef USER_BTN_PRESSED
  #define USER_BTN_PRESSED LOW
#endif

#define DISPLAY_PAGES           3
#define AUTO_OFF_MILLIS         30000   // display turns off after 30s of inactivity
#define AUTO_ROTATE_MILLIS       5000   // auto-advance page every 5s
#define MANUAL_PAUSE_MILLIS     10000   // pause auto-rotate 10s after button press
#define BOOT_SCREEN_MILLIS       4000   // boot logo shown for 4s

class UITask {
  DisplayDriver*  _display;
  NodePrefs*      _node_prefs;
  RepeaterStatus* _status;

  unsigned long _next_read;
  unsigned long _next_refresh;
  unsigned long _auto_off;
  unsigned long _next_rotate;
  unsigned long _manual_until;  // auto-rotate paused until this timestamp

  uint8_t _page;
  int     _prevBtnState;
  char    _version_info[40];

  void renderPage0();       // Radio config
  void renderPage1();       // Network stats
  void renderPage2();       // Health / diagnostics
  void renderTimeAndDots(); // Bottom bar: UTC time + page indicator dots

  const char*          txModeStr()   const;
  DisplayDriver::Color txModeColor() const;
  DisplayDriver::Color battColor()   const;
  DisplayDriver::Color tempColor()   const;

public:
  UITask(DisplayDriver& display, RepeaterStatus* status)
    : _display(&display), _status(status), _page(0),
      _prevBtnState(HIGH),
      _next_read(0), _next_refresh(0), _auto_off(0),
      _next_rotate(0), _manual_until(0) { }

  void begin(NodePrefs* node_prefs, const char* build_date, const char* firmware_version);
  void loop();
};
