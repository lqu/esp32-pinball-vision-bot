#pragma once
#include <Arduino.h>

// Put types in a header so Arduino's auto-prototype generation sees them first.

enum FlipperSide : uint8_t { FLIPPER_LEFT = 0, FLIPPER_RIGHT = 1 };

static inline const char* sideName(FlipperSide s) { return (s == FLIPPER_LEFT) ? "L" : "R"; }

struct ROI {
  int x=0, y=0, w=0, h=0;
};

struct Zone {
  bool enabled = false;
  ROI roi;
  FlipperSide side = FLIPPER_LEFT;
  uint32_t tap_ms = 120;
  uint32_t last_fire_ms = 0;
};
