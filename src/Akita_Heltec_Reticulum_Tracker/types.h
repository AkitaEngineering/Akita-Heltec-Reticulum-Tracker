// Common types for Akita Heltec Reticulum Tracker
#pragma once

#include <stdint.h>


// Use plain enums with unique enumerator names for maximum toolchain
// compatibility (older GCCs / Arduino cores may not default to C++11).
// Values are intentionally simple integers; callers cast to `uint8_t`
// when packing into binary structures.
enum AkitaFixStatus_t {
  AKITA_FIX_NO_FIX = 0,
  AKITA_FIX_NEW_FIX = 1,
  AKITA_FIX_LAST_KNOWN_FIX = 2
};

enum AkitaLedIndicatorState_t {
  AKITA_LED_OFF = 0,
  AKITA_LED_ON,
  AKITA_LED_BLINK_SLOW,
  AKITA_LED_BLINK_GPS,
  AKITA_LED_BLINK_SUCCESS,
  AKITA_LED_BLINK_ERROR_LORA,
  AKITA_LED_BLINK_ERROR_GPS,
  AKITA_LED_BLINK_ERROR_RNS
};

// Note: use the `Akita*` canonical names in code to avoid toolchain
// aliasing or macro collisions. Do not reintroduce unscoped typedefs here.
