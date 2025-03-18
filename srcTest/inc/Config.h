#pragma once

//
// Prefix header for all sources
//

#include "ConfigPre.h"

// #ifndef __MARLIN_DEPS__
  #include "../HAL/HAL.h"
// #endif

#include "../pins/stm32f1/pins_stm32f103c8_common.h"
#include "../core/macros.h"

// #ifndef __MARLIN_DEPS__
  #include HAL_PATH(.., timers.h)
//   #include HAL_PATH(.., spi_pins.h)
// #endif

// #include "Conditionals_post.h"

// #ifndef __MARLIN_DEPS__

//   #include HAL_PATH(.., inc/Conditionals_post.h)

  #include "../core/types.h"  // Ahead of sanity-checks

//   #include "SanityCheck.h"
//   #include HAL_PATH(.., inc/SanityCheck.h)

  // Include all core headers
//   #include "../core/language.h"
//   #include "../core/utility.h"
//   #include "../core/serial.h"

// #endif

// #include "../core/multi_language.h"
