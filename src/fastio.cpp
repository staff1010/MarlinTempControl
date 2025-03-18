// #include "../platforms.h"
#include "fastio.h"
#include "pins_arduino.h"  // 这里包含了 LastPort 的定义
#include "variant.h"       // 某些情况下可能需要这个

#ifdef HAL_STM32

// #include "../../inc/MarlinConfig.h"

GPIO_TypeDef* FastIOPortMap[LastPort + 1] = { 0 };

void FastIO_init() {
  for (uint8_t i = 0; i < NUM_DIGITAL_PINS; ++i)
    FastIOPortMap[STM_PORT(digitalPin[i])] = get_GPIO_Port(STM_PORT(digitalPin[i]));
}

#endif // HAL_STM32
