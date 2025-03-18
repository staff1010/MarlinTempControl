#include "timers.h"
// ------------------------
// Local defines
// ------------------------

// Default timer priorities. Override by specifying alternate priorities in the board pins file.
// The TONE timer is not present here, as it currently cannot be set programmatically. It is set
// by defining TIM_IRQ_PRIO in the variant.h or platformio.ini file, which adjusts the default
// priority for STM32 HardwareTimer objects.
#define SWSERIAL_TIMER_IRQ_PRIO_DEFAULT  1 // Requires tight bit timing to communicate reliably with TMC drivers
#define SERVO_TIMER_IRQ_PRIO_DEFAULT     1 // Requires tight PWM timing to control a BLTouch reliably
#define STEP_TIMER_IRQ_PRIO_DEFAULT      2
#define TEMP_TIMER_IRQ_PRIO_DEFAULT     14 // Low priority avoids interference with other hardware and timers

#ifndef STEP_TIMER_IRQ_PRIO
  #define STEP_TIMER_IRQ_PRIO STEP_TIMER_IRQ_PRIO_DEFAULT
#endif
#ifndef TEMP_TIMER_IRQ_PRIO
  #define TEMP_TIMER_IRQ_PRIO TEMP_TIMER_IRQ_PRIO_DEFAULT
#endif

#if defined(STM32F0xx) || defined(STM32G0xx)
  #define MCU_STEP_TIMER 16
  #define MCU_TEMP_TIMER 17
#elif defined(STM32F1xx)
  #define MCU_STEP_TIMER  4
  #define MCU_TEMP_TIMER  2
#elif defined(STM32F401xC) || defined(STM32F401xE)
  #define MCU_STEP_TIMER  9           // STM32F401 has no TIM6, TIM7, or TIM8
  #define MCU_TEMP_TIMER 10
#elif defined(STM32F4xx) || defined(STM32F7xx) || defined(STM32H7xx)
  #define MCU_STEP_TIMER  6
  #define MCU_TEMP_TIMER 14           // TIM7 is consumed by Software Serial if used.
#endif

#ifndef HAL_TIMER_RATE
  #define HAL_TIMER_RATE GetStepperTimerClkFreq()
#endif

#ifndef STEP_TIMER
  #define STEP_TIMER MCU_STEP_TIMER
#endif
#ifndef TEMP_TIMER
  #define TEMP_TIMER MCU_TEMP_TIMER
#endif

#define __TIMER_DEV(X) TIM##X
#define _TIMER_DEV(X) __TIMER_DEV(X)
#define STEP_TIMER_DEV _TIMER_DEV(STEP_TIMER)
#define TEMP_TIMER_DEV _TIMER_DEV(TEMP_TIMER)

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 2

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS] = { nullptr };

// ------------------------
// Public functions
// ------------------------

uint32_t GetStepperTimerClkFreq() {
  // Timer input clocks vary between devices, and in some cases between timers on the same device.
  // Retrieve at runtime to ensure device compatibility. Cache result to avoid repeated overhead.
  static uint32_t clkfreq = timer_instance[MF_TIMER_STEP]->getTimerClkFreq();
  return clkfreq;
}

// frequency is in Hertz
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (!HAL_timer_initialized(timer_num)) {
    switch (timer_num) {
      case MF_TIMER_STEP: // STEPPER TIMER - use a 32bit timer if possible
        timer_instance[timer_num] = new HardwareTimer(STEP_TIMER_DEV);
        /* Set the prescaler to the final desired value.
         * This will change the effective ISR callback frequency but when
         * HAL_timer_start(timer_num=0) is called in the core for the first time
         * the real frequency isn't important as long as, after boot, the ISR
         * gets called with the correct prescaler and count register. So here
         * we set the prescaler to the correct, final value and ignore the frequency
         * asked. We will call back the ISR in 1 second to start at full speed.
         *
         * The proper fix, however, would be a correct initialization OR a
         * HAL_timer_change(const uint8_t timer_num, const uint32_t frequency)
         * which changes the prescaler when an IRQ frequency change is needed
         * (for example when steppers are turned on)
         */

        timer_instance[timer_num]->setPrescaleFactor(STEPPER_TIMER_PRESCALE); //the -1 is done internally
        timer_instance[timer_num]->setOverflow(_MIN(hal_timer_t(HAL_TIMER_TYPE_MAX), (HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE) /* /frequency */), TICK_FORMAT);
        break;
      case MF_TIMER_TEMP: // TEMP TIMER - any available 16bit timer
        timer_instance[timer_num] = new HardwareTimer(TEMP_TIMER_DEV);
        // The prescale factor is computed automatically for HERTZ_FORMAT
        timer_instance[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        break;
    }

    // Disable preload. Leaving it default-enabled can cause the timer to stop if it happens
    // to exit the ISR after the start time for the next interrupt has already passed.
    timer_instance[timer_num]->setPreloadEnable(false);

    HAL_timer_enable_interrupt(timer_num);

    // Start the timer.
    timer_instance[timer_num]->resume(); // First call to resume() MUST follow the attachInterrupt()

    // This is fixed in Arduino_Core_STM32 1.8.
    // These calls can be removed and replaced with
    // timer_instance[timer_num]->setInterruptPriority
    switch (timer_num) {
      case MF_TIMER_STEP:
        timer_instance[timer_num]->setInterruptPriority(STEP_TIMER_IRQ_PRIO, 0);
        break;
      case MF_TIMER_TEMP:
        timer_instance[timer_num]->setInterruptPriority(TEMP_TIMER_IRQ_PRIO, 0);
        break;
    }
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num) && !timer_instance[timer_num]->hasInterrupt()) {
    switch (timer_num) {
      case MF_TIMER_STEP:
        timer_instance[timer_num]->attachInterrupt(Step_Handler);
        break;
      case MF_TIMER_TEMP:
        timer_instance[timer_num]->attachInterrupt(Temp_Handler);
        break;
    }
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num)) timer_instance[timer_num]->detachInterrupt();
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) && timer_instance[timer_num]->hasInterrupt();
}

void SetTimerInterruptPriorities() {
  TERN_(HAS_TMC_SW_SERIAL, SoftwareSerial::setInterruptPriority(SWSERIAL_TIMER_IRQ_PRIO, 0));
  TERN_(HAS_SERVOS, libServo::setInterruptPriority(SERVO_TIMER_IRQ_PRIO, 0));
}
