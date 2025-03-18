#include <Arduino.h>
#include "main.h"
#include "temperature.h"


HardwareSerial Serial1(PA10, PA9);  //将串口1的管脚指定到PA10（RX），PA9(TX)引脚上


// #define LED_TASK
#ifdef LED_TASK
  void led_init() {
    OUT_WRITE(PC13, true);
  }
  void led_task() {
    OUT_WRITE(PC13, false);
  }
#endif



// Pass true to keep steppers from timing out
void idle(const bool no_stepper_sleep=false);
/**
 * Standard idle routine keeps the machine alive:
 *  - Core Marlin activities
 *  - Manage heaters (and Watchdog)
 *  - Max7219 heartbeat, animation, etc.
 *
 *  Only after setup() is complete:
 *  - Handle filament runout sensors
 *  - Run HAL idle tasks
 *  - Handle Power-Loss Recovery
 *  - Run StallGuard endstop checks
 *  - Handle SD Card insert / remove
 *  - Handle USB Flash Drive insert / remove
 *  - Announce Host Keepalive state (if any)
 *  - Update the Print Job Timer state
 *  - Update the Beeper queue
 *  - Read Buttons and Update the LCD
 *  - Run i2c Position Encoders
 *  - Auto-report Temperatures / SD Status
 *  - Update the Průša MMU2
 *  - Handle Joystick jogging
 */
void idle(const bool no_stepper_sleep/*=false*/) {
  // Manage Heaters (and Watchdog)
  temperature_task();
  // Serial1.print("temp_hotend[0] : ");
  // Serial1.println(temp_hotend[0].getraw());
  // Serial1.println(temp_hotend[0].celsius);
  #ifdef LED_TASK
    led_task();
  #endif
}

void setup() {
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);

  Serial1.begin(115200);  //打开串口，波特率为115200

  #ifdef FASTIO_INIT
    FASTIO_INIT();
  #endif

  temperature_init();
  
  #ifdef LED_TASK
    led_init();
  #endif



}

void loop() {
  // put your main code here, to run repeatedly:
  idle();
  // Serial1.println("This is a test message !");
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }