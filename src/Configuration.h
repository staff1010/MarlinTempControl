#pragma once
//
// Heaters / Fans
//
// #define HEATER_0_PIN PA2 // HOTEND MOSFET
#define HEATER_0_PIN PC13 // HOTEND LCD Test
#if (HEATER_0_PIN == PC13)
  #define HEATER_0_INVERTING
#endif

#define HEATER_0_MAXTEMP 275


/*
 * PID
 */
#define DEFAULT_Kp  22.20
#define DEFAULT_Ki   1.08
#define DEFAULT_Kd 114.00


/**
 * Temperature Control
 *
 *  (NONE) : Bang-bang heating
 * PIDTEMP : PID temperature control (~4.1K)
 * MPCTEMP : Predictive Model temperature control. (~1.8K without auto-tune)
 */
#define PIDTEMP           // See the PID Tuning Guide at https://reprap.org/wiki/PID_Tuning
//#define MPCTEMP         // See https://marlinfw.org/docs/features/model_predictive_control.html

#define PID_MAX  255      // Limit hotend current while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_K1     0.95   // Smoothing factor within any PID loop


// put function declarations here:
// int myFunction(int, int);
#define TEMP_0_PIN PB1 // Analog Input (HOTEND thermistor)

#define TEMP_SENSOR_0 11

#define TARGET_TEMP 45

#define DEBUG