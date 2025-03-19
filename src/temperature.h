#pragma once
#include "fastio.h"
#include <Arduino.h>
#include "macros.h"
#include "timers.h"
#include "millis_t.h"
// #include "main.h"
#include "Configuration.h"


void temperature_reset();
void temperature_init();
void temperature_task();

#if TEMP_SENSOR_0 > 0
  #define TEMP_SENSOR_0_IS_THERMISTOR 1
#endif

#if ANY(TEMP_SENSOR_0_IS_THERMISTOR, TEMP_SENSOR_1_IS_THERMISTOR, TEMP_SENSOR_2_IS_THERMISTOR, TEMP_SENSOR_3_IS_THERMISTOR, \
  TEMP_SENSOR_4_IS_THERMISTOR, TEMP_SENSOR_5_IS_THERMISTOR, TEMP_SENSOR_6_IS_THERMISTOR, TEMP_SENSOR_7_IS_THERMISTOR )
#define HAS_HOTEND_THERMISTOR 1
#endif


// PID heating
#if ANY(PIDTEMP, PIDTEMPBED, PIDTEMPCHAMBER)
  #define HAS_PID_HEATING 1
#endif

/**
 * States for ADC reading in the ISR
 */
enum ADCSensorState : char
{
    StartSampling,
    PrepareTemp_0,
    MeasureTemp_0,
    SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
    StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

// Minimum number of Temperature::ISR loops between sensor readings.
// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
// get all oversampled sensor readings
#define MIN_ADC_ISR_LOOPS 10

#define ACTUAL_ADC_SAMPLES _MAX(int(MIN_ADC_ISR_LOOPS), int(SensorsReady))

//
// celsius_t is the native unit of temperature. Signed to handle a disconnected thermistor value (-14).
// For more resolition (e.g., for a chocolate printer) this may later be changed to Celsius x 100
//
typedef uint16_t raw_adc_t;
typedef int16_t celsius_t;
typedef float celsius_float_t;

// A temperature sensor
typedef struct TempInfo
{
private:
    raw_adc_t acc;
    raw_adc_t raw;

public:
    celsius_float_t celsius;
    inline void reset() { acc = 0; }
    inline void sample(const raw_adc_t s) { acc += s; }
    inline void update() { raw = acc; }
    void setraw(const raw_adc_t r) { raw = r; }
    raw_adc_t getraw() const { return raw; }
} temp_info_t;

// typedef struct TemperatureClass
// {
//     ;
// } Temperature;


// A PWM heater with temperature sensor
typedef struct HeaterInfo : public TempInfo {
    celsius_t target;
    uint8_t soft_pwm_amount;
    bool is_below_target(const celsius_t offs=0) const { return (target - celsius > offs); } // celsius < target - offs
    bool is_above_target(const celsius_t offs=0) const { return (celsius - target > offs); } // celsius > target + offs
  } heater_info_t;

// A heater with PID stabilization
template<typename T>
struct PIDHeaterInfo : public HeaterInfo {
  T pid;  // Initialized by settings.load
};

#define HOTENDS 1
// At least one hotend...
#if HOTENDS
  #define HAS_HOTEND 1
  #ifndef HOTEND_OVERSHOOT
    #define HOTEND_OVERSHOOT 15
  #endif
#endif

#define _HOTEND_LOOP(H) for (int8_t H = 0; H < HOTENDS; H++)
#define HOTEND_LOOP() _HOTEND_LOOP(e)


#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders

static volatile bool temperature_raw_temps_ready  = false;
static void temperature_updateTemperaturesFromRawValues();
static bool temperature_updateTemperaturesIfReady() {
  if (!temperature_raw_temps_ready) return false;
  temperature_updateTemperaturesFromRawValues();
  temperature_raw_temps_ready = false;
  return true;
}



#define ARRAY_BY_EXTRUDERS(V...) ARRAY_N(EXTRUDERS, V)
#define ARRAY_BY_EXTRUDERS1(v1) ARRAY_N_1(EXTRUDERS, v1)
#define ARRAY_BY_HOTENDS(V...) ARRAY_N(HOTENDS, V)
#define ARRAY_BY_HOTENDS1(v1) ARRAY_N_1(HOTENDS, v1)

#define _SENSOR_IS(I,N) || (TEMP_SENSOR(N) == I)
#define _E_SENSOR_IS(I,N) _SENSOR_IS(N,I)
#define ANY_E_SENSOR_IS(N) (0 REPEAT2(HOTENDS, _E_SENSOR_IS, N))
#define ANY_THERMISTOR_IS(N) ( ANY_E_SENSOR_IS(N) _SENSOR_IS(N,REDUNDANT) \
  _SENSOR_IS(N,BED) _SENSOR_IS(N,PROBE) _SENSOR_IS(N,CHAMBER) _SENSOR_IS(N,COOLER) _SENSOR_IS(N,BOARD) )

// ------------------------
// ADC
// ------------------------
// 再ini文件中定义build_flags -DADC_RESOLUTION=12
#ifdef ADC_RESOLUTION
  #define HAL_ADC_RESOLUTION ADC_RESOLUTION
#else
  #define HAL_ADC_RESOLUTION 12
#endif

#define HAL_ADC_VREF_MV   3300

#define HAL_ADC_RANGE _BV(HAL_ADC_RESOLUTION)
// 把温度表的分辨率(10位)转化为ADC读出的分辨率值(12位)
#define THERMISTOR_TABLE_ADC_RESOLUTION 10
#define THERMISTOR_TABLE_SCALE (HAL_ADC_RANGE / _BV(THERMISTOR_TABLE_ADC_RESOLUTION))

// 把读出的raw值(16倍）转换为10位分辨率的温度表
#define THERMISTOR_RAW_SCALE (RECIPROCAL(OVERSAMPLENR) * _BV(THERMISTOR_TABLE_ADC_RESOLUTION) / HAL_ADC_RANGE)

// 采样次数，注意跟分辨率不是一个概念
#if ENABLED(HAL_ADC_FILTERED)
  #define OVERSAMPLENR 1
#else
  #define OVERSAMPLENR 16
#endif
  
// Currently Marlin stores all oversampled ADC values as uint16_t, make sure the HAL settings do not overflow 16 bit
#if (HAL_ADC_RANGE) * (OVERSAMPLENR) > 1 << 16
  #error "MAX_RAW_THERMISTOR_VALUE is too large for uint16_t. Reduce OVERSAMPLENR or HAL_ADC_RESOLUTION."
#endif
#define MAX_RAW_THERMISTOR_VALUE (uint16_t(HAL_ADC_RANGE) * (OVERSAMPLENR) - 1)

#define OV_SCALE(N) float(N)
#define OV(N) raw_adc_t(OV_SCALE(N) * (OVERSAMPLENR) * (THERMISTOR_TABLE_SCALE))

typedef struct { raw_adc_t value; celsius_t celsius; } temp_entry_t;


#if ANY_THERMISTOR_IS(1)      // beta25 = 4092 K, R25 = 100kΩ, Pullup = 4.7kΩ, "EPCOS"
  #include "thermistor_1.h"
#endif
#if ANY_THERMISTOR_IS(11)     // beta25 = 3950 K, R25 = 100kΩ, Pullup = 4.7kΩ, "QU-BD silicone bed, QWG-104F-3950"
  #include "thermistor_11.h"
#endif

#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#if TEMP_SENSOR_0 > 0
  #define TEMPTABLE_0 TT_NAME(TEMP_SENSOR_0)
  #define TEMPTABLE_0_LEN COUNT(TEMPTABLE_0)
#else
  #define TEMPTABLE_0 nullptr
  #define TEMPTABLE_0_LEN 0
#endif


/**
 * The watchdog hardware timer will do a reset and disable all outputs
 * if the firmware gets too overloaded to read the temperature sensors.
 *
 * If you find that watchdog reboot causes your AVR board to hang forever,
 * enable WATCHDOG_RESET_MANUAL to use a custom timer instead of WDTO.
 * NOTE: This method is less reliable as it can only catch hangups while
 * interrupts are enabled.
 */
#define USE_WATCHDOG
#if ENABLED(USE_WATCHDOG)
  //#define WATCHDOG_RESET_MANUAL
#endif

// Watchdog
static void watchdog_init()    IF_DISABLED(USE_WATCHDOG, {});
static void watchdog_refresh() IF_DISABLED(USE_WATCHDOG, {});


#ifdef ADC_RESOLUTION
#define HAL_ADC_RESOLUTION ADC_RESOLUTION
#else
#define HAL_ADC_RESOLUTION 12
#endif

#define OVERSAMPLENR 16

// temperature_raw_temps_ready = false;

typedef int32_t pin_t; // Parity with platform/ststm32

#if HAS_HOTEND_THERMISTOR
  #define NEXT_TEMPTABLE(N) ,TEMPTABLE_##N
  #define NEXT_TEMPTABLE_LEN(N) ,TEMPTABLE_##N##_LEN
  static const temp_entry_t* heater_ttbl_map[HOTENDS] = ARRAY_BY_HOTENDS(TEMPTABLE_0 REPEAT_S(1, HOTENDS, NEXT_TEMPTABLE));
  static constexpr uint8_t heater_ttbllen_map[HOTENDS] = ARRAY_BY_HOTENDS(TEMPTABLE_0_LEN REPEAT_S(1, HOTENDS, NEXT_TEMPTABLE_LEN));
#endif



// Temperature sensor read value ranges
typedef struct { raw_adc_t raw_min, raw_max; celsius_t mintemp, maxtemp; } temp_range_t;

// Set the high and low raw values for the heaters
// For thermistors the highest temperature results in the lowest ADC value
// For thermocouples the highest temperature results in the highest ADC value

#define _TT_REV(N)    REVERSE_TEMP_SENSOR_RANGE_##N
#define TT_REV(N)     TERN0(TEMP_SENSOR_##N##_IS_THERMISTOR, DEFER4(_TT_REV)(TEMP_SENSOR(N)))
#define _TT_REVRAW(N) !TEMP_SENSOR_##N##_IS_THERMISTOR
#define TT_REVRAW(N)  (TT_REV(N) || _TT_REVRAW(N))

#ifdef TEMPTABLE_0
  #if TT_REV(0)
    #define TEMP_SENSOR_0_MINTEMP_IND 0
    #define TEMP_SENSOR_0_MAXTEMP_IND TEMPTABLE_0_LEN - 1
  #else
    #define TEMP_SENSOR_0_MINTEMP_IND TEMPTABLE_0_LEN - 1
    #define TEMP_SENSOR_0_MAXTEMP_IND 0
  #endif
#endif
#ifdef TEMPTABLE_1
  #if TT_REV(1)
    #define TEMP_SENSOR_1_MINTEMP_IND 0
    #define TEMP_SENSOR_1_MAXTEMP_IND TEMPTABLE_1_LEN - 1
  #else
    #define TEMP_SENSOR_1_MINTEMP_IND TEMPTABLE_1_LEN - 1
    #define TEMP_SENSOR_1_MAXTEMP_IND 0
  #endif
#endif
#ifdef TEMPTABLE_2
  #if TT_REV(2)
    #define TEMP_SENSOR_2_MINTEMP_IND 0
    #define TEMP_SENSOR_2_MAXTEMP_IND TEMPTABLE_2_LEN - 1
  #else
    #define TEMP_SENSOR_2_MINTEMP_IND TEMPTABLE_2_LEN - 1
    #define TEMP_SENSOR_2_MAXTEMP_IND 0
  #endif
#endif
#ifdef TEMPTABLE_3
  #if TT_REV(3)
    #define TEMP_SENSOR_3_MINTEMP_IND 0
    #define TEMP_SENSOR_3_MAXTEMP_IND TEMPTABLE_3_LEN - 1
  #else
    #define TEMP_SENSOR_3_MINTEMP_IND TEMPTABLE_3_LEN - 1
    #define TEMP_SENSOR_3_MAXTEMP_IND 0
  #endif
#endif
#ifdef TEMPTABLE_4
  #if TT_REV(4)
    #define TEMP_SENSOR_4_MINTEMP_IND 0
    #define TEMP_SENSOR_4_MAXTEMP_IND TEMPTABLE_4_LEN - 1
  #else
    #define TEMP_SENSOR_4_MINTEMP_IND TEMPTABLE_4_LEN - 1
    #define TEMP_SENSOR_4_MAXTEMP_IND 0
  #endif
#endif
#ifdef TEMPTABLE_5
  #if TT_REV(5)
    #define TEMP_SENSOR_5_MINTEMP_IND 0
    #define TEMP_SENSOR_5_MAXTEMP_IND TEMPTABLE_5_LEN - 1
  #else
    #define TEMP_SENSOR_5_MINTEMP_IND TEMPTABLE_5_LEN - 1
    #define TEMP_SENSOR_5_MAXTEMP_IND 0
  #endif
#endif
#ifdef TEMPTABLE_6
  #if TT_REV(6)
    #define TEMP_SENSOR_6_MINTEMP_IND 0
    #define TEMP_SENSOR_6_MAXTEMP_IND TEMPTABLE_6_LEN - 1
  #else
    #define TEMP_SENSOR_6_MINTEMP_IND TEMPTABLE_6_LEN - 1
    #define TEMP_SENSOR_6_MAXTEMP_IND 0
  #endif
#endif
#ifdef TEMPTABLE_7
  #if TT_REV(7)
    #define TEMP_SENSOR_7_MINTEMP_IND 0
    #define TEMP_SENSOR_7_MAXTEMP_IND TEMPTABLE_7_LEN - 1
  #else
    #define TEMP_SENSOR_7_MINTEMP_IND TEMPTABLE_7_LEN - 1
    #define TEMP_SENSOR_7_MAXTEMP_IND 0
  #endif
#endif

#ifndef TEMP_SENSOR_0_RAW_HI_TEMP
  #if TT_REVRAW(0)
    #define TEMP_SENSOR_0_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_0_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_0_RAW_HI_TEMP 0
    #define TEMP_SENSOR_0_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_1_RAW_HI_TEMP
  #if TT_REVRAW(1)
    #define TEMP_SENSOR_1_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_1_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_1_RAW_HI_TEMP 0
    #define TEMP_SENSOR_1_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_2_RAW_HI_TEMP
  #if TT_REVRAW(2)
    #define TEMP_SENSOR_2_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_2_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_2_RAW_HI_TEMP 0
    #define TEMP_SENSOR_2_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_3_RAW_HI_TEMP
  #if TT_REVRAW(3)
    #define TEMP_SENSOR_3_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_3_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_3_RAW_HI_TEMP 0
    #define TEMP_SENSOR_3_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_4_RAW_HI_TEMP
  #if TT_REVRAW(4)
    #define TEMP_SENSOR_4_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_4_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_4_RAW_HI_TEMP 0
    #define TEMP_SENSOR_4_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_5_RAW_HI_TEMP
  #if TT_REVRAW(5)
    #define TEMP_SENSOR_5_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_5_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_5_RAW_HI_TEMP 0
    #define TEMP_SENSOR_5_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_6_RAW_HI_TEMP
  #if TT_REVRAW(6)
    #define TEMP_SENSOR_6_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_6_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_6_RAW_HI_TEMP 0
    #define TEMP_SENSOR_6_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_7_RAW_HI_TEMP
  #if TT_REVRAW(7)
    #define TEMP_SENSOR_7_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_7_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_7_RAW_HI_TEMP 0
    #define TEMP_SENSOR_7_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_BED_RAW_HI_TEMP
  #if TT_REVRAW(BED)
    #define TEMP_SENSOR_BED_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_BED_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_BED_RAW_HI_TEMP 0
    #define TEMP_SENSOR_BED_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_CHAMBER_RAW_HI_TEMP
  #if TT_REVRAW(CHAMBER)
    #define TEMP_SENSOR_CHAMBER_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_CHAMBER_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_CHAMBER_RAW_HI_TEMP 0
    #define TEMP_SENSOR_CHAMBER_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_COOLER_RAW_HI_TEMP
  #if TT_REVRAW(COOLER)
    #define TEMP_SENSOR_COOLER_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_COOLER_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_COOLER_RAW_HI_TEMP 0
    #define TEMP_SENSOR_COOLER_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_PROBE_RAW_HI_TEMP
  #if TT_REVRAW(PROBE)
    #define TEMP_SENSOR_PROBE_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_PROBE_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_PROBE_RAW_HI_TEMP 0
    #define TEMP_SENSOR_PROBE_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_BOARD_RAW_HI_TEMP
  #if TT_REVRAW(BOARD)
    #define TEMP_SENSOR_BOARD_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_BOARD_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_BOARD_RAW_HI_TEMP 0
    #define TEMP_SENSOR_BOARD_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif
#ifndef TEMP_SENSOR_REDUNDANT_RAW_HI_TEMP
  #if TT_REVRAW(REDUNDANT)
    #define TEMP_SENSOR_REDUNDANT_RAW_HI_TEMP MAX_RAW_THERMISTOR_VALUE
    #define TEMP_SENSOR_REDUNDANT_RAW_LO_TEMP 0
  #else
    #define TEMP_SENSOR_REDUNDANT_RAW_HI_TEMP 0
    #define TEMP_SENSOR_REDUNDANT_RAW_LO_TEMP MAX_RAW_THERMISTOR_VALUE
  #endif
#endif

#undef __TT_REV
#undef _TT_REV
#undef TT_REV
#undef _TT_REVRAW
#undef TT_REVRAW

    /**
     * Preheating hotends
     */
    #if MILLISECONDS_PREHEAT_TIME > 0
      static bool is_preheating(const uint8_t E_NAME) {
        return preheat_end_ms_hotend[HOTEND_INDEX] && PENDING(millis(), preheat_end_ms_hotend[HOTEND_INDEX]);
      }
      static void start_preheat_time(const uint8_t E_NAME) {
        preheat_end_ms_hotend[HOTEND_INDEX] = millis() + MILLISECONDS_PREHEAT_TIME;
      }
      static void reset_preheat_time(const uint8_t E_NAME) {
        preheat_end_ms_hotend[HOTEND_INDEX] = 0;
      }
    #else
      #define is_preheating(n) (false)
    #endif

    
#define HOTEND_INDEX TERN(HAS_MULTI_HOTEND, e, 0)
#define E_NAME TERN_(HAS_MULTI_HOTEND, e)

// @section pid temp

#if ANY(PIDTEMP, PIDTEMPBED, PIDTEMPCHAMBER)
  //#define PID_OPENLOOP          // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  //#define SLOW_PWM_HEATERS      // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                  // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

  //#define PID_EDIT_MENU         // Add PID editing to the "Advanced Settings" menu. (~700 bytes of flash)
  //#define PID_AUTOTUNE_MENU     // Add PID auto-tuning to the "Advanced Settings" menu. (~250 bytes of flash)
#endif

//
// PID
//

typedef struct { float p, i, d; } raw_pid_t;
typedef struct { float p, i, d, c, f; } raw_pidcf_t;


#if HAS_PID_HEATING

  #define PID_K2 (1.0f - float(PID_K1))
  #define PID_dT ((OVERSAMPLENR * float(ACTUAL_ADC_SAMPLES)) / (TEMP_TIMER_FREQUENCY))

  // Apply the scale factors to the PID values
  #define scalePID_i(i)   ( float(i) * PID_dT )
  #define unscalePID_i(i) ( float(i) / PID_dT )
  #define scalePID_d(d)   ( float(d) / PID_dT )
  #define unscalePID_d(d) ( float(d) * PID_dT )

  /**
   * @brief The default PID class, only has Kp, Ki, Kd, other classes extend this one
   * @tparam MIN_POW output when current is above target by functional_range
   * @tparam MAX_POW output when current is below target by functional_range
   * @details This class has methods for Kc and Kf terms, but returns constant default values.
   *          PID classes that implement these features are expected to override these methods.
   *          Since the eventual PID class is typedef-d, there is no need to use virtual functions.
   */
  template<int MIN_POW, int MAX_POW>
  struct PID_t {
    protected:
      bool pid_reset = true;
      float temp_iState = 0.0f, temp_dState = 0.0f;
      float work_p = 0, work_i = 0, work_d = 0;

    public:
      float Kp = 0, Ki = 0, Kd = 0;
      float p() const { return Kp; }
      float i() const { return unscalePID_i(Ki); }
      float d() const { return unscalePID_d(Kd); }
      float c() const { return 1; }
      float f() const { return 0; }
      float pTerm() const { return work_p; }
      float iTerm() const { return work_i; }
      float dTerm() const { return work_d; }
      float cTerm() const { return 0; }
      float fTerm() const { return 0; }
      void set_Kp(float p) { Kp = p; }
      void set_Ki(float i) { Ki = scalePID_i(i); }
      void set_Kd(float d) { Kd = scalePID_d(d); }
      void set_Kc(float) {}
      void set_Kf(float) {}
      int low() const { return MIN_POW; }
      int high() const { return MAX_POW; }
      void reset() { pid_reset = true; }
      void set(float p, float i, float d, float c=1, float f=0) { set_Kp(p); set_Ki(i); set_Kd(d); set_Kc(c); set_Kf(f); }
      void set(const raw_pid_t &raw) { set(raw.p, raw.i, raw.d); }
      void set(const raw_pidcf_t &raw) { set(raw.p, raw.i, raw.d, raw.c, raw.f); }

      float get_fan_scale_output(const uint8_t) { return 0; }

      float get_extrusion_scale_output(const bool, const int32_t, const float, const int16_t) { return 0; }

      float get_pid_output(const float target, const float current) {
        const float pid_error = target - current;
        if (!target || pid_error < -(PID_FUNCTIONAL_RANGE)) {
          pid_reset = true;
          return 0;
        }
        else if (pid_error > PID_FUNCTIONAL_RANGE) {
          pid_reset = true;
          return MAX_POW;
        }

        if (pid_reset) {
          pid_reset = false;
          temp_iState = 0.0;
          work_d = 0.0;
        }

        const float max_power_over_i_gain = float(MAX_POW) / Ki - float(MIN_POW);
        temp_iState = constrain(temp_iState + pid_error, 0, max_power_over_i_gain);

        work_p = Kp * pid_error;
        work_i = Ki * temp_iState;
        work_d = work_d + PID_K2 * (Kd * (temp_dState - current) - work_d);

        temp_dState = current;

        return constrain(work_p + work_i + work_d + float(MIN_POW), 0, MAX_POW);
      }

  };
#endif // HAS_PID_HEATING

typedef
  PID_t<0, PID_MAX>
hotend_pid_t;

typedef struct PIDHeaterInfo<hotend_pid_t> hotend_info_t;

static hotend_info_t temp_hotend[HOTENDS];
static constexpr celsius_t hotend_maxtemp[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP, HEATER_4_MAXTEMP, HEATER_5_MAXTEMP, HEATER_6_MAXTEMP, HEATER_7_MAXTEMP);
static celsius_t hotend_max_target(const uint8_t e) { return hotend_maxtemp[e] - (HOTEND_OVERSHOOT); }


/**
 * Helper Macros for heaters and extruder fan
 */
#define WRITE_HEATER_0P(v) WRITE(HEATER_0_PIN, (v) ^ ENABLED(HEATER_0_INVERTING))
#if ANY(HAS_MULTI_HOTEND, HEATERS_PARALLEL)
  #define WRITE_HEATER_1(v) WRITE(HEATER_1_PIN, (v) ^ ENABLED(HEATER_1_INVERTING))
  #if HOTENDS > 2
    #define WRITE_HEATER_2(v) WRITE(HEATER_2_PIN, (v) ^ ENABLED(HEATER_2_INVERTING))
    #if HOTENDS > 3
      #define WRITE_HEATER_3(v) WRITE(HEATER_3_PIN, (v) ^ ENABLED(HEATER_3_INVERTING))
      #if HOTENDS > 4
        #define WRITE_HEATER_4(v) WRITE(HEATER_4_PIN, (v) ^ ENABLED(HEATER_4_INVERTING))
        #if HOTENDS > 5
          #define WRITE_HEATER_5(v) WRITE(HEATER_5_PIN, (v) ^ ENABLED(HEATER_5_INVERTING))
          #if HOTENDS > 6
            #define WRITE_HEATER_6(v) WRITE(HEATER_6_PIN, (v) ^ ENABLED(HEATER_6_INVERTING))
            #if HOTENDS > 7
              #define WRITE_HEATER_7(v) WRITE(HEATER_7_PIN, (v) ^ ENABLED(HEATER_7_INVERTING))
            #endif // HOTENDS > 7
          #endif // HOTENDS > 6
        #endif // HOTENDS > 5
      #endif // HOTENDS > 4
    #endif // HOTENDS > 3
  #endif // HOTENDS > 2
#endif // HAS_MULTI_HOTEND || HEATERS_PARALLEL
#if ENABLED(HEATERS_PARALLEL)
  #define WRITE_HEATER_0(v) { WRITE_HEATER_0P(v); WRITE_HEATER_1(v); }
#else
  #define WRITE_HEATER_0(v) WRITE_HEATER_0P(v)
#endif


static celsius_t temperature_degTargetHotend(const uint8_t E_NAME) {
  return TERN0(HAS_HOTEND, temp_hotend[HOTEND_INDEX].target);
}

static void temperature_setTargetHotend(const celsius_t celsius, const uint8_t E_NAME) {
  const uint8_t ee = HOTEND_INDEX;
  #if MILLISECONDS_PREHEAT_TIME > 0
    if (celsius == 0)
      reset_preheat_time(ee);
    else if (temp_hotend[ee].target == 0)
      start_preheat_time(ee);
  #endif
  TERN_(AUTO_POWER_CONTROL, if (celsius) powerManager.power_on());
  temp_hotend[ee].target = _MIN(celsius, hotend_max_target(ee));
  // temperature_start_watching_hotend(ee);
}

// Start watching a Hotend to make sure it's really heating up
static void temperature_start_watching_hotend(const uint8_t E_NAME) {
  UNUSED(HOTEND_INDEX);
  #if WATCH_HOTENDS
    watch_hotend[HOTEND_INDEX].restart(degHotend(HOTEND_INDEX), degTargetHotend(HOTEND_INDEX));
  #endif
}