#include "temperature.h"


#if HAS_HOTEND
  // Init mintemp and maxtemp with extreme values to prevent false errors during startup
  constexpr temp_range_t sensor_heater_0 { TEMP_SENSOR_0_RAW_LO_TEMP, TEMP_SENSOR_0_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_1 { TEMP_SENSOR_1_RAW_LO_TEMP, TEMP_SENSOR_1_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_2 { TEMP_SENSOR_2_RAW_LO_TEMP, TEMP_SENSOR_2_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_3 { TEMP_SENSOR_3_RAW_LO_TEMP, TEMP_SENSOR_3_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_4 { TEMP_SENSOR_4_RAW_LO_TEMP, TEMP_SENSOR_4_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_5 { TEMP_SENSOR_5_RAW_LO_TEMP, TEMP_SENSOR_5_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_6 { TEMP_SENSOR_6_RAW_LO_TEMP, TEMP_SENSOR_6_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_7 { TEMP_SENSOR_7_RAW_LO_TEMP, TEMP_SENSOR_7_RAW_HI_TEMP, 0, 16383 };

  temp_range_t temp_range[HOTENDS] = ARRAY_BY_HOTENDS(sensor_heater_0, sensor_heater_1, sensor_heater_2, sensor_heater_3, sensor_heater_4, sensor_heater_5, sensor_heater_6, sensor_heater_7);
#endif


#if ENABLED(USE_WATCHDOG)

  #define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout

  #include <IWatchdog.h>

  void watchdog_init() {
    IF_DISABLED(DISABLE_WATCHDOG_INIT, IWatchdog.begin(WDT_TIMEOUT_US));
  }

  void watchdog_refresh() {
    IWatchdog.reload();
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN);  // heartbeat indicator
    #endif
  }

#endif


//
// ADC Methods
//

static uint16_t adc_result;

// Called by Temperature::init once at startup
static void adc_init()
{
  analogReadResolution(HAL_ADC_RESOLUTION);
}
// Called by Temperature::init for each sensor at startup
static void adc_enable(const pin_t pin) { pinMode(pin, INPUT); }

// Is the ADC ready for reading?
static bool adc_ready() { return true; }

// The current value of the ADC register
static uint16_t adc_value() { return adc_result; }


// Begin ADC sampling on the given pin. Called from Temperature::isr!
static void adc_start(const pin_t pin) { adc_result = analogRead(pin); }
/**
 * Update raw temperatures
 *
 * Called by ISR => readings_ready when new temperatures have been set by updateTemperaturesFromRawValues.
 * Applies all the accumulators to the current raw temperatures.
 */
void temperature_update_raw_temperatures() {

  // TODO: can this be collapsed into a HOTEND_LOOP()?
    temp_hotend[0].update();
}

/**
 * Called by the Temperature ISR when all the ADCs have been processed.
 * Reset all the ADC accumulators for another round of updates.
 */
void temperature_readings_ready() {

  // Update raw values only if they're not already set.
  if (!temperature_raw_temps_ready) {
    temperature_update_raw_temperatures();
    temperature_raw_temps_ready = true;
  }

  HOTEND_LOOP() temp_hotend[e].reset();
}

#if HAS_PID_HEATING

  template<typename TT>
  class PIDRunner {
  public:
    TT &tempinfo;

    PIDRunner(TT &t) : tempinfo(t) { }

    float get_pid_output(const uint8_t extr=0) {
      #if ENABLED(PID_OPENLOOP)

        return constrain(tempinfo.target, 0, MAX_POW);

      #else // !PID_OPENLOOP

        float out = tempinfo.pid.get_pid_output(tempinfo.target, tempinfo.celsius);

        #if ENABLED(PID_FAN_SCALING)
          out += tempinfo.pid.get_fan_scale_output(thermalManager.fan_speed[extr]);
        #endif

        #if ENABLED(PID_EXTRUSION_SCALING)
          out += tempinfo.pid.get_extrusion_scale_output(
            extr == active_extruder, stepper.position(E_AXIS), planner.mm_per_step[E_AXIS], thermalManager.lpq_len
          );
        #endif

        return constrain(out, tempinfo.pid.low(), tempinfo.pid.high());

      #endif // !PID_OPENLOOP
    }
  };

#endif // HAS_PID_HEATING

#if HAS_HOTEND

  float temperature_get_pid_output_hotend(const uint8_t E_NAME) {
    const uint8_t ee = HOTEND_INDEX;

    const bool is_idling = TERN0(HEATER_IDLE_HANDLER, heater_idle[ee].timed_out);


      typedef PIDRunner<hotend_info_t> PIDRunnerHotend;

      static PIDRunnerHotend hotend_pid[HOTENDS] = {
        #define _HOTENDPID(E) temp_hotend[E],
        REPEAT(HOTENDS, _HOTENDPID)
      };

      const float pid_output = is_idling ? 0 : hotend_pid[ee].get_pid_output(ee);

      #if ENABLED(PID_DEBUG)
        if (ee == active_extruder)
          hotend_pid[ee].debug(temp_hotend[ee].celsius, pid_output, F("E"), ee);
      #endif

    return pid_output;
  }

#endif // HAS_HOTEND

#if HAS_HOTEND

  void temperature_manage_hotends(const millis_t &ms) {
    HOTEND_LOOP() {
      #if ENABLED(THERMAL_PROTECTION_HOTENDS)
        // if (degHotend(e) > temp_range[e].maxtemp) maxtemp_error((heater_id_t)e);
      #endif

      TERN_(HEATER_IDLE_HANDLER, heater_idle[e].update(ms));

      #if ENABLED(THERMAL_PROTECTION_HOTENDS)
        // Check for thermal runaway
        // tr_state_machine[e].run(temp_hotend[e].celsius, temp_hotend[e].target, (heater_id_t)e, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
      #endif
      // 把 pwm 值从 0-255 缩放到 0-127 
      temp_hotend[e].soft_pwm_amount = (temp_hotend[e].celsius > temp_range[e].mintemp || is_preheating(e)) && temp_hotend[e].celsius < temp_range[e].maxtemp ? (int)temperature_get_pid_output_hotend(e) >> 1 : 0;

      #if WATCH_HOTENDS
        // Make sure temperature is increasing
        if (watch_hotend[e].elapsed(ms)) {          // Enabled and time to check?
          if (watch_hotend[e].check(degHotend(e)))  // Increased enough?
            start_watching_hotend(e);               // If temp reached, turn off elapsed check
          else {
            TERN_(HAS_DWIN_E3V2_BASIC, DWIN_Popup_Temperature(0));
            _temp_error((heater_id_t)e, FPSTR(str_t_heating_failed), GET_TEXT_F(MSG_HEATING_FAILED_LCD));
          }
        }
      #endif

    } // HOTEND_LOOP
  }
#endif //HAS_HOTEND


class SoftPWM {
  public:
    uint8_t count;
    inline bool add(const uint8_t mask, const uint8_t amount) {
      count = (count & mask) + amount; return (count > mask);
    }
  };
  

/**
 * Handle various ~1kHz tasks associated with temperature
 *  - Check laser safety timeout
 *  - Heater PWM (~1kHz with scaler)
 *  - LCD Button polling (~500Hz)
 *  - Start / Read one ADC sensor
 *  - Advance Babysteps
 *  - Endstop polling
 *  - Planner clean buffer
 */
void temperature_isr()
{
  static int8_t temp_count = -1;
  static ADCSensorState adc_sensor_state = StartupDelay;

  #ifndef SOFT_PWM_SCALE
    #define SOFT_PWM_SCALE 0
  #endif
  static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);

  // Avoid multiple loads of pwm_count
  uint8_t pwm_count_tmp = pwm_count;


  #if HAS_HOTEND
    static SoftPWM soft_pwm_hotend[HOTENDS];
  #endif

  #if HAS_HEATED_BED
    static SoftPWM soft_pwm_bed;
  #endif


  // #define WRITE_FAN(n, v) WRITE(FAN##n##_PIN, (v) ^ ENABLED(FAN_INVERTING))

  #if DISABLED(SLOW_PWM_HEATERS)

    #if ANY(HAS_HOTEND, HAS_HEATED_BED, HAS_HEATED_CHAMBER, HAS_COOLER, FAN_SOFT_PWM)
      constexpr uint8_t pwm_mask = TERN0(SOFT_PWM_DITHER, _BV(SOFT_PWM_SCALE) - 1);
      #define _PWM_MOD(N,S,T) do{                           \
        const bool on = S.add(pwm_mask, T.soft_pwm_amount); \
        WRITE_HEATER_##N(on);                               \
      }while(0)
    #endif

    /**
     * Standard heater PWM modulation
     */
    if (pwm_count_tmp >= 127) {
      pwm_count_tmp -= 127;

      #if HAS_HOTEND
        #define _PWM_MOD_E(N) _PWM_MOD(N,soft_pwm_hotend[N],temp_hotend[N]);
        REPEAT(HOTENDS, _PWM_MOD_E);
      #endif

      #if HAS_HEATED_BED
        _PWM_MOD(BED, soft_pwm_bed, temp_bed);
      #endif

    }
    else {
      #define _PWM_LOW(N,S) do{ if (S.count <= pwm_count_tmp) WRITE_HEATER_##N(LOW); }while(0)
      #if HAS_HOTEND
        #define _PWM_LOW_E(N) _PWM_LOW(N, soft_pwm_hotend[N]);
        REPEAT(HOTENDS, _PWM_LOW_E);
      #endif

      #if HAS_HEATED_BED
        _PWM_LOW(BED, soft_pwm_bed);
      #endif

    }

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);

  #endif // SLOW_PWM_HEATERS

/**
 * One sensor is sampled on every other call of the ISR.
 * Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
 *
 * On each Prepare pass, ADC is started for a sensor pin.
 * On the next pass, the ADC value is read and accumulated.
 *
 * This gives each ADC 0.9765ms to charge up.
 */
#define ACCUMULATE_ADC(obj)                 \
  do                                        \
  {                                         \
    if (!adc_ready())                       \
      next_sensor_state = adc_sensor_state; \
    else                                    \
      obj.sample(adc_value());              \
  } while (0)

  ADCSensorState next_sensor_state = adc_sensor_state < SensorsReady ? (ADCSensorState)(int(adc_sensor_state) + 1) : StartSampling;

  switch (adc_sensor_state)
  {

#pragma GCC diagnostic push
#if __has_cpp_attribute(fallthrough)
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#endif

  case SensorsReady:
  {
    // All sensors have been read. Stay in this state for a few
    // ISRs to save on calls to temp update/checking code below.
    constexpr int8_t extra_loops = MIN_ADC_ISR_LOOPS - (int8_t)SensorsReady;
    static uint8_t delay_count = 0;
    if (extra_loops > 0)
    {
      if (delay_count == 0)
        delay_count = extra_loops;        // Init this delay
      if (--delay_count)                  // While delaying...
        next_sensor_state = SensorsReady; // retain this state (else, next state will be 0)
      break;
    }
    else
    {
      adc_sensor_state = StartSampling; // Fall-through to start sampling
      next_sensor_state = (ADCSensorState)(int(StartSampling) + 1);
    }
  }

#pragma GCC diagnostic pop
  case StartSampling: // Start of sampling loops. Do updates/checks.
    if (++temp_count >= OVERSAMPLENR)
    { // 10 * 16 * 1/(16000000/64/256)  = 164ms.
      temp_count = 0;
      temperature_readings_ready();
    }
    break;

  case PrepareTemp_0:
    adc_start(TEMP_0_PIN);
    break;
  case MeasureTemp_0:
    ACCUMULATE_ADC(temp_hotend[0]);
    break;
  case StartupDelay:
    break;

  } // switch(adc_sensor_state)
  // Go to the next state
  adc_sensor_state = next_sensor_state;
}

/**
 * Timer 0 is shared with millies so don't change the prescaler.
 *
 * On AVR this ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the TCNT0 set
 * in OCR0B above (128 or halfway between OVFs).
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Check new temperature values for MIN/MAX errors (kill on error)
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 *  - Call planner.isr to count down its "ignore" time
 */
HAL_TEMP_TIMER_ISR()
{
  HAL_timer_isr_prologue(MF_TIMER_TEMP);

  temperature_isr();

  HAL_timer_isr_epilogue(MF_TIMER_TEMP);
}

HAL_STEP_TIMER_ISR()
{
  ;
}


/**
 * Bisect search for the range of the 'raw' value, then interpolate
 * proportionally between the under and over values.
 */
// 二分法，线性插值
#define SCAN_THERMISTOR_TABLE(TBL,LEN) do{                                \
  uint8_t l = 0, r = LEN, m;                                              \
  for (;;) {                                                              \
    m = (l + r) >> 1;                                                     \
    if (!m) return celsius_t(pgm_read_word(&TBL[0].celsius));             \
    if (m == l || m == r) return celsius_t(pgm_read_word(&TBL[LEN-1].celsius)); \
    raw_adc_t v00 = pgm_read_word(&TBL[m-1].value),                       \
              v10 = pgm_read_word(&TBL[m-0].value);                       \
         if (raw < v00) r = m;                                            \
    else if (raw > v10) l = m;                                            \
    else {                                                                \
      const celsius_t v01 = celsius_t(pgm_read_word(&TBL[m-1].celsius)),  \
                      v11 = celsius_t(pgm_read_word(&TBL[m-0].celsius));  \
      return v01 + (raw - v00) * float(v11 - v01) / float(v10 - v00);     \
    }                                                                     \
  }                                                                       \
}while(0)


// ADC值转换为温度值
#if HAS_HOTEND
  // Derived from RepRap FiveD extruder::getTemperature()
  // For hot end temperature measurement.
  celsius_float_t temperature_analog_to_celsius_hotend(const raw_adc_t raw, const uint8_t e) {
    if (e >= HOTENDS) {
      // SERIAL_ERROR_START();
      // SERIAL_ECHO(e);
      // SERIAL_ECHOLNPGM(STR_INVALID_EXTRUDER_NUM);
      // kill();
      return 0;
    }
    
  // Serial1.print("temp_hotend[HOTENDS].getraw() : ");
  // Serial1.println(raw);
  // Serial1.println(temp_hotend[HOTENDS].celsius);

    #if HAS_HOTEND_THERMISTOR
      // Thermistor with conversion table?
      const temp_entry_t(*tt)[] = (temp_entry_t(*)[])(heater_ttbl_map[e]);
      SCAN_THERMISTOR_TABLE((*tt), heater_ttbllen_map[e]);
    #endif

    return 0;
  }
#endif // HAS_HOTEND

// 采样16次后更新温度值
void temperature_updateTemperaturesFromRawValues() {

  watchdog_refresh(); // Reset because raw_temps_ready was set by the interrupt


  #if HAS_HOTEND
    HOTEND_LOOP() temp_hotend[e].celsius = temperature_analog_to_celsius_hotend(temp_hotend[e].getraw(), e);
  #endif


  #if 0
    static constexpr int8_t temp_dir[HOTENDS] = {
      #if TEMP_SENSOR_IS_ANY_MAX_TC(0)
        0
      #else
        TEMPDIR(0)
      #endif
      #if HAS_MULTI_HOTEND
        #if TEMP_SENSOR_IS_ANY_MAX_TC(1)
          , 0
        #else
          , TEMPDIR(1)
        #endif
      #endif
      #if HOTENDS > 2
        #if TEMP_SENSOR_IS_ANY_MAX_TC(2)
          , 0
        #else
          , TEMPDIR(2)
        #endif
      #endif
      #if HOTENDS > 3
        #define _TEMPDIR(N) , TEMPDIR(N)
        REPEAT_S(3, HOTENDS, _TEMPDIR)
      #endif
    };

    HOTEND_LOOP() {
      const raw_adc_t r = temp_hotend[e].getraw();
      const bool neg = temp_dir[e] < 0, pos = temp_dir[e] > 0;
      if ((neg && r < temp_range[e].raw_max) || (pos && r > temp_range[e].raw_max))
        maxtemp_error((heater_id_t)e);

      /**
      // DEBUG PREHEATING TIME
      SERIAL_ECHOLNPGM("\nExtruder = ", e, " Preheat On/Off = ", is_preheating(e));
      const float test_is_preheating = (preheat_end_ms_hotend[HOTEND_INDEX] - millis()) * 0.001f;
      if (test_is_preheating < 31) SERIAL_ECHOLNPGM("Extruder = ", e, " Preheat remaining time = ", test_is_preheating, "s", "\n");
      //*/

      const bool heater_on = temp_hotend[e].target > 0;
      if (heater_on && !is_preheating(e) && ((neg && r > temp_range[e].raw_min) || (pos && r < temp_range[e].raw_min))) {
        if (TERN1(MULTI_MAX_CONSECUTIVE_LOW_TEMP_ERR, ++consecutive_low_temperature_error[e] >= MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED))
          mintemp_error((heater_id_t)e);
      }
      else {
        TERN_(MULTI_MAX_CONSECUTIVE_LOW_TEMP_ERR, consecutive_low_temperature_error[e] = 0);
      }
    }

  #endif // HAS_HOTEND

  #define TP_CMP(S,A,B) (TEMPDIR(S) < 0 ? ((A)<(B)) : ((A)>(B)))


} // Temperature::updateTemperaturesFromRawValues



void temperature_reset()
{
  //
  // Hotend PID
  //

  #if ENABLED(PIDTEMP)
    #define PID_DEFAULT(N,E) DEFAULT_##N
    HOTEND_LOOP() {
      temp_hotend[e].pid.set(
        PID_DEFAULT(Kp, ALIM(e, defKp)),
        PID_DEFAULT(Ki, ALIM(e, defKi)),
        PID_DEFAULT(Kd, ALIM(e, defKd))
        OPTARG(PID_EXTRUSION_SCALING, PID_DEFAULT(Kc, ALIM(e, defKc)))
        OPTARG(PID_FAN_SCALING, PID_DEFAULT(Kf, ALIM(e, defKf)))
      );
    }
  #endif
}


void temperature_init()
{
  temperature_reset();
  OUT_WRITE(HEATER_0_PIN, false);
  adc_init();
  adc_enable(TEMP_0_PIN);
  HAL_timer_start(MF_TIMER_TEMP, TEMP_TIMER_FREQUENCY);
  ENABLE_TEMPERATURE_INTERRUPT();
  Serial1.print("Notset, TargetHotend(0): ");
  Serial1.println(temperature_degTargetHotend(0));
  temperature_setTargetHotend(45, 0);
  Serial1.print("Set, TargetHotend(0): ");
  Serial1.println(temperature_degTargetHotend(0));

}


void temperature_task()
{
  if (!temperature_updateTemperaturesIfReady()) {
    Serial1.print("temp_hotend[HOTENDS].celsius() : ");
    Serial1.println(temp_hotend[0].celsius);
    // Serial1.print("temp_hotend[HOTENDS].raw() : ");
    // Serial1.println(temp_hotend[0].getraw() * THERMISTOR_RAW_SCALE);
    return; // Will also reset the watchdog if temperatures are ready
  }
  const millis_t ms = millis();

  // Handle Hotend Temp Errors, Heating Watch, etc.
  TERN_(HAS_HOTEND, temperature_manage_hotends(ms));
}
