#include "temperature.h"


#if HAS_HOTEND
  // 设置温度传感器的 最大/最小 温度/ADC值
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
    // 此次为读取一次ADC值的最小循环次数，减掉正常三步流程（判断是否完成16次采样->读取引脚ADC值->加入到结构体的acc值中）后，仍在这里循环
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
    if (++temp_count >= OVERSAMPLENR) // 到达采样次数
    { // 10 * 16 * 1/(16000000/64/256)  = 164ms.
      temp_count = 0;
      temperature_readings_ready();   // 设置完成采样标志位，并讲acc存入raw，复位acc
    }
    break;

  case PrepareTemp_0:
    adc_start(TEMP_0_PIN);            // 读取引脚ADC数值
    break;
  case MeasureTemp_0:
    ACCUMULATE_ADC(temp_hotend[0]);   // 把上一次ISR中断读取的ADC数值加入到结构体acc中
    break;
  case StartupDelay:                  // 初始状态的第一次ISR()中断
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
  #if ENABLED(DEBUG)
    Serial1.print("Notset, TargetHotend(0): ");
    Serial1.println(temperature_degTargetHotend(0));
  #endif
    temperature_setTargetHotend(TARGET_TEMP, 0);
  #if ENABLED(DEBUG)
    Serial1.print("Set, TargetHotend(0): ");
    Serial1.println(temperature_degTargetHotend(0));
  #endif

}


void temperature_task()
{
  if (!temperature_updateTemperaturesIfReady()) {
    #if ENABLED(DEBUG)
      Serial1.print("temp_hotend[HOTENDS].celsius() : ");
      Serial1.println(temp_hotend[0].celsius);
      // Serial1.print("temp_hotend[HOTENDS].raw() : ");
      // Serial1.println(temp_hotend[0].getraw() * THERMISTOR_RAW_SCALE);
    #endif
      return; // Will also reset the watchdog if temperatures are ready
  }
  const millis_t ms = millis();

  // Handle Hotend Temp Errors, Heating Watch, etc.
  TERN_(HAS_HOTEND, temperature_manage_hotends(ms));
}


// HAL idle task
void hal_idletask() {
  #if HAS_SHARED_MEDIA
    // Stm32duino currently doesn't have a "loop/idle" method
    CDC_resume_receive();
    CDC_continue_transmit();
  #endif
}


void temperature_disable_all_heaters() {

  // Disable autotemp, unpause and reset everything
  TERN_(AUTOTEMP, planner.autotemp_enabled = false);
  TERN_(PROBING_HEATERS_OFF, pause_heaters(false));

  #if HAS_HOTEND
    HOTEND_LOOP() {
      temperature_setTargetHotend(0, e);
      temp_hotend[e].soft_pwm_amount = 0;
    }
  #endif

  #if HAS_TEMP_HOTEND
    #define DISABLE_HEATER(N) WRITE_HEATER_##N(LOW);
    REPEAT(HOTENDS, DISABLE_HEATER);
  #endif

  #if HAS_HEATED_BED
    setTargetBed(0);
    temp_bed.soft_pwm_amount = 0;
    WRITE_HEATER_BED(LOW);
  #endif

  #if HAS_HEATED_CHAMBER
    setTargetChamber(0);
    temp_chamber.soft_pwm_amount = 0;
    WRITE_HEATER_CHAMBER(LOW);
  #endif

  #if HAS_COOLER
    setTargetCooler(0);
    temp_cooler.soft_pwm_amount = 0;
    WRITE_HEATER_COOLER(LOW);
  #endif
}


// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
bool wait_for_heatup = true;

#if HAS_PID_HEATING

  inline void say_default_() { SERIAL_ECHOPGM("#define DEFAULT_"); }

  /**
   * PID Autotuning (M303)
   *
   * Alternately heat and cool the nozzle, observing its behavior to
   * determine the best PID values to achieve a stable temperature.
   * Needs sufficient heater power to make some overshoot at target
   * temperature to succeed.
   */
  void temperature_PID_autotune(const celsius_t target, const heater_id_t heater_id, const int8_t ncycles, const bool set_result/*=false*/) {
    celsius_float_t current_temp = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t next_temp_ms = millis(), t1 = next_temp_ms, t2 = next_temp_ms;
    long t_high = 0, t_low = 0;

    raw_pid_t tune_pid = { 0, 0, 0 };
    celsius_float_t maxT = 0, minT = 10000;

    const bool isbed = (heater_id == H_BED),
           ischamber = (heater_id == H_CHAMBER);

    #if ENABLED(PIDTEMPCHAMBER)
      #define C_TERN(T,A,B) ((T) ? (A) : (B))
    #else
      #define C_TERN(T,A,B) (B)
    #endif
    #if ENABLED(PIDTEMPBED)
      #define B_TERN(T,A,B) ((T) ? (A) : (B))
    #else
      #define B_TERN(T,A,B) (B)
    #endif
    #define GHV(C,B,H) C_TERN(ischamber, C, B_TERN(isbed, B, H))
    #define SHV(V) C_TERN(ischamber, temp_chamber.soft_pwm_amount = V, B_TERN(isbed, temp_bed.soft_pwm_amount = V, temp_hotend[heater_id].soft_pwm_amount = V))
    #define ONHEATINGSTART() C_TERN(ischamber, printerEventLEDs.onChamberHeatingStart(), B_TERN(isbed, printerEventLEDs.onBedHeatingStart(), printerEventLEDs.onHotendHeatingStart()))
    #define ONHEATING(S,C,T) C_TERN(ischamber, printerEventLEDs.onChamberHeating(S,C,T), B_TERN(isbed, printerEventLEDs.onBedHeating(S,C,T), printerEventLEDs.onHotendHeating(S,C,T)))

    #define WATCH_PID DISABLED(NO_WATCH_PID_TUNING) && (ALL(WATCH_CHAMBER, PIDTEMPCHAMBER) || ALL(WATCH_BED, PIDTEMPBED) || ALL(WATCH_HOTENDS, PIDTEMP))

    #if WATCH_PID
      #if ALL(THERMAL_PROTECTION_CHAMBER, PIDTEMPCHAMBER)
        #define C_GTV(T,A,B) ((T) ? (A) : (B))
      #else
        #define C_GTV(T,A,B) (B)
      #endif
      #if ALL(THERMAL_PROTECTION_BED, PIDTEMPBED)
        #define B_GTV(T,A,B) ((T) ? (A) : (B))
      #else
        #define B_GTV(T,A,B) (B)
      #endif
      #define GTV(C,B,H) C_GTV(ischamber, C, B_GTV(isbed, B, H))
      const uint16_t watch_temp_period = GTV(WATCH_CHAMBER_TEMP_PERIOD, WATCH_BED_TEMP_PERIOD, WATCH_TEMP_PERIOD);
      const uint8_t watch_temp_increase = GTV(WATCH_CHAMBER_TEMP_INCREASE, WATCH_BED_TEMP_INCREASE, WATCH_TEMP_INCREASE);
      const celsius_float_t watch_temp_target = celsius_float_t(target - (watch_temp_increase + GTV(TEMP_CHAMBER_HYSTERESIS, TEMP_BED_HYSTERESIS, TEMP_HYSTERESIS) + 1));
      millis_t temp_change_ms = next_temp_ms + SEC_TO_MS(watch_temp_period);
      celsius_float_t next_watch_temp = 0.0;
      bool heated = false;
    #endif

    TERN_(HAS_FAN_LOGIC, fan_update_ms = next_temp_ms + fan_update_interval_ms);

    TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_STARTED));
    TERN_(DWIN_LCD_PROUI, DWIN_PidTuning(isbed ? PIDTEMPBED_START : PIDTEMP_START));

    // 目标温度超过最大温度 - 超调量， 则报错返回
    if (target > GHV(CHAMBER_MAX_TARGET, BED_MAX_TARGET, temp_range[heater_id].maxtemp - (HOTEND_OVERSHOOT))) {
      SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
      TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_TEMP_TOO_HIGH));
      TERN_(DWIN_LCD_PROUI, DWIN_PidTuning(PID_TEMP_TOO_HIGH));
      TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH)));
      return;
    }
    
    // 开始pid打印
    SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_START);

    temperature_disable_all_heaters();
    TERN_(AUTO_POWER_CONTROL, powerManager.power_on());
  
    // 初始状态设置满功率运行，pwm设置为127（255/2）。bias 为基础输出功率（根据加热/冷却时间动态调节），d 为震荡幅度（确保输出不超过最大功率限制）
    long bias = GHV(MAX_CHAMBER_POWER, MAX_BED_POWER, PID_MAX) >> 1, d = bias;
    SHV(bias);

    #if ENABLED(PRINTER_EVENT_LEDS)
      const celsius_float_t start_temp = GHV(degChamber(), degBed(), degHotend(heater_id));
      LEDColor color = ONHEATINGSTART();
    #endif

    TERN_(NO_FAN_SLOWING_IN_PID_TUNING, adaptive_fan_slowing = false);

    LCD_MESSAGE(MSG_HEATING);

    // PID Tuning loop
    wait_for_heatup = true;
    while (wait_for_heatup) { // Can be interrupted with M108

      const millis_t ms = millis();

      if (temperature_updateTemperaturesIfReady()) { // temp sample ready

        // Get the current temperature and constrain it
        // 获取当前温度
        current_temp = GHV(degChamber(), degBed(), temperature_degHotend(heater_id));
        NOLESS(maxT, current_temp);       // maxT 不能小于 current_temp，记录波峰，更新最高温度
        NOMORE(minT, current_temp);       // minT 不能大于 current_temp，记录波谷，更新最低温度

        #if ENABLED(PRINTER_EVENT_LEDS)
          ONHEATING(start_temp, current_temp, target);
        #endif

        TERN_(HAS_FAN_LOGIC, manage_extruder_fans(ms));
        
        // PID 自动调节核心部分
        // 加热中、当前温度大于目标温度，时间超过 t2 + 5s，t2可能跟超调量有关，第一次全速加热后，超过目标温度就执行
        if (heating && current_temp > target && ELAPSED(ms, t2 + 5000UL)) {
          heating = false;                
          SHV((bias - d) >> 1);           // 设置 PWM 参数，第一次设置为0
          t1 = ms;                        // 当前时间设置为 t1
          t_high = t1 - t2;               // 温度上升所花费的时间
          maxT = target;                  // maxT设置为目标温度
        }

        // PID 自动调节核心部分
        // 关闭加热之后，当前温度小于目标温度，时间超过 t1 + 5s，t1可能跟未达量有关
        if (!heating && current_temp < target && ELAPSED(ms, t1 + 5000UL)) {
          heating = true;
          t2 = ms;                        // 当前时间设置为 t2
          t_low = t2 - t1;                // 温度下降所花费的时间
          if (cycles > 0) {               // 第一次加热后再关闭加热，不执行
            const long max_pow = GHV(MAX_CHAMBER_POWER, MAX_BED_POWER, PID_MAX);
            // 归一化处理，当(t_high - t_low)大于0则代表加热时间较长，需要增加功率，小于0则代表加热时间较短，冷却时间过长，温度太高，需要降低功率
            bias += (d * (t_high - t_low)) / (t_low + t_high);
            // 限制范围 20 <= bias <= 235
            LIMIT(bias, 20, max_pow - 20);
            // 确保 (bias + d) <= max_pow 且 (bias - d) >= 0
            d = (bias > max_pow >> 1) ? max_pow - 1 - bias : bias;

            SERIAL_ECHOPGM(STR_BIAS, bias, STR_D_COLON, d, STR_T_MIN, minT, STR_T_MAX, maxT);
            if (cycles > 2) {
              // 齐格勒－尼科尔斯方法（英语：Ziegler–Nichols method）是一种整定PID控制器、探索其控制参数的方法
              const float Ku = (4.0f * d) / (float(M_PI) * (maxT - minT) * 0.5f),       // 系统开始等幅度震荡时的临界增益
                          Tu = float(t_low + t_high) * 0.001f,                          // 震荡周期（秒），临界周期
                          pf = (ischamber || isbed) ? 0.2f : 0.6f,                      // 比例因子
                          df = (ischamber || isbed) ? 1.0f / 3.0f : 1.0f / 8.0f;        // 微分因子

              tune_pid.p = Ku * pf;                                                     // Kp = Ku * 0.6
              tune_pid.i = tune_pid.p * 2.0f / Tu;                                      // Ki = Kp / Ti = Kp / (0.5 * Tu)
              tune_pid.d = tune_pid.p * Tu * df;                                        // Kd = Kp * Td = Kp * (0.125 * Tu)

              SERIAL_ECHOLNPGM(STR_KU, Ku, STR_TU, Tu);
              if (ischamber || isbed)
                SERIAL_ECHOLNPGM(" No overshoot");
              else
                SERIAL_ECHOLNPGM(STR_CLASSIC_PID);
              SERIAL_ECHOLNPGM(STR_KP, tune_pid.p, STR_KI, tune_pid.i, STR_KD, tune_pid.d);
            }
          }
          SHV((bias + d) >> 1);               // 设置计算出的pwm加热功率，count 为0时，设置为 127
          TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/%i"), GET_TEXT(MSG_PID_CYCLE), cycles, ncycles));
          cycles++;
          minT = target;                      // 设置minT为目标温度
        }
      }

      // Did the temperature overshoot very far?
      // 当前温度超过 目标温度 + pid调节温度范围，则报错并结束
      #ifndef MAX_OVERSHOOT_PID_AUTOTUNE
        #define MAX_OVERSHOOT_PID_AUTOTUNE 30
      #endif
      if (current_temp > target + MAX_OVERSHOOT_PID_AUTOTUNE) {
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
        TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_TEMP_TOO_HIGH));
        TERN_(DWIN_LCD_PROUI, DWIN_PidTuning(PID_TEMP_TOO_HIGH));
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH)));
        break;
      }

      // Report heater states every 2 seconds
      // 每2s报告温度，并检测pid加热情况，温度变化没超过next_watch_temp报错 下降过快也报错
      if (ELAPSED(ms, next_temp_ms)) {
        #if HAS_TEMP_SENSOR
          print_heater_states(heater_id < 0 ? active_extruder : (int8_t)heater_id);
          SERIAL_EOL();
        #endif
        next_temp_ms = ms + 2000UL;

        // Make sure heating is actually working
        #if WATCH_PID
          if (ALL(WATCH_BED, WATCH_HOTENDS) || isbed == DISABLED(WATCH_HOTENDS) || ischamber == DISABLED(WATCH_HOTENDS)) {
            if (!heated) {                                            // If not yet reached target...
              if (current_temp > next_watch_temp) {                   // Over the watch temp?
                next_watch_temp = current_temp + watch_temp_increase; // - set the next temp to watch for
                temp_change_ms = ms + SEC_TO_MS(watch_temp_period);   // - move the expiration timer up
                if (current_temp > watch_temp_target) heated = true;  // - Flag if target temperature reached
              }
              else if (ELAPSED(ms, temp_change_ms))                   // Watch timer expired
                _temp_error(heater_id, FPSTR(str_t_heating_failed), GET_TEXT_F(MSG_HEATING_FAILED_LCD));
            }
            else if (current_temp < target - (MAX_OVERSHOOT_PID_AUTOTUNE)) // Heated, then temperature fell too far?
              _temp_error(heater_id, FPSTR(str_t_thermal_runaway), GET_TEXT_F(MSG_THERMAL_RUNAWAY));
          }
        #endif
      } // every 2 seconds

      // Timeout after MAX_CYCLE_TIME_PID_AUTOTUNE minutes since the last undershoot/overshoot cycle
      // 据t1 t2 超过20分钟
      #ifndef MAX_CYCLE_TIME_PID_AUTOTUNE
        #define MAX_CYCLE_TIME_PID_AUTOTUNE 20L
      #endif
      if ((ms - _MIN(t1, t2)) > (MAX_CYCLE_TIME_PID_AUTOTUNE * 60L * 1000L)) {
        TERN_(DWIN_CREALITY_LCD, DWIN_Popup_Temperature(0));
        TERN_(DWIN_LCD_PROUI, DWIN_PidTuning(PID_TUNING_TIMEOUT));
        TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_TUNING_TIMEOUT));
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TIMEOUT)));
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TIMEOUT);
        break;
      }
      
      // 自动 调节 pid 次数到达 ，取结果，结束循环
      if (cycles > ncycles && cycles > 2) {
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_FINISHED);
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_AUTOTUNE_DONE)));

        #if ANY(PIDTEMPBED, PIDTEMPCHAMBER)
          FSTR_P const estring = GHV(F("chamber"), F("bed"), FPSTR(NUL_STR));
          say_default_(); SERIAL_ECHOF(estring); SERIAL_ECHOLNPGM("Kp ", tune_pid.p);
          say_default_(); SERIAL_ECHOF(estring); SERIAL_ECHOLNPGM("Ki ", tune_pid.i);
          say_default_(); SERIAL_ECHOF(estring); SERIAL_ECHOLNPGM("Kd ", tune_pid.d);
        #else
          say_default_(); SERIAL_ECHOLNPGM("Kp ", tune_pid.p);
          say_default_(); SERIAL_ECHOLNPGM("Ki ", tune_pid.i);
          say_default_(); SERIAL_ECHOLNPGM("Kd ", tune_pid.d);
        #endif

        auto _set_hotend_pid = [](const uint8_t tool, const raw_pid_t &in_pid) {
          #if ENABLED(PIDTEMP)
            #if ENABLED(PID_PARAMS_PER_HOTEND)
              thermalManager.temp_hotend[tool].pid.set(in_pid);
            #else
              HOTEND_LOOP() temp_hotend[e].pid.set(in_pid);
            #endif
            updatePID();
          #endif
          UNUSED(tool); UNUSED(in_pid);
        };

        #if ENABLED(PIDTEMPBED)
          auto _set_bed_pid = [](const raw_pid_t &in_pid) {
            temp_bed.pid.set(in_pid);
          };
        #endif

        #if ENABLED(PIDTEMPCHAMBER)
          auto _set_chamber_pid = [](const raw_pid_t &in_pid) {
            temp_chamber.pid.set(in_pid);
          };
        #endif

        // Use the result? (As with "M303 U1")
        // 是否使用 测试后的pid
        if (set_result)
          GHV(_set_chamber_pid(tune_pid), _set_bed_pid(tune_pid), _set_hotend_pid(heater_id, tune_pid));

        TERN_(PRINTER_EVENT_LEDS, printerEventLEDs.onPidTuningDone(color));

        TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_DONE));
        TERN_(DWIN_LCD_PROUI, DWIN_PidTuning(PID_DONE));

        goto EXIT_M303;
      }

      // Run HAL idle tasks
      hal_idletask();

      // Run UI update
      // TERN(DWIN_CREALITY_LCD, DWIN_Update(), ui.update());
    }
    wait_for_heatup = false;

    temperature_disable_all_heaters();

    TERN_(PRINTER_EVENT_LEDS, printerEventLEDs.onPidTuningDone(color));

    TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_DONE));
    TERN_(DWIN_LCD_PROUI, DWIN_PidTuning(PID_DONE));

    EXIT_M303:
      TERN_(NO_FAN_SLOWING_IN_PID_TUNING, adaptive_fan_slowing = true);
      return;
  }

#endif // HAS_PID_HEATING