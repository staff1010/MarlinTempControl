#pragma once
#include "../inc/Config.h"

/**
 * temperature.h - temperature controller
 */

typedef struct TempInfo {
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

class Temperature {

    public:
  
      #if HAS_HOTEND
        static hotend_info_t temp_hotend[HOTENDS];
        static constexpr celsius_t hotend_maxtemp[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP, HEATER_4_MAXTEMP, HEATER_5_MAXTEMP, HEATER_6_MAXTEMP, HEATER_7_MAXTEMP);
        static celsius_t hotend_max_target(const uint8_t e) { return hotend_maxtemp[e] - (HOTEND_OVERSHOOT); }
      #endif
  
      #if HAS_HEATED_BED
        static bed_info_t temp_bed;
      #endif
      #if HAS_TEMP_PROBE
        static probe_info_t temp_probe;
      #endif
      #if HAS_TEMP_CHAMBER
        static chamber_info_t temp_chamber;
      #endif
      #if HAS_TEMP_COOLER
        static cooler_info_t temp_cooler;
      #endif
      #if HAS_TEMP_BOARD
        static board_info_t temp_board;
      #endif
      #if HAS_TEMP_REDUNDANT
        static redundant_info_t temp_redundant;
      #endif
  
      #if EITHER(AUTO_POWER_E_FANS, HAS_FANCHECK)
        static uint8_t autofan_speed[HOTENDS];
      #endif
      #if ENABLED(AUTO_POWER_CHAMBER_FAN)
        static uint8_t chamberfan_speed;
      #endif
      #if ENABLED(AUTO_POWER_COOLER_FAN)
        static uint8_t coolerfan_speed;
      #endif
  
      #if ENABLED(FAN_SOFT_PWM)
        static uint8_t soft_pwm_amount_fan[FAN_COUNT],
                       soft_pwm_count_fan[FAN_COUNT];
      #endif
  
      #if BOTH(FAN_SOFT_PWM, USE_CONTROLLER_FAN)
        static uint8_t soft_pwm_controller_speed;
      #endif
  
      #if BOTH(HAS_MARLINUI_MENU, PREVENT_COLD_EXTRUSION) && E_MANUAL > 0
        static bool allow_cold_extrude_override;
        static void set_menu_cold_override(const bool allow) { allow_cold_extrude_override = allow; }
      #else
        static constexpr bool allow_cold_extrude_override = false;
        static void set_menu_cold_override(const bool) {}
      #endif
  
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        static bool allow_cold_extrude;
        static celsius_t extrude_min_temp;
        static bool tooCold(const celsius_t temp) { return !allow_cold_extrude && !allow_cold_extrude_override && temp < extrude_min_temp - (TEMP_WINDOW); }
        static bool tooColdToExtrude(const uint8_t E_NAME)       { return tooCold(wholeDegHotend(HOTEND_INDEX)); }
        static bool targetTooColdToExtrude(const uint8_t E_NAME) { return tooCold(degTargetHotend(HOTEND_INDEX)); }
      #else
        static bool tooColdToExtrude(const uint8_t) { return false; }
        static bool targetTooColdToExtrude(const uint8_t) { return false; }
      #endif
  
      static bool hotEnoughToExtrude(const uint8_t e) { return !tooColdToExtrude(e); }
      static bool targetHotEnoughToExtrude(const uint8_t e) { return !targetTooColdToExtrude(e); }
  
      #if EITHER(SINGLENOZZLE_STANDBY_TEMP, SINGLENOZZLE_STANDBY_FAN)
        #if ENABLED(SINGLENOZZLE_STANDBY_TEMP)
          static celsius_t singlenozzle_temp[EXTRUDERS];
        #endif
        #if ENABLED(SINGLENOZZLE_STANDBY_FAN)
          static uint8_t singlenozzle_fan_speed[EXTRUDERS];
        #endif
        static void singlenozzle_change(const uint8_t old_tool, const uint8_t new_tool);
      #endif
  
      #if HEATER_IDLE_HANDLER
  
        // Heater idle handling. Marlin creates one per hotend and one for the heated bed.
        typedef struct {
          millis_t timeout_ms;
          bool timed_out;
          inline void update(const millis_t &ms) { if (!timed_out && timeout_ms && ELAPSED(ms, timeout_ms)) timed_out = true; }
          inline void start(const millis_t &ms) { timeout_ms = millis() + ms; timed_out = false; }
          inline void reset() { timeout_ms = 0; timed_out = false; }
          inline void expire() { start(0); }
        } heater_idle_t;
  
        // Indices and size for the heater_idle array
        enum IdleIndex : int8_t {
          _II = -1
  
          #define _IDLE_INDEX_E(N) ,IDLE_INDEX_E##N
          REPEAT(HOTENDS, _IDLE_INDEX_E)
          #undef _IDLE_INDEX_E
  
          OPTARG(HAS_HEATED_BED, IDLE_INDEX_BED)
  
          , NR_HEATER_IDLE
        };
  
        // Convert the given heater_id_t to idle array index
        static IdleIndex idle_index_for_id(const int8_t heater_id) {
          TERN_(HAS_HEATED_BED, if (heater_id == H_BED) return IDLE_INDEX_BED);
          return (IdleIndex)_MAX(heater_id, 0);
        }
  
        static heater_idle_t heater_idle[NR_HEATER_IDLE];
  
      #endif // HEATER_IDLE_TIMER
  
      #if HAS_ADC_BUTTONS
        static uint32_t current_ADCKey_raw;
        static uint16_t ADCKey_count;
      #endif
  
      #if ENABLED(PID_EXTRUSION_SCALING)
        static int16_t lpq_len;
      #endif
  
      #if HAS_FAN_LOGIC
        static constexpr millis_t fan_update_interval_ms = TERN(HAS_PWMFANCHECK, 5000, TERN(HAS_FANCHECK, 1000, 2500));
      #endif
  
    private:
  
      #if ENABLED(WATCH_HOTENDS)
        static hotend_watch_t watch_hotend[HOTENDS];
      #endif
  
      #if ENABLED(MPCTEMP)
        static int32_t mpc_e_position;
      #endif
  
      #if HAS_HOTEND
        static temp_range_t temp_range[HOTENDS];
      #endif
  
      #if HAS_HEATED_BED
        #if WATCH_BED
          static bed_watch_t watch_bed;
        #endif
        #if DISABLED(PIDTEMPBED)
          static millis_t next_bed_check_ms;
        #endif
        static raw_adc_t mintemp_raw_BED, maxtemp_raw_BED;
      #endif
  
      #if HAS_HEATED_CHAMBER
        #if WATCH_CHAMBER
          static chamber_watch_t watch_chamber;
        #endif
        #if DISABLED(PIDTEMPCHAMBER)
          static millis_t next_chamber_check_ms;
        #endif
        static raw_adc_t mintemp_raw_CHAMBER, maxtemp_raw_CHAMBER;
      #endif
  
      #if HAS_COOLER
        #if WATCH_COOLER
          static cooler_watch_t watch_cooler;
        #endif
        static millis_t next_cooler_check_ms, cooler_fan_flush_ms;
        static raw_adc_t mintemp_raw_COOLER, maxtemp_raw_COOLER;
      #endif
  
      #if HAS_TEMP_BOARD && ENABLED(THERMAL_PROTECTION_BOARD)
        static raw_adc_t mintemp_raw_BOARD, maxtemp_raw_BOARD;
      #endif
  
      #if MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED > 1
        static uint8_t consecutive_low_temperature_error[HOTENDS];
      #endif
  
      #if MILLISECONDS_PREHEAT_TIME > 0
        static millis_t preheat_end_time[HOTENDS];
      #endif
  
      #if HAS_FAN_LOGIC
        static millis_t fan_update_ms;
  
        static void manage_extruder_fans(millis_t ms) {
          if (ELAPSED(ms, fan_update_ms)) { // only need to check fan state very infrequently
            const millis_t next_ms = ms + fan_update_interval_ms;
            #if HAS_PWMFANCHECK
              #define FAN_CHECK_DURATION 100
              if (fan_check.is_measuring()) {
                fan_check.compute_speed(ms + FAN_CHECK_DURATION - fan_update_ms);
                fan_update_ms = next_ms;
              }
              else
                fan_update_ms = ms + FAN_CHECK_DURATION;
              fan_check.toggle_measuring();
            #else
              TERN_(HAS_FANCHECK, fan_check.compute_speed(next_ms - fan_update_ms));
              fan_update_ms = next_ms;
            #endif
            TERN_(HAS_AUTO_FAN, update_autofans()); // Needed as last when HAS_PWMFANCHECK to properly force fan speed
          }
        }
      #endif
  
      #if ENABLED(PROBING_HEATERS_OFF)
        static bool paused_for_probing;
      #endif
  
    public:
      /**
       * Instance Methods
       */
  
      void init();
  
      /**
       * Static (class) methods
       */
  
      #if HAS_USER_THERMISTORS
        static user_thermistor_t user_thermistor[USER_THERMISTORS];
        static void M305_report(const uint8_t t_index, const bool forReplay=true);
        static void reset_user_thermistors();
        static celsius_float_t user_thermistor_to_deg_c(const uint8_t t_index, const raw_adc_t raw);
        static bool set_pull_up_res(int8_t t_index, float value) {
          //if (!WITHIN(t_index, 0, USER_THERMISTORS - 1)) return false;
          if (!WITHIN(value, 1, 1000000)) return false;
          user_thermistor[t_index].series_res = value;
          return true;
        }
        static bool set_res25(int8_t t_index, float value) {
          if (!WITHIN(value, 1, 10000000)) return false;
          user_thermistor[t_index].res_25 = value;
          user_thermistor[t_index].pre_calc = true;
          return true;
        }
        static bool set_beta(int8_t t_index, float value) {
          if (!WITHIN(value, 1, 1000000)) return false;
          user_thermistor[t_index].beta = value;
          user_thermistor[t_index].pre_calc = true;
          return true;
        }
        static bool set_sh_coeff(int8_t t_index, float value) {
          if (!WITHIN(value, -0.01f, 0.01f)) return false;
          user_thermistor[t_index].sh_c_coeff = value;
          user_thermistor[t_index].pre_calc = true;
          return true;
        }
      #endif
  
      #if HAS_HOTEND
        static celsius_float_t analog_to_celsius_hotend(const raw_adc_t raw, const uint8_t e);
      #endif
      #if HAS_HEATED_BED
        static celsius_float_t analog_to_celsius_bed(const raw_adc_t raw);
      #endif
      #if HAS_TEMP_CHAMBER
        static celsius_float_t analog_to_celsius_chamber(const raw_adc_t raw);
      #endif
      #if HAS_TEMP_PROBE
        static celsius_float_t analog_to_celsius_probe(const raw_adc_t raw);
      #endif
      #if HAS_TEMP_COOLER
        static celsius_float_t analog_to_celsius_cooler(const raw_adc_t raw);
      #endif
      #if HAS_TEMP_BOARD
        static celsius_float_t analog_to_celsius_board(const raw_adc_t raw);
      #endif
      #if HAS_TEMP_REDUNDANT
        static celsius_float_t analog_to_celsius_redundant(const raw_adc_t raw);
      #endif
  
      #if HAS_FAN
  
        static uint8_t fan_speed[FAN_COUNT];
        #define FANS_LOOP(I) LOOP_L_N(I, FAN_COUNT)
  
        static void set_fan_speed(const uint8_t fan, const uint16_t speed);
  
        #if ENABLED(REPORT_FAN_CHANGE)
          static void report_fan_speed(const uint8_t fan);
        #endif
  
        #if EITHER(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
          static bool fans_paused;
          static uint8_t saved_fan_speed[FAN_COUNT];
        #endif
  
        #if ENABLED(ADAPTIVE_FAN_SLOWING)
          static uint8_t fan_speed_scaler[FAN_COUNT];
        #endif
  
        static uint8_t scaledFanSpeed(const uint8_t fan, const uint8_t fs) {
          UNUSED(fan); // Potentially unused!
          return (fs * uint16_t(TERN(ADAPTIVE_FAN_SLOWING, fan_speed_scaler[fan], 128))) >> 7;
        }
  
        static uint8_t scaledFanSpeed(const uint8_t fan) {
          return scaledFanSpeed(fan, fan_speed[fan]);
        }
  
        static constexpr inline uint8_t pwmToPercent(const uint8_t speed) { return ui8_to_percent(speed); }
        static uint8_t fanSpeedPercent(const uint8_t fan)          { return ui8_to_percent(fan_speed[fan]); }
        static uint8_t scaledFanSpeedPercent(const uint8_t fan)    { return ui8_to_percent(scaledFanSpeed(fan)); }
  
        #if ENABLED(EXTRA_FAN_SPEED)
          typedef struct { uint8_t saved, speed; } extra_fan_t;
          static extra_fan_t extra_fan_speed[FAN_COUNT];
          static void set_temp_fan_speed(const uint8_t fan, const uint16_t command_or_speed);
        #endif
  
        #if EITHER(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
          void set_fans_paused(const bool p);
        #endif
  
      #endif // HAS_FAN
  
      static void zero_fan_speeds() {
        #if HAS_FAN
          FANS_LOOP(i) set_fan_speed(i, 0);
        #endif
      }
  
      /**
       * Called from the Temperature ISR
       */
      static void isr();
      static void readings_ready();
  
      /**
       * Call periodically to manage heaters and keep the watchdog fed
       */
      static void task();
  
      /**
       * Preheating hotends
       */
      #if MILLISECONDS_PREHEAT_TIME > 0
        static bool is_preheating(const uint8_t E_NAME) {
          return preheat_end_time[HOTEND_INDEX] && PENDING(millis(), preheat_end_time[HOTEND_INDEX]);
        }
        static void start_preheat_time(const uint8_t E_NAME) {
          preheat_end_time[HOTEND_INDEX] = millis() + MILLISECONDS_PREHEAT_TIME;
        }
        static void reset_preheat_time(const uint8_t E_NAME) {
          preheat_end_time[HOTEND_INDEX] = 0;
        }
      #else
        #define is_preheating(n) (false)
      #endif
  
      //high level conversion routines, for use outside of temperature.cpp
      //inline so that there is no performance decrease.
      //deg=degreeCelsius
  
      static celsius_float_t degHotend(const uint8_t E_NAME) {
        return TERN0(HAS_HOTEND, temp_hotend[HOTEND_INDEX].celsius);
      }
  
      static celsius_t wholeDegHotend(const uint8_t E_NAME) {
        return TERN0(HAS_HOTEND, static_cast<celsius_t>(temp_hotend[HOTEND_INDEX].celsius + 0.5f));
      }
  
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawHotendTemp(const uint8_t E_NAME) {
          return TERN0(HAS_HOTEND, temp_hotend[HOTEND_INDEX].getraw());
        }
      #endif
  
      static celsius_t degTargetHotend(const uint8_t E_NAME) {
        return TERN0(HAS_HOTEND, temp_hotend[HOTEND_INDEX].target);
      }
  
      #if HAS_HOTEND
  
        static void setTargetHotend(const celsius_t celsius, const uint8_t E_NAME) {
          const uint8_t ee = HOTEND_INDEX;
          #if MILLISECONDS_PREHEAT_TIME > 0
            if (celsius == 0)
              reset_preheat_time(ee);
            else if (temp_hotend[ee].target == 0)
              start_preheat_time(ee);
          #endif
          TERN_(AUTO_POWER_CONTROL, if (celsius) powerManager.power_on());
          temp_hotend[ee].target = _MIN(celsius, hotend_max_target(ee));
          start_watching_hotend(ee);
        }
  
        static bool isHeatingHotend(const uint8_t E_NAME) {
          return temp_hotend[HOTEND_INDEX].target > temp_hotend[HOTEND_INDEX].celsius;
        }
  
        static bool isCoolingHotend(const uint8_t E_NAME) {
          return temp_hotend[HOTEND_INDEX].target < temp_hotend[HOTEND_INDEX].celsius;
        }
  
        #if HAS_TEMP_HOTEND
          static bool wait_for_hotend(const uint8_t target_extruder, const bool no_wait_for_cooling=true
            OPTARG(G26_CLICK_CAN_CANCEL, const bool click_to_cancel=false)
          );
  
          #if ENABLED(WAIT_FOR_HOTEND)
            static void wait_for_hotend_heating(const uint8_t target_extruder);
          #endif
        #endif
  
        static bool still_heating(const uint8_t e) {
          return degTargetHotend(e) > TEMP_HYSTERESIS && ABS(wholeDegHotend(e) - degTargetHotend(e)) > TEMP_HYSTERESIS;
        }
  
        static bool degHotendNear(const uint8_t e, const celsius_t temp) {
          return ABS(wholeDegHotend(e) - temp) < (TEMP_HYSTERESIS);
        }
  
        // Start watching a Hotend to make sure it's really heating up
        static void start_watching_hotend(const uint8_t E_NAME) {
          UNUSED(HOTEND_INDEX);
          #if WATCH_HOTENDS
            watch_hotend[HOTEND_INDEX].restart(degHotend(HOTEND_INDEX), degTargetHotend(HOTEND_INDEX));
          #endif
        }
  
        static void manage_hotends(const millis_t &ms);
  
      #endif // HAS_HOTEND
  
      #if HAS_HEATED_BED
  
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          static raw_adc_t rawBedTemp()  { return temp_bed.getraw(); }
        #endif
        static celsius_float_t degBed()  { return temp_bed.celsius; }
        static celsius_t wholeDegBed()   { return static_cast<celsius_t>(degBed() + 0.5f); }
        static celsius_t degTargetBed()  { return temp_bed.target; }
        static bool isHeatingBed()       { return temp_bed.target > temp_bed.celsius; }
        static bool isCoolingBed()       { return temp_bed.target < temp_bed.celsius; }
        static bool degBedNear(const celsius_t temp) {
          return ABS(wholeDegBed() - temp) < (TEMP_BED_HYSTERESIS);
        }
  
        // Start watching the Bed to make sure it's really heating up
        static void start_watching_bed() { TERN_(WATCH_BED, watch_bed.restart(degBed(), degTargetBed())); }
  
        static void setTargetBed(const celsius_t celsius) {
          TERN_(AUTO_POWER_CONTROL, if (celsius) powerManager.power_on());
          temp_bed.target = _MIN(celsius, BED_MAX_TARGET);
          start_watching_bed();
        }
  
        static bool wait_for_bed(const bool no_wait_for_cooling=true
          OPTARG(G26_CLICK_CAN_CANCEL, const bool click_to_cancel=false)
        );
  
        static void wait_for_bed_heating();
  
        static void manage_heated_bed(const millis_t &ms);
  
      #endif // HAS_HEATED_BED
  
      #if HAS_TEMP_PROBE
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          static raw_adc_t rawProbeTemp()  { return temp_probe.getraw(); }
        #endif
        static celsius_float_t degProbe()  { return temp_probe.celsius; }
        static celsius_t wholeDegProbe()   { return static_cast<celsius_t>(degProbe() + 0.5f); }
        static bool isProbeBelowTemp(const celsius_t target_temp) { return wholeDegProbe() < target_temp; }
        static bool isProbeAboveTemp(const celsius_t target_temp) { return wholeDegProbe() > target_temp; }
        static bool wait_for_probe(const celsius_t target_temp, bool no_wait_for_cooling=true);
      #endif
  
      #if HAS_TEMP_CHAMBER
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          static raw_adc_t rawChamberTemp()    { return temp_chamber.getraw(); }
        #endif
        static celsius_float_t degChamber()    { return temp_chamber.celsius; }
        static celsius_t wholeDegChamber()     { return static_cast<celsius_t>(degChamber() + 0.5f); }
        #if HAS_HEATED_CHAMBER
          static celsius_t degTargetChamber()  { return temp_chamber.target; }
          static bool isHeatingChamber()       { return temp_chamber.target > temp_chamber.celsius; }
          static bool isCoolingChamber()       { return temp_chamber.target < temp_chamber.celsius; }
          static bool wait_for_chamber(const bool no_wait_for_cooling=true);
          static void manage_heated_chamber(const millis_t &ms);
        #endif
      #endif
  
      #if HAS_HEATED_CHAMBER
        static void setTargetChamber(const celsius_t celsius) {
          temp_chamber.target = _MIN(celsius, CHAMBER_MAX_TARGET);
          start_watching_chamber();
        }
        // Start watching the Chamber to make sure it's really heating up
        static void start_watching_chamber() { TERN_(WATCH_CHAMBER, watch_chamber.restart(degChamber(), degTargetChamber())); }
      #endif
  
      #if HAS_TEMP_COOLER
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          static raw_adc_t rawCoolerTemp()   { return temp_cooler.getraw(); }
        #endif
        static celsius_float_t degCooler()   { return temp_cooler.celsius; }
        static celsius_t wholeDegCooler()    { return static_cast<celsius_t>(temp_cooler.celsius + 0.5f); }
        #if HAS_COOLER
          static celsius_t degTargetCooler() { return temp_cooler.target; }
          static bool isLaserHeating()       { return temp_cooler.target > temp_cooler.celsius; }
          static bool isLaserCooling()       { return temp_cooler.target < temp_cooler.celsius; }
          static bool wait_for_cooler(const bool no_wait_for_cooling=true);
          static void manage_cooler(const millis_t &ms);
        #endif
      #endif
  
      #if HAS_TEMP_BOARD
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          static raw_adc_t rawBoardTemp()  { return temp_board.getraw(); }
        #endif
        static celsius_float_t degBoard()  { return temp_board.celsius; }
        static celsius_t wholeDegBoard()   { return static_cast<celsius_t>(temp_board.celsius + 0.5f); }
      #endif
  
      #if HAS_TEMP_REDUNDANT
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          static raw_adc_t rawRedundantTemp()       { return temp_redundant.getraw(); }
        #endif
        static celsius_float_t degRedundant()       { return temp_redundant.celsius; }
        static celsius_float_t degRedundantTarget() { return (*temp_redundant.target).celsius; }
        static celsius_t wholeDegRedundant()        { return static_cast<celsius_t>(temp_redundant.celsius + 0.5f); }
        static celsius_t wholeDegRedundantTarget()  { return static_cast<celsius_t>((*temp_redundant.target).celsius + 0.5f); }
      #endif
  
      #if HAS_COOLER
        static void setTargetCooler(const celsius_t celsius) {
          temp_cooler.target = constrain(celsius, COOLER_MIN_TARGET, COOLER_MAX_TARGET);
          start_watching_cooler();
        }
        // Start watching the Cooler to make sure it's really cooling down
        static void start_watching_cooler() { TERN_(WATCH_COOLER, watch_cooler.restart(degCooler(), degTargetCooler())); }
      #endif
  
      /**
       * The software PWM power for a heater
       */
      static int16_t getHeaterPower(const heater_id_t heater_id);
  
      /**
       * Switch off all heaters, set all target temperatures to 0
       */
      static void disable_all_heaters();
  
      /**
       * Cooldown, as from the LCD. Disables all heaters and fans.
       */
      static void cooldown() {
        zero_fan_speeds();
        disable_all_heaters();
      }
  
      #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
        /**
         * Methods to check if heaters are enabled, indicating an active job
         */
        static bool auto_job_over_threshold();
        static void auto_job_check_timer(const bool can_start, const bool can_stop);
      #endif
  
      /**
       * Perform auto-tuning for hotend or bed in response to M303
       */
      #if HAS_PID_HEATING
  
        #if HAS_PID_DEBUG
          static bool pid_debug_flag;
        #endif
  
        static void PID_autotune(const celsius_t target, const heater_id_t heater_id, const int8_t ncycles, const bool set_result=false);
  
        #if ENABLED(NO_FAN_SLOWING_IN_PID_TUNING)
          static bool adaptive_fan_slowing;
        #elif ENABLED(ADAPTIVE_FAN_SLOWING)
          static constexpr bool adaptive_fan_slowing = true;
        #endif
  
        // Update the temp manager when PID values change
        #if ENABLED(PIDTEMP)
          static void updatePID() { HOTEND_LOOP() temp_hotend[e].pid.reset(); }
          static void setPID(const uint8_t hotend, const_float_t p, const_float_t i, const_float_t d) {
            #if ENABLED(PID_PARAMS_PER_HOTEND)
              temp_hotend[hotend].pid.set(p, i, d);
            #else
              HOTEND_LOOP() temp_hotend[e].pid.set(p, i, d);
            #endif
            updatePID();
          }
        #endif
  
      #endif
  
      #if ENABLED(MPCTEMP)
        void MPC_autotune();
      #endif
  
      #if ENABLED(PROBING_HEATERS_OFF)
        static void pause_heaters(const bool p);
      #endif
  
      #if HEATER_IDLE_HANDLER
  
        static void reset_hotend_idle_timer(const uint8_t E_NAME) {
          heater_idle[HOTEND_INDEX].reset();
          start_watching_hotend(HOTEND_INDEX);
        }
  
        #if HAS_HEATED_BED
          static void reset_bed_idle_timer() {
            heater_idle[IDLE_INDEX_BED].reset();
            start_watching_bed();
          }
        #endif
  
      #endif // HEATER_IDLE_HANDLER
  
      #if HAS_TEMP_SENSOR
        static void print_heater_states(const int8_t target_extruder
          OPTARG(HAS_TEMP_REDUNDANT, const bool include_r=false)
        );
        #if ENABLED(AUTO_REPORT_TEMPERATURES)
          struct AutoReportTemp { static void report(); };
          static AutoReporter<AutoReportTemp> auto_reporter;
        #endif
      #endif
  
      #if HAS_HOTEND && HAS_STATUS_MESSAGE
        static void set_heating_message(const uint8_t e, const bool isM104=false);
      #else
        static void set_heating_message(const uint8_t, const bool=false) {}
      #endif
  
      #if HAS_MARLINUI_MENU && HAS_TEMPERATURE && HAS_PREHEAT
        static void lcd_preheat(const uint8_t e, const int8_t indh, const int8_t indb);
      #endif
  
    private:
  
      // Reading raw temperatures and converting to Celsius when ready
      static volatile bool raw_temps_ready;
      static void update_raw_temperatures();
      static void updateTemperaturesFromRawValues();
      static bool updateTemperaturesIfReady() {
        if (!raw_temps_ready) return false;
        updateTemperaturesFromRawValues();
        raw_temps_ready = false;
        return true;
      }
  
      // MAX Thermocouples
      #if HAS_MAX_TC
        #define MAX_TC_COUNT TEMP_SENSOR_IS_MAX_TC(0) + TEMP_SENSOR_IS_MAX_TC(1) + TEMP_SENSOR_IS_MAX_TC(REDUNDANT)
        #if MAX_TC_COUNT > 1
          #define HAS_MULTI_MAX_TC 1
          #define READ_MAX_TC(N) read_max_tc(N)
        #else
          #define READ_MAX_TC(N) read_max_tc()
        #endif
        static raw_adc_t read_max_tc(TERN_(HAS_MULTI_MAX_TC, const uint8_t hindex=0));
      #endif
  
      #if HAS_AUTO_FAN
        #if ENABLED(POWER_OFF_WAIT_FOR_COOLDOWN)
          static bool autofans_on;
        #endif
        static void update_autofans();
      #endif
  
      #if HAS_HOTEND
        static float get_pid_output_hotend(const uint8_t e);
      #endif
      #if ENABLED(PIDTEMPBED)
        static float get_pid_output_bed();
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        static float get_pid_output_chamber();
      #endif
  
      static void _temp_error(const heater_id_t e, FSTR_P const serial_msg, FSTR_P const lcd_msg);
      static void mintemp_error(const heater_id_t e);
      static void maxtemp_error(const heater_id_t e);
  
      #define HAS_THERMAL_PROTECTION ANY(THERMAL_PROTECTION_HOTENDS, THERMAL_PROTECTION_CHAMBER, THERMAL_PROTECTION_BED, THERMAL_PROTECTION_COOLER)
  
      #if HAS_THERMAL_PROTECTION
  
        // Indices and size for the tr_state_machine array. One for each protected heater.
        enum RunawayIndex : int8_t {
          _RI = -1
          #if ENABLED(THERMAL_PROTECTION_HOTENDS)
            #define _RUNAWAY_IND_E(N) ,RUNAWAY_IND_E##N
            REPEAT(HOTENDS, _RUNAWAY_IND_E)
            #undef _RUNAWAY_IND_E
          #endif
          OPTARG(THERMAL_PROTECTION_BED, RUNAWAY_IND_BED)
          OPTARG(THERMAL_PROTECTION_CHAMBER, RUNAWAY_IND_CHAMBER)
          OPTARG(THERMAL_PROTECTION_COOLER, RUNAWAY_IND_COOLER)
          , NR_HEATER_RUNAWAY
        };
  
        // Convert the given heater_id_t to runaway state array index
        static RunawayIndex runaway_index_for_id(const int8_t heater_id) {
          TERN_(THERMAL_PROTECTION_CHAMBER, if (heater_id == H_CHAMBER) return RUNAWAY_IND_CHAMBER);
          TERN_(THERMAL_PROTECTION_COOLER,  if (heater_id == H_COOLER)  return RUNAWAY_IND_COOLER);
          TERN_(THERMAL_PROTECTION_BED,     if (heater_id == H_BED)     return RUNAWAY_IND_BED);
          return (RunawayIndex)_MAX(heater_id, 0);
        }
  
        enum TRState : char { TRInactive, TRFirstHeating, TRStable, TRRunaway
          OPTARG(THERMAL_PROTECTION_VARIANCE_MONITOR, TRMalfunction)
        };
  
        typedef struct {
          millis_t timer = 0;
          TRState state = TRInactive;
          celsius_float_t running_temp;
          #if ENABLED(THERMAL_PROTECTION_VARIANCE_MONITOR)
            millis_t variance_timer = 0;
            celsius_float_t last_temp = 0.0, variance = 0.0;
          #endif
          void run(const_celsius_float_t current, const_celsius_float_t target, const heater_id_t heater_id, const uint16_t period_seconds, const celsius_float_t hysteresis_degc);
        } tr_state_machine_t;
  
        static tr_state_machine_t tr_state_machine[NR_HEATER_RUNAWAY];
  
      #endif // HAS_THERMAL_PROTECTION
  };
  
  extern Temperature thermalManager;
  