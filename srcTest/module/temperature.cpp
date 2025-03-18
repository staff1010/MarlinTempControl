/**
 * temperature.cpp - temperature control
 */

#include "temperature.h"

/**
 * Initialize the temperature manager
 *
 * The manager is implemented by periodic calls to task()
 *
 *  - Init (and disable) SPI thermocouples like MAX6675 and MAX31865
 *  - Disable RUMBA JTAG to accommodate a thermocouple extension
 *  - Read-enable thermistors with a read-enable pin
 *  - Init HEATER and COOLER pins for OUTPUT in OFF state
 *  - Init the FAN pins as PWM or OUTPUT
 *  - Init the SPI interface for SPI thermocouples
 *  - Init ADC according to the HAL
 *  - Set thermistor pins to analog inputs according to the HAL
 *  - Start the Temperature ISR timer
 *  - Init the AUTO FAN pins as PWM or OUTPUT
 *  - Wait 250ms for temperatures to settle
 *  - Init temp_range[], used for catching min/maxtemp
 */
void Temperature::init() {

    OUT_WRITE(HEATER_0_PIN, false);

    hal.adc_init();
  
    hal.adc_enable(TEMP_0_PIN);
  
    HAL_timer_start(MF_TIMER_TEMP, TEMP_TIMER_FREQUENCY);
    ENABLE_TEMPERATURE_INTERRUPT();
  
    #define _TEMP_MIN_E(NR) do{ \
    const celsius_t tmin_tmp = TERN(TEMP_SENSOR_##NR##_IS_CUSTOM, 0, int16_t(pgm_read_word(&TEMPTABLE_##NR [TEMP_SENSOR_##NR##_MINTEMP_IND].celsius))), \
                    tmin = _MAX(HEATER_##NR##_MINTEMP, tmin_tmp); \
    temp_range[NR].mintemp = tmin; \
    while (analog_to_celsius_hotend(temp_range[NR].raw_min, NR) < tmin) \
        temp_range[NR].raw_min += TEMPDIR(NR) * (OVERSAMPLENR); \
    }while(0)
    #define _TEMP_MAX_E(NR) do{ \
    const celsius_t tmax_tmp = TERN(TEMP_SENSOR_##NR##_IS_CUSTOM, 2000, int16_t(pgm_read_word(&TEMPTABLE_##NR [TEMP_SENSOR_##NR##_MAXTEMP_IND].celsius)) - 1), \
                    tmax = _MIN(HEATER_##NR##_MAXTEMP, tmax_tmp); \
    temp_range[NR].maxtemp = tmax; \
    while (analog_to_celsius_hotend(temp_range[NR].raw_max, NR) > tmax) \
        temp_range[NR].raw_max -= TEMPDIR(NR) * (OVERSAMPLENR); \
    }while(0)

    #define _MINMAX_TEST(N,M) (HOTENDS > N && TEMP_SENSOR(N) > 0 && TEMP_SENSOR(N) != 998 && TEMP_SENSOR(N) != 999 && defined(HEATER_##N##_##M##TEMP))

    _TEMP_MIN_E(0);

    _TEMP_MAX_E(0);

}