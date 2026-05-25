#include "power.h"
#include "main.h"
#include "my_print.h"

/** @brief STM32L4 VDDA supply voltage in millivolts (3.3 V). */
#define VDDA_MV        3300.0f
/** @brief ADC full-scale count for an 8-bit result register. */
#define ADC_FULL_SCALE  255.0f
/** @brief Resistive divider ratio on the battery sense line (e.g. 2:3 → 1.5). */
#define DIVIDER_RATIO     1.5f

static ADC_ChannelConfTypeDef adc_ch_cfg = {
    .Channel      = ADC_CHANNEL_10,
    .Rank         = ADC_REGULAR_RANK_1,
    .SamplingTime = ADC_SAMPLETIME_640CYCLES_5,
    .SingleDiff   = ADC_SINGLE_ENDED,
    .OffsetNumber = ADC_OFFSET_NONE,
    .Offset       = 0,
};

static BatteryStatus_t battery_status;

/**
 * @brief Sample GPIO charge-status pins and perform one ADC battery voltage conversion.
 *
 * Execution sequence:
 * 1. Wakes the ADC from deep power-down and enables the internal voltage regulator.
 * 2. Reads the three charge-status GPIO pins (VBUS sense, charge-indicator, standby).
 * 3. Waits 1 ms for the ADC regulator to stabilise (datasheet t_start requirement).
 * 4. Runs a self-calibration (~3 µs at 312 kHz ADC clock), configures the channel,
 *    and starts a single conversion.
 * 5. Polls for completion (10 ms timeout; typical ~2 ms at 640.5-cycle sample time).
 *    On success, converts the raw count to volts via @c VDDA_MV and @c DIVIDER_RATIO.
 *    On timeout, stores -1.0 V as a conversion-failure sentinel.
 * 6. Stops the ADC and returns it to deep power-down (~0.3 µA quiescent).
 *
 * Results are stored in the module-static @c battery_status struct and
 * accessed via @ref power_get_status.
 *
 * @note Must not be called from ISR context — uses @c osDelay and blocking HAL calls.
 */
void power_read(void)
{
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);

    /* Read charge-status GPIO pins before the ADC conversion */
    battery_status.usb_connected  = (HAL_GPIO_ReadPin(VBUS_sense_GPIO_Port,  VBUS_sense_Pin)   == GPIO_PIN_SET);
    battery_status.is_charging    = (HAL_GPIO_ReadPin(CHRG_IND_GPIO_Port,    CHRG_IND_Pin)    == GPIO_PIN_RESET);
    battery_status.charge_complete = (HAL_GPIO_ReadPin(CHRG_STDBY_GPIO_Port, CHRG_STDBY_Pin)  == GPIO_PIN_RESET);

    osDelay(1); /* ADC internal regulator settling (t_start per datasheet) */

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); /* ~3 µs @ 312 kHz ADC clk */

    HAL_ADC_ConfigChannel(&hadc1, &adc_ch_cfg);
    HAL_ADC_Start(&hadc1);

    uint32_t raw = 0;

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) /* timeout 10 ms; typical ~2 ms */
    {
        raw = HAL_ADC_GetValue(&hadc1);
        float v_adc_mv = ((float)raw / ADC_FULL_SCALE) * VDDA_MV;
        battery_status.battery_voltage_V = (v_adc_mv * DIVIDER_RATIO) / 1000.0f;
    }
    else
    {
        battery_status.battery_voltage_V = -1.0f; /* sentinel: conversion failed */
    }

    HAL_ADC_Stop(&hadc1);
    LL_ADC_DisableInternalRegulator(ADC1);
    LL_ADC_EnableDeepPowerDown(ADC1); /* back to ~0.3 µA quiescent */

#ifdef UWB_DEBUG
    mprintf("PWR: %.2fV, raw:%04X | USB:%d | CHG:%d | STBY:%d\r\n",
            battery_status.battery_voltage_V,
            raw,
            battery_status.usb_connected,
            battery_status.is_charging,
            battery_status.charge_complete);
#endif
}

/**
 * @brief Return a pointer to the most recently sampled battery status.
 *
 * The pointed-to struct is updated on each call to @ref power_read.
 * The caller must not modify the returned struct.
 *
 * @return Const pointer to the internal @c BatteryStatus_t.
 */
const BatteryStatus_t *power_get_status(void)
{
    return &battery_status;
}