#include "power.h"
#include "main.h"
#include "my_print.h"

#define VDDA_MV         3300.0f
#define ADC_FULL_SCALE  255.0f
#define DIVIDER_RATIO   1.5f    

static ADC_ChannelConfTypeDef adc_ch_cfg = {
    .Channel      = ADC_CHANNEL_10,
    .Rank         = ADC_REGULAR_RANK_1,
    .SamplingTime = ADC_SAMPLETIME_640CYCLES_5,
    .SingleDiff   = ADC_SINGLE_ENDED,
    .OffsetNumber = ADC_OFFSET_NONE,
    .Offset       = 0,
};

BatteryStatus_t battery_status;

void power_read(void)
{
    /* --- GPIO reads (instantaneous) --- */
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    
    battery_status.usb_connected  = (HAL_GPIO_ReadPin(VBUS_sense_GPIO_Port, VBUS_sense_Pin) == GPIO_PIN_SET);
    battery_status.is_charging    = (HAL_GPIO_ReadPin(CHRG_IND_GPIO_Port, CHRG_IND_Pin) == GPIO_PIN_RESET);
    battery_status.charge_complete= (HAL_GPIO_ReadPin(CHRG_STDBY_GPIO_Port, CHRG_STDBY_Pin) == GPIO_PIN_RESET);

    osDelay(1);    

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);   // ~3 µs @ 312 kHz ADC clk

    HAL_ADC_ConfigChannel(&hadc1, &adc_ch_cfg);
    HAL_ADC_Start(&hadc1);

    uint32_t raw = 0;

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)    // timeout 10 ms (actual ~2 ms)
    {
        raw = HAL_ADC_GetValue(&hadc1);
        float v_adc_mv = ((float)raw / ADC_FULL_SCALE) * VDDA_MV;
        battery_status.battery_voltage_V = (v_adc_mv * DIVIDER_RATIO) / 1000.0f;
    }
    else
    {
        battery_status.battery_voltage_V = -1.0f;              // flag conversion failure
    }

    HAL_ADC_Stop(&hadc1);
    LL_ADC_DisableInternalRegulator(ADC1);
    LL_ADC_EnableDeepPowerDown(ADC1);                   // back to ~0.3 µA

    mprintf("PWR: %.2fV, raw:%04X | USB:%d | CHG:%d | STBY:%d\r\n",
        battery_status.battery_voltage_V,
        raw,
        battery_status.usb_connected,
        battery_status.is_charging,
        battery_status.charge_complete);
}

const BatteryStatus_t* power_get_status(void)
{
    return &battery_status;
}