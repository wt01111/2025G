#include "rlc_app.h"
#include "rlc_display.h"
#include "rlc_filter.h"

typedef enum
{
  RLC_APP_STATE_IDLE = 0U,
  RLC_APP_STATE_LEARNING,
  RLC_APP_STATE_FILTERING
} rlc_app_state_t;

static rlc_app_state_t app_state = RLC_APP_STATE_IDLE;

void rlc_app_init(void)
{
  rlc_filter_init();
  rlc_display_init();
  app_state = RLC_APP_STATE_IDLE;
  rlc_display_debug_puts("CMD_READY,start_learn,start_filter\r\n");
}

void rlc_app_task(void)
{
  if (rlc_display_take_start_learn() != 0U)
  {
    if (app_state == RLC_APP_STATE_FILTERING)
    {
      rlc_filter_stop_realtime();
    }

    app_state = RLC_APP_STATE_LEARNING;
    rlc_filter_start_learning();
    app_state = RLC_APP_STATE_IDLE;
    return;
  }

  if ((rlc_display_take_start_filter() != 0U) && (app_state != RLC_APP_STATE_FILTERING))
  {
    rlc_filter_start_realtime();
    if (rlc_filter_is_running() != 0U)
    {
      app_state = RLC_APP_STATE_FILTERING;
    }
    else
    {
      app_state = RLC_APP_STATE_IDLE;
    }
  }

  rlc_filter_task();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  rlc_display_uart_rx_cplt_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  rlc_display_uart_error_callback(huart);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  rlc_filter_adc_conv_cplt_callback(hadc);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  rlc_filter_adc_conv_half_cplt_callback(hadc);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  rlc_filter_adc_error_callback(hadc);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  rlc_filter_dac_conv_cplt_ch1_callback(hdac);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  rlc_filter_dac_conv_half_cplt_ch1_callback(hdac);
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  rlc_filter_dac_error_ch1_callback(hdac);
}
