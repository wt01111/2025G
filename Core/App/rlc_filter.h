#ifndef RLC_FILTER_H
#define RLC_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void rlc_filter_init(void);
void rlc_filter_start_learning(void);
void rlc_filter_start_realtime(void);
void rlc_filter_stop_realtime(void);
void rlc_filter_task(void);
uint8_t rlc_filter_is_running(void);

void rlc_filter_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc);
void rlc_filter_adc_conv_half_cplt_callback(ADC_HandleTypeDef *hadc);
void rlc_filter_adc_error_callback(ADC_HandleTypeDef *hadc);
void rlc_filter_dac_conv_cplt_ch1_callback(DAC_HandleTypeDef *hdac);
void rlc_filter_dac_conv_half_cplt_ch1_callback(DAC_HandleTypeDef *hdac);
void rlc_filter_dac_error_ch1_callback(DAC_HandleTypeDef *hdac);

#ifdef __cplusplus
}
#endif

#endif /* RLC_FILTER_H */
