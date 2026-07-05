#ifndef RLC_DISPLAY_H
#define RLC_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void rlc_display_init(void);
uint8_t rlc_display_take_start_learn(void);
uint8_t rlc_display_take_start_filter(void);
void rlc_display_set_debug_enabled(uint8_t enabled);

void rlc_display_spectrum_begin(uint32_t total_points);
void rlc_display_send_spectrum_point(uint32_t point_index, uint32_t total_points,
                                     float mag, float phase_deg);
void rlc_display_send_filter_type(const char *type_text);

void rlc_display_debug_puts(const char *text);
void rlc_display_debug_print_uint(uint32_t value);
void rlc_display_debug_print_fixed(float value, uint32_t scale);
void rlc_display_debug_print_sci(float value);

void rlc_display_uart_rx_cplt_callback(UART_HandleTypeDef *huart);
void rlc_display_uart_error_callback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* RLC_DISPLAY_H */
