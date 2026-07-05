#include "rlc_display.h"
#include "usart.h"

#include <math.h>

#define UART1_CMD_START2           "Start_2"
#define UART1_CMD_START2_LEN       (7U)
#define SCREEN_CMD_LEARN           "start_learn"
#define SCREEN_CMD_LEARN_LEN       (11U)
#define SCREEN_CMD_FILTER          "start_filter"
#define SCREEN_CMD_FILTER_LEN      (12U)
#define SCREEN_MAG_DB_MIN          (-40.0f)
#define SCREEN_MAG_DB_MAX          (0.0f)
#define SCREEN_CURVE_MAX_VALUE     (127U)
#define SCREEN_CURVE_MAX_POINTS    (320U)

static uint8_t uart1_rx_byte;
static uint8_t screen_rx_byte;
static uint8_t uart1_start2_match;
static uint8_t screen_learn_match;
static uint8_t screen_filter_match;
static volatile uint8_t start_learn_requested;
static volatile uint8_t start_filter_requested;
static volatile uint8_t debug_enabled = 1U;
static uint32_t curve_sent_count;
static uint32_t curve_total_points;

static uint32_t text_len(const char *text);
static void screen_send_raw(const char *text);
static void screen_send_end(void);
static void screen_send_uint(uint32_t value);
static void screen_send_curve(uint32_t channel, uint32_t value);
static uint32_t screen_scale_mag(float mag);
static uint32_t screen_scale_phase(float phase_deg);
static uint8_t screen_curve_point_allowed(uint32_t point_index, uint32_t total_points);
static void screen_match_byte(uint8_t value);

void rlc_display_init(void)
{
  uart1_start2_match = 0U;
  screen_learn_match = 0U;
  screen_filter_match = 0U;
  start_learn_requested = 0U;
  start_filter_requested = 0U;
  debug_enabled = 1U;

  (void)HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);

  (void)HAL_UART_AbortReceive(&huart3);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  (void)HAL_UART_Receive_IT(&huart3, &screen_rx_byte, 1U);
}

uint8_t rlc_display_take_start_learn(void)
{
  uint8_t requested;

  __disable_irq();
  requested = start_learn_requested;
  start_learn_requested = 0U;
  __enable_irq();

  return requested;
}

uint8_t rlc_display_take_start_filter(void)
{
  uint8_t requested;

  __disable_irq();
  requested = start_filter_requested;
  start_filter_requested = 0U;
  __enable_irq();

  return requested;
}

void rlc_display_set_debug_enabled(uint8_t enabled)
{
  debug_enabled = enabled;
}

void rlc_display_spectrum_begin(uint32_t total_points)
{
  curve_sent_count = 0U;
  curve_total_points = total_points;
}

void rlc_display_send_spectrum_point(uint32_t point_index, uint32_t total_points,
                                     float mag, float phase_deg)
{
  if (total_points == 0U)
  {
    total_points = curve_total_points;
  }
  if (screen_curve_point_allowed(point_index, total_points) == 0U)
  {
    return;
  }

  screen_send_curve(0U, screen_scale_mag(mag));
  screen_send_curve(1U, screen_scale_phase(phase_deg));
}

void rlc_display_send_filter_type(const char *type_text)
{
  screen_send_raw("t0.txt=\"");
  screen_send_raw(type_text);
  screen_send_raw("\"");
  screen_send_end();
}

void rlc_display_debug_puts(const char *text)
{
  if (debug_enabled == 0U)
  {
    return;
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)text_len(text), HAL_MAX_DELAY);
}

void rlc_display_debug_print_uint(uint32_t value)
{
  char text[11];
  uint32_t index = 0U;

  if (debug_enabled == 0U)
  {
    return;
  }
  if (value == 0U)
  {
    rlc_display_debug_puts("0");
    return;
  }

  while ((value > 0U) && (index < sizeof(text)))
  {
    text[index++] = (char)('0' + (value % 10U));
    value /= 10U;
  }

  while (index > 0U)
  {
    index--;
    HAL_UART_Transmit(&huart1, (uint8_t *)&text[index], 1U, HAL_MAX_DELAY);
  }
}

void rlc_display_debug_print_fixed(float value, uint32_t scale)
{
  int32_t scaled;
  uint32_t integer;
  uint32_t fraction;
  uint32_t divisor;

  if (debug_enabled == 0U)
  {
    return;
  }
  if (value < 0.0f)
  {
    rlc_display_debug_puts("-");
    value = -value;
  }

  scaled = (int32_t)((value * (float)scale) + 0.5f);
  integer = (uint32_t)scaled / scale;
  fraction = (uint32_t)scaled % scale;

  rlc_display_debug_print_uint(integer);
  if (scale > 1U)
  {
    rlc_display_debug_puts(".");
    for (divisor = scale / 10U; divisor > 0U; divisor /= 10U)
    {
      char digit = (char)('0' + ((fraction / divisor) % 10U));
      HAL_UART_Transmit(&huart1, (uint8_t *)&digit, 1U, HAL_MAX_DELAY);
    }
  }
}

void rlc_display_debug_print_sci(float value)
{
  int32_t exp = 0;

  if (debug_enabled == 0U)
  {
    return;
  }
  if (value < 0.0f)
  {
    rlc_display_debug_puts("-");
    value = -value;
  }
  if (value == 0.0f)
  {
    rlc_display_debug_puts("0.000000e+0");
    return;
  }

  while (value >= 10.0f)
  {
    value *= 0.1f;
    exp++;
  }
  while (value < 1.0f)
  {
    value *= 10.0f;
    exp--;
  }

  rlc_display_debug_print_fixed(value, 1000000U);
  if (exp >= 0)
  {
    rlc_display_debug_puts("e+");
    rlc_display_debug_print_uint((uint32_t)exp);
  }
  else
  {
    rlc_display_debug_puts("e-");
    rlc_display_debug_print_uint((uint32_t)(-exp));
  }
}

void rlc_display_uart_rx_cplt_callback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    const char *cmd = UART1_CMD_START2;

    if (uart1_rx_byte == (uint8_t)cmd[uart1_start2_match])
    {
      uart1_start2_match++;
      if (uart1_start2_match >= UART1_CMD_START2_LEN)
      {
        start_filter_requested = 1U;
        uart1_start2_match = 0U;
      }
    }
    else
    {
      uart1_start2_match = (uart1_rx_byte == (uint8_t)cmd[0]) ? 1U : 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    (void)HAL_UART_Transmit(&huart1, &screen_rx_byte, 1U, 1U);
    screen_match_byte(screen_rx_byte);
    (void)HAL_UART_Receive_IT(&huart3, &screen_rx_byte, 1U);
  }
}

void rlc_display_uart_error_callback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    (void)HAL_UART_Receive_IT(&huart3, &screen_rx_byte, 1U);
  }
}

static uint32_t text_len(const char *text)
{
  uint32_t len = 0U;

  while (text[len] != '\0')
  {
    len++;
  }

  return len;
}

static void screen_send_raw(const char *text)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)text_len(text), HAL_MAX_DELAY);
}

static void screen_send_end(void)
{
  static const uint8_t end_bytes[3] = {0xFFU, 0xFFU, 0xFFU};

  HAL_UART_Transmit(&huart3, (uint8_t *)end_bytes, sizeof(end_bytes), HAL_MAX_DELAY);
}

static void screen_send_uint(uint32_t value)
{
  char text[11];
  uint32_t index = 0U;

  if (value == 0U)
  {
    screen_send_raw("0");
    return;
  }

  while ((value > 0U) && (index < sizeof(text)))
  {
    text[index++] = (char)('0' + (value % 10U));
    value /= 10U;
  }

  while (index > 0U)
  {
    index--;
    HAL_UART_Transmit(&huart3, (uint8_t *)&text[index], 1U, HAL_MAX_DELAY);
  }
}

static void screen_send_curve(uint32_t channel, uint32_t value)
{
  if (value > SCREEN_CURVE_MAX_VALUE)
  {
    value = SCREEN_CURVE_MAX_VALUE;
  }

  screen_send_raw("add 3,");
  screen_send_uint(channel);
  screen_send_raw(",");
  screen_send_uint(value);
  screen_send_end();
}

static uint8_t screen_curve_point_allowed(uint32_t point_index, uint32_t total_points)
{
  uint32_t target_sent;

  if (curve_sent_count >= SCREEN_CURVE_MAX_POINTS)
  {
    return 0U;
  }
  if (total_points == 0U)
  {
    total_points = curve_total_points;
  }
  if (total_points == 0U)
  {
    return 0U;
  }

  target_sent = ((point_index + 1U) * SCREEN_CURVE_MAX_POINTS) / total_points;
  if (target_sent >= SCREEN_CURVE_MAX_POINTS)
  {
    target_sent = SCREEN_CURVE_MAX_POINTS - 1U;
  }
  if (curve_sent_count <= target_sent)
  {
    curve_sent_count++;
    return 1U;
  }

  return 0U;
}

static uint32_t screen_scale_mag(float mag)
{
  float db;
  float scaled;

  if (mag <= 1.0e-6f)
  {
    db = SCREEN_MAG_DB_MIN;
  }
  else
  {
    db = 20.0f * log10f(mag);
  }

  if (db < SCREEN_MAG_DB_MIN)
  {
    db = SCREEN_MAG_DB_MIN;
  }
  if (db > SCREEN_MAG_DB_MAX)
  {
    db = SCREEN_MAG_DB_MAX;
  }

  scaled = ((db - SCREEN_MAG_DB_MIN) * (float)SCREEN_CURVE_MAX_VALUE) /
           (SCREEN_MAG_DB_MAX - SCREEN_MAG_DB_MIN);
  if (scaled < 0.0f)
  {
    scaled = 0.0f;
  }
  if (scaled > (float)SCREEN_CURVE_MAX_VALUE)
  {
    scaled = (float)SCREEN_CURVE_MAX_VALUE;
  }

  return (uint32_t)(scaled + 0.5f);
}

static uint32_t screen_scale_phase(float phase_deg)
{
  float scaled;

  while (phase_deg > 180.0f)
  {
    phase_deg -= 360.0f;
  }
  while (phase_deg < -180.0f)
  {
    phase_deg += 360.0f;
  }

  scaled = ((phase_deg + 180.0f) * (float)SCREEN_CURVE_MAX_VALUE) / 360.0f;
  if (scaled < 0.0f)
  {
    scaled = 0.0f;
  }
  if (scaled > (float)SCREEN_CURVE_MAX_VALUE)
  {
    scaled = (float)SCREEN_CURVE_MAX_VALUE;
  }

  return (uint32_t)(scaled + 0.5f);
}

static void screen_match_byte(uint8_t value)
{
  const char *learn = SCREEN_CMD_LEARN;
  const char *filter = SCREEN_CMD_FILTER;

  if (value == (uint8_t)learn[screen_learn_match])
  {
    screen_learn_match++;
    if (screen_learn_match >= SCREEN_CMD_LEARN_LEN)
    {
      start_learn_requested = 1U;
      screen_learn_match = 0U;
    }
  }
  else
  {
    screen_learn_match = (value == (uint8_t)learn[0]) ? 1U : 0U;
  }

  if (value == (uint8_t)filter[screen_filter_match])
  {
    screen_filter_match++;
    if (screen_filter_match >= SCREEN_CMD_FILTER_LEN)
    {
      start_filter_requested = 1U;
      screen_filter_match = 0U;
    }
  }
  else
  {
    screen_filter_match = (value == (uint8_t)filter[0]) ? 1U : 0U;
  }
}
