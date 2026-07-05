/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "fmac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWEEP_TIM2_CLOCK_HZ        (275000000.0f)
#define SWEEP_SAMPLE_RATE_HZ       (2500000.0f)
#define SWEEP_SAMPLES              (1000U)
#define SWEEP_LOW_CYCLES           (10U)
#define SWEEP_MID_CYCLES           (20U)
#define SWEEP_HIGH_CYCLES          (50U)
#define SWEEP_LOW_BAND_HZ          (5000U)
#define SWEEP_MID_BAND_HZ          (20000U)
#define SWEEP_SETTLE_RECORDS       (2U)
#define SWEEP_ADC_TOTAL_SAMPLES    (SWEEP_SAMPLES * (SWEEP_SETTLE_RECORDS + 1U))
#define SWEEP_START_HZ             (1000U)
#define SWEEP_END_HZ               (100000U)
#define SWEEP_STEP_HZ              (200U)
#define SWEEP_MAX_POINTS           (((SWEEP_END_HZ - SWEEP_START_HZ) / SWEEP_STEP_HZ) + 1U)
#define SWEEP_DAC_MID              (2048.0f)
#define SWEEP_DAC_AMP              (1240.0f)
#define SWEEP_ADC_FULL_SCALE       (65535.0f)
#define SWEEP_DAC_FULL_SCALE       (4095.0f)
#define SWEEP_ADC_TIMEOUT_MARGIN_MS (100U)
#define FIT_MIN_POINTS             (12U)
#define FIT_VALID_MAG_RATIO        (0.025f)
#define FIT_MODEL_LOWPASS          (0U)
#define FIT_MODEL_BANDPASS         (1U)
#define FIT_MODEL_HIGHPASS         (2U)
#define UART1_CMD_START2           "Start_2"
#define UART1_CMD_START2_LEN       (7U)
#define IIR_SAMPLE_RATE_HZ         (2500000U)
#define IIR_BLOCK_SAMPLES          (512U)
#define IIR_DMA_SAMPLES            (IIR_BLOCK_SAMPLES * 2U)
#define IIR_USE_FMAC               (1U)
#define IIR_DEBUG_ADC_TO_DAC       (0U)
#define IIR_DEBUG_REPORT_BLOCKS    (1000U)
#define IIR_UART_PRINT_ENABLE      (1U)
#define IIR_FMAC_COEFF_B_SIZE      (3U)
#define IIR_FMAC_COEFF_A_SIZE      (2U)
#define PI_F                       (3.14159265358979323846f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_DMA_ATTR __attribute__((aligned(32)))
#define DAC_DMA_ATTR __attribute__((aligned(32)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t adc_buf[SWEEP_ADC_TOTAL_SAMPLES] ADC_DMA_ATTR;
static uint16_t dac_wave[SWEEP_SAMPLES] DAC_DMA_ATTR;
static volatile uint8_t adc_done;
static volatile uint8_t adc_error;
static volatile uint32_t dac_rounds;
static float sweep_cycles_per_record = 0.0f;
static float fit_freq_hz[SWEEP_MAX_POINTS];
static float fit_mag[SWEEP_MAX_POINTS];
static float fit_phase_deg[SWEEP_MAX_POINTS];
static uint32_t fit_count;
static uint8_t uart1_rx_byte;
static volatile uint8_t uart1_start2_requested;
static uint8_t uart1_start2_match;
static volatile uint8_t uart_print_enabled = 1U;
static uint16_t iir_adc_buf[IIR_DMA_SAMPLES] ADC_DMA_ATTR;
static uint16_t iir_dac_buf[IIR_DMA_SAMPLES] DAC_DMA_ATTR;
static volatile uint8_t iir_running;
static volatile uint8_t iir_process_flags;
static volatile uint8_t iir_overrun;
static volatile uint32_t iir_half_irq_count;
static volatile uint32_t iir_full_irq_count;
static volatile uint32_t iir_dac_half_irq_count;
static volatile uint32_t iir_dac_full_irq_count;
static volatile uint32_t iir_dac_error_count;
static volatile uint32_t iir_fmac_error_count;
static uint32_t iir_processed_count;
static uint32_t iir_next_debug_count;
static uint16_t iir_adc_min;
static uint16_t iir_adc_max;
static uint16_t iir_dac_min;
static uint16_t iir_dac_max;
static float fit_n2;
static float fit_n1;
static float fit_n0;
static float fit_a;
static float fit_b;
static uint8_t fit_system_valid;
static float iir_b0;
static float iir_b1;
static float iir_b2;
static float iir_a1;
static float iir_a2;
static uint8_t iir_coeff_valid;
static float32_t iir_coeffs[5];
static float32_t iir_state[4];
static float32_t iir_float_buf[IIR_BLOCK_SAMPLES];
static arm_biquad_casd_df1_inst_f32 iir_biquad;
static int16_t iir_fmac_input[IIR_BLOCK_SAMPLES];
static int16_t iir_fmac_output[IIR_BLOCK_SAMPLES];
static int16_t iir_fmac_coeff_b[IIR_FMAC_COEFF_B_SIZE];
static int16_t iir_fmac_coeff_a[IIR_FMAC_COEFF_A_SIZE];
static int16_t iir_fmac_zero[3];
static uint8_t iir_fmac_ready;
static uint8_t iir_fmac_coeff_clipped;

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_dac1_ch1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void sweep_run_once(void);
static uint8_t sweep_prepare_point(void);
static uint8_t sweep_force_adc_dma_oneshot(void);
static uint32_t sweep_select_cycles(uint32_t target_freq_hz);
static float sweep_config_timer_for_freq(uint32_t target_freq_hz);
static void sweep_make_dac_wave(void);
static float sweep_reference_amp_adc_counts(void);
static void sweep_analyze(float *mag, float *phase_deg);
static void sweep_fit_store(float mag, float phase_deg, uint32_t freq_hz);
static void sweep_fit_rlc_and_print(void);
static float sweep_rlc_mag_base(uint32_t model, float freq_hz, float f0_hz, float q);
static float sweep_rlc_phase_deg(uint32_t model, float freq_hz, float f0_hz, float q);
static void sweep_uart_print_model(uint32_t model);
static float sweep_unwrap_phase(float phase_deg, float last_phase_deg);
static void uart1_cmd_rx_start(void);
static void iir_start_from_fit(void);
static void iir_load_fixed_coefficients(void);
static void iir_config_timer(void);
static uint8_t iir_make_coefficients(void);
static void iir_process_pending(void);
static void iir_process_block(uint32_t offset);
static void iir_debug_report_once(void);
static uint16_t iir_float_to_dac(float value);
static int16_t iir_float_to_fmac_q15(float value);
static uint8_t iir_fmac_configure_from_coefficients(void);
static uint8_t iir_fmac_process_block(uint32_t offset);
static void sweep_uart_puts(const char *text);
static void sweep_uart_print_uint(uint32_t value);
static void sweep_uart_print_fixed(float value, uint32_t scale);
static void sweep_uart_print_sci(float value);
static void sweep_uart_print_line(float mag, float phase_deg, uint32_t freq_hz);
static float sweep_sinf(float x);
static float sweep_cosf(float x);
static float sweep_sqrtf(float x);
static float sweep_atanf(float x);
static float sweep_atan2f(float y, float x);
static uint32_t sweep_strlen(const char *text);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  uart_print_enabled = 1U;
  sweep_run_once();
  uart1_cmd_rx_start();
  sweep_uart_puts("CMD_READY,Start_2\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((uart1_start2_requested != 0U) && (iir_running == 0U))
    {
      uart1_start2_requested = 0U;
      uart_print_enabled = 0U;
      sweep_uart_puts("CMD_START_2\r\n");
      iir_start_from_fit();
    }

    iir_process_pending();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void sweep_uart_puts(const char *text)
{
#if (IIR_UART_PRINT_ENABLE == 0U)
  (void)text;
  return;
#else
  if (uart_print_enabled == 0U)
  {
    return;
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)sweep_strlen(text), HAL_MAX_DELAY);
#endif
}

static uint32_t sweep_strlen(const char *text)
{
  uint32_t len = 0U;

  while (text[len] != '\0')
  {
    len++;
  }

  return len;
}

static void sweep_uart_print_uint(uint32_t value)
{
#if (IIR_UART_PRINT_ENABLE == 0U)
  (void)value;
  return;
#else
  if (uart_print_enabled == 0U)
  {
    return;
  }
  char text[11];
  uint32_t index = 0U;

  if (value == 0U)
  {
    sweep_uart_puts("0");
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
#endif
}

static void sweep_uart_print_fixed(float value, uint32_t scale)
{
#if (IIR_UART_PRINT_ENABLE == 0U)
  (void)value;
  (void)scale;
  return;
#else
  if (uart_print_enabled == 0U)
  {
    return;
  }
  int32_t scaled;
  uint32_t integer;
  uint32_t fraction;
  uint32_t divisor;

  if (value < 0.0f)
  {
    sweep_uart_puts("-");
    value = -value;
  }

  scaled = (int32_t)((value * (float)scale) + 0.5f);
  integer = (uint32_t)scaled / scale;
  fraction = (uint32_t)scaled % scale;

  sweep_uart_print_uint(integer);
  if (scale > 1U)
  {
    sweep_uart_puts(".");
    for (divisor = scale / 10U; divisor > 0U; divisor /= 10U)
    {
      char digit = (char)('0' + ((fraction / divisor) % 10U));
      HAL_UART_Transmit(&huart1, (uint8_t *)&digit, 1U, HAL_MAX_DELAY);
    }
  }
#endif
}

static void sweep_uart_print_sci(float value)
{
#if (IIR_UART_PRINT_ENABLE == 0U)
  (void)value;
  return;
#else
  if (uart_print_enabled == 0U)
  {
    return;
  }
  int32_t exp = 0;

  if (value < 0.0f)
  {
    sweep_uart_puts("-");
    value = -value;
  }

  if (value == 0.0f)
  {
    sweep_uart_puts("0.000000e+0");
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

  sweep_uart_print_fixed(value, 1000000U);
  if (exp >= 0)
  {
    sweep_uart_puts("e+");
    sweep_uart_print_uint((uint32_t)exp);
  }
  else
  {
    sweep_uart_puts("e-");
    sweep_uart_print_uint((uint32_t)(-exp));
  }
#endif
}

static void sweep_uart_print_line(float mag, float phase_deg, uint32_t freq_hz)
{
#if (IIR_UART_PRINT_ENABLE == 0U)
  (void)mag;
  (void)phase_deg;
  (void)freq_hz;
  return;
#else
  if (uart_print_enabled == 0U)
  {
    return;
  }
  sweep_uart_print_fixed(mag, 1000000U);
  sweep_uart_puts(",");
  sweep_uart_print_fixed(phase_deg, 100U);
  sweep_uart_puts(",");
  sweep_uart_print_uint(freq_hz);
  sweep_uart_puts("\r\n");
#endif
}

static float sweep_config_timer_for_freq(uint32_t target_freq_hz)
{
  uint32_t cycles = sweep_select_cycles(target_freq_hz);
  float target_sample_rate = ((float)target_freq_hz * (float)SWEEP_SAMPLES) / (float)cycles;
  uint32_t arr = (uint32_t)((SWEEP_TIM2_CLOCK_HZ / target_sample_rate) + 0.5f);
  float actual_sample_rate;

  if (arr > 0U)
  {
    arr--;
  }
  if (arr < 1U)
  {
    arr = 1U;
  }

  HAL_TIM_Base_Stop(&htim2);
  __HAL_TIM_SET_PRESCALER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  htim2.Instance->EGR = TIM_EGR_UG;

  actual_sample_rate = SWEEP_TIM2_CLOCK_HZ / ((float)arr + 1.0f);
  return actual_sample_rate;
}

static uint32_t sweep_select_cycles(uint32_t target_freq_hz)
{
  if (target_freq_hz <= SWEEP_LOW_BAND_HZ)
  {
    return SWEEP_LOW_CYCLES;
  }
  if (target_freq_hz <= SWEEP_MID_BAND_HZ)
  {
    return SWEEP_MID_CYCLES;
  }
  return SWEEP_HIGH_CYCLES;
}

static void sweep_make_dac_wave(void)
{
  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    float angle = 2.0f * PI_F * (float)sweep_cycles_per_record * (float)n / (float)SWEEP_SAMPLES;
    float sample = SWEEP_DAC_MID + (SWEEP_DAC_AMP * sweep_cosf(angle));

    if (sample < 0.0f)
    {
      sample = 0.0f;
    }
    else if (sample > 4095.0f)
    {
      sample = 4095.0f;
    }

    dac_wave[n] = (uint16_t)(sample + 0.5f);
  }
}

static uint8_t sweep_force_adc_dma_oneshot(void)
{
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_ONESHOT);
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD, ADC_OVR_DATA_OVERWRITTEN);

  if (READ_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMNGT) != ADC_CONVERSIONDATA_DMA_ONESHOT)
  {
    return 0U;
  }
  if (READ_BIT(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD) != ADC_OVR_DATA_OVERWRITTEN)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t sweep_prepare_point(void)
{
  HAL_TIM_Base_Stop(&htim2);
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  iir_running = 0U;
  iir_process_flags = 0U;

  (void)HAL_DMA_DeInit(&hdma_adc1);
  hdma_adc1.Init.Mode = DMA_NORMAL;
  hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    return 0U;
  }
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  (void)HAL_DMA_DeInit(&hdma_dac1_ch1);
  hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    return 0U;
  }
  __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);

  return sweep_force_adc_dma_oneshot();
}

static float sweep_reference_amp_adc_counts(void)
{
  float ref_re = 0.0f;
  float ref_im = 0.0f;

  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    float x = (float)dac_wave[n] - SWEEP_DAC_MID;
    float angle = 2.0f * PI_F * (float)sweep_cycles_per_record * (float)n / (float)SWEEP_SAMPLES;
    ref_re += x * sweep_cosf(angle);
    ref_im -= x * sweep_sinf(angle);
  }

  return ((2.0f * sweep_sqrtf((ref_re * ref_re) + (ref_im * ref_im))) /
          (float)SWEEP_SAMPLES) * SWEEP_ADC_FULL_SCALE / SWEEP_DAC_FULL_SCALE;
}

static void sweep_analyze(float *mag, float *phase_deg)
{
  uint32_t sum = 0U;
  float mean;
  float re = 0.0f;
  float im = 0.0f;
  float ref_amp_adc_counts = sweep_reference_amp_adc_counts();
  uint32_t offset = SWEEP_SAMPLES * SWEEP_SETTLE_RECORDS;

  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    sum += (uint32_t)adc_buf[offset + n];
  }
  mean = (float)sum / (float)SWEEP_SAMPLES;

  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    float y = (float)adc_buf[offset + n] - mean;
    float angle = 2.0f * PI_F * (float)sweep_cycles_per_record * (float)n / (float)SWEEP_SAMPLES;
    re += y * sweep_cosf(angle);
    im -= y * sweep_sinf(angle);
  }

  if (ref_amp_adc_counts <= 1.0e-6f)
  {
    *mag = 0.0f;
  }
  else
  {
    *mag = (2.0f * sweep_sqrtf((re * re) + (im * im))) / ((float)SWEEP_SAMPLES * ref_amp_adc_counts);
  }
  *phase_deg = sweep_atan2f(im, re) * 180.0f / PI_F;
}

static void sweep_run_once(void)
{
  uint32_t last_freq_hz = 0U;

  fit_count = 0U;
  sweep_uart_puts("mag,phase_deg,freq_hz\r\n");

  for (uint32_t target_freq_hz = SWEEP_START_HZ; target_freq_hz <= SWEEP_END_HZ; target_freq_hz += SWEEP_STEP_HZ)
  {
    float mag;
    float phase_deg;
    uint32_t cycles = sweep_select_cycles(target_freq_hz);
    float actual_sample_rate = sweep_config_timer_for_freq(target_freq_hz);
    uint32_t freq_hz = (uint32_t)(((actual_sample_rate * (float)cycles) / (float)SWEEP_SAMPLES) + 0.5f);
    uint32_t adc_timeout_ms = SWEEP_ADC_TIMEOUT_MARGIN_MS +
      (uint32_t)(((float)SWEEP_ADC_TOTAL_SAMPLES * 1000.0f) / actual_sample_rate);
    sweep_cycles_per_record = (float)cycles;

    if (freq_hz == last_freq_hz)
    {
      continue;
    }
    last_freq_hz = freq_hz;

    if (sweep_prepare_point() == 0U)
    {
      sweep_uart_puts("SWEEP_DMA_CONFIG_ERROR,");
      sweep_uart_print_uint(freq_hz);
      sweep_uart_puts("\r\n");
      continue;
    }

    adc_done = 0U;
    adc_error = 0U;
    dac_rounds = 0U;
    sweep_make_dac_wave();
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
      SCB_CleanDCache_by_Addr((uint32_t *)dac_wave, (int32_t)sizeof(dac_wave));
      SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_buf, (int32_t)sizeof(adc_buf));
    }

    if (sweep_force_adc_dma_oneshot() == 0U)
    {
      sweep_uart_puts("ADC_CFG_ERROR,");
      sweep_uart_print_uint(freq_hz);
      sweep_uart_puts("\r\n");
      continue;
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, SWEEP_ADC_TOTAL_SAMPLES) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_wave, SWEEP_SAMPLES, DAC_ALIGN_12B_R) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
      Error_Handler();
    }

    uint32_t start = HAL_GetTick();
    while (adc_done == 0U)
    {
      if (adc_error != 0U)
      {
        sweep_uart_puts("ADC_ERROR,");
        sweep_uart_print_uint(HAL_ADC_GetError(&hadc1));
        sweep_uart_puts(",");
        sweep_uart_print_uint(freq_hz);
        sweep_uart_puts("\r\n");
        break;
      }
      if ((HAL_GetTick() - start) > adc_timeout_ms)
      {
        sweep_uart_puts("ADC_TIMEOUT,");
        sweep_uart_print_uint(HAL_ADC_GetError(&hadc1));
        sweep_uart_puts(",");
        sweep_uart_print_uint(freq_hz);
        sweep_uart_puts("\r\n");
        break;
      }
    }

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim2);

    if (adc_done != 0U)
    {
      if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
      {
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc_buf, (int32_t)sizeof(adc_buf));
      }
      sweep_analyze(&mag, &phase_deg);
      sweep_uart_print_line(mag, phase_deg, freq_hz);
      sweep_fit_store(mag, phase_deg, freq_hz);
    }

    HAL_Delay(2U);
  }

  sweep_uart_puts("SWEEP_DONE\r\n");
  sweep_fit_rlc_and_print();
}

static void sweep_fit_store(float mag, float phase_deg, uint32_t freq_hz)
{
  if (fit_count >= SWEEP_MAX_POINTS)
  {
    return;
  }

  fit_freq_hz[fit_count] = (float)freq_hz;
  fit_mag[fit_count] = mag;
  fit_phase_deg[fit_count] = phase_deg;
  fit_count++;
}

static float sweep_rlc_mag_base(uint32_t model, float freq_hz, float f0_hz, float q)
{
  float w = 2.0f * PI_F * freq_hz;
  float w0 = 2.0f * PI_F * f0_hz;
  float a = w0 / q;
  float b = w0 * w0;
  float real = (w0 * w0) - (w * w);
  float imag = a * w;
  float den = sweep_sqrtf((real * real) + (imag * imag));

  if (den <= 0.0f)
  {
    return 0.0f;
  }

  if (model == FIT_MODEL_LOWPASS)
  {
    return b / den;
  }
  if (model == FIT_MODEL_HIGHPASS)
  {
    return (w * w) / den;
  }
  return imag / den;
}

static float sweep_rlc_phase_deg(uint32_t model, float freq_hz, float f0_hz, float q)
{
  float w = 2.0f * PI_F * freq_hz;
  float w0 = 2.0f * PI_F * f0_hz;
  float a = w0 / q;
  float real = (w0 * w0) - (w * w);
  float imag = a * w;
  float num_phase_deg = 90.0f;

  if (model == FIT_MODEL_LOWPASS)
  {
    num_phase_deg = 0.0f;
  }
  else if (model == FIT_MODEL_HIGHPASS)
  {
    num_phase_deg = 180.0f;
  }

  return num_phase_deg - (sweep_atan2f(imag, real) * 180.0f / PI_F);
}

static void sweep_uart_print_model(uint32_t model)
{
  if (model == FIT_MODEL_LOWPASS)
  {
    sweep_uart_puts("LOWPASS");
  }
  else if (model == FIT_MODEL_HIGHPASS)
  {
    sweep_uart_puts("HIGHPASS");
  }
  else
  {
    sweep_uart_puts("BANDPASS");
  }
}

static float sweep_unwrap_phase(float phase_deg, float last_phase_deg)
{
  while ((phase_deg - last_phase_deg) > 180.0f)
  {
    phase_deg -= 360.0f;
  }
  while ((phase_deg - last_phase_deg) < -180.0f)
  {
    phase_deg += 360.0f;
  }

  return phase_deg;
}

static void sweep_fit_rlc_and_print(void)
{
  float peak_mag = 0.0f;
  float best_f0 = 0.0f;
  float best_q = 1.0f;
  float best_k = 1.0f;
  float best_err = 3.4e38f;
  uint32_t best_model = FIT_MODEL_BANDPASS;
  float valid_min_mag;

  if (fit_count < FIT_MIN_POINTS)
  {
    sweep_uart_puts("FIT_ERROR,not_enough_points\r\n");
    return;
  }

  for (uint32_t i = 0; i < fit_count; i++)
  {
    if (fit_mag[i] > peak_mag)
    {
      peak_mag = fit_mag[i];
    }
  }

  if (peak_mag <= 0.0f)
  {
    sweep_uart_puts("FIT_ERROR,bad_data\r\n");
    return;
  }

  best_f0 = fit_freq_hz[fit_count / 2U];
  valid_min_mag = peak_mag * FIT_VALID_MAG_RATIO;

  for (uint32_t model = FIT_MODEL_LOWPASS; model <= FIT_MODEL_HIGHPASS; model++)
  {
    float model_best_f0 = best_f0;
    float model_best_q = 1.0f;
    float model_best_k = peak_mag;
    float model_best_k_at_err = peak_mag;
    float model_best_err = 3.4e38f;

    for (uint32_t pass = 0; pass < 3U; pass++)
    {
      float f_min;
      float f_max;
      float q_min;
      float q_max;

      if (pass == 0U)
      {
        f_min = (float)SWEEP_START_HZ;
        f_max = (float)SWEEP_END_HZ;
        q_min = 0.10f;
        q_max = 20.0f;
      }
      else
      {
        float f_span = model_best_f0 * ((pass == 1U) ? 0.35f : 0.12f);
        float q_span = model_best_q * ((pass == 1U) ? 0.55f : 0.20f);

        f_min = model_best_f0 - f_span;
        f_max = model_best_f0 + f_span;
        q_min = model_best_q - q_span;
        q_max = model_best_q + q_span;
        if (f_min < (float)SWEEP_START_HZ)
        {
          f_min = (float)SWEEP_START_HZ;
        }
        if (f_max > (float)SWEEP_END_HZ)
        {
          f_max = (float)SWEEP_END_HZ;
        }
        if (q_min < 0.08f)
        {
          q_min = 0.08f;
        }
      }

      for (uint32_t fi = 0; fi <= 24U; fi++)
      {
        float f0 = f_min + (((f_max - f_min) * (float)fi) / 24.0f);

        for (uint32_t qi = 0; qi <= 24U; qi++)
        {
          float q = q_min + (((q_max - q_min) * (float)qi) / 24.0f);
          float k_num = 0.0f;
          float k_den = 0.0f;
          float err = 0.0f;
          uint32_t used = 0U;

          for (uint32_t i = 0; i < fit_count; i++)
          {
            float base;

            if (fit_mag[i] < valid_min_mag)
            {
              continue;
            }

            base = sweep_rlc_mag_base(model, fit_freq_hz[i], f0, q);
            if (base > 1.0e-9f)
            {
              k_num += fit_mag[i] * base;
              k_den += base * base;
              used++;
            }
          }

          if ((used < FIT_MIN_POINTS) || (k_den <= 0.0f))
          {
            continue;
          }

          model_best_k = k_num / k_den;

          for (uint32_t i = 0; i < fit_count; i++)
          {
            float base;
            float pred;
            float e;

            if (fit_mag[i] < valid_min_mag)
            {
              continue;
            }

            base = sweep_rlc_mag_base(model, fit_freq_hz[i], f0, q);
            pred = model_best_k * base;
            e = (pred - fit_mag[i]) / fit_mag[i];
            err += e * e;
          }

          err /= (float)used;
          if (err < model_best_err)
          {
            model_best_err = err;
            model_best_f0 = f0;
            model_best_q = q;
            model_best_k_at_err = model_best_k;
          }
        }
      }
    }

    if (model_best_err < best_err)
    {
      best_err = model_best_err;
      best_model = model;
      best_f0 = model_best_f0;
      best_q = model_best_q;
      best_k = model_best_k_at_err;
    }
  }

  {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float last_phase = fit_phase_deg[0];
    float phase_offset_deg = 0.0f;
    float phase_slope_deg_per_hz = 0.0f;
    float w0 = 2.0f * PI_F * best_f0;
    float a = w0 / best_q;
    float b = w0 * w0;
    uint32_t used = 0U;

    for (uint32_t i = 0; i < fit_count; i++)
    {
      float unwrapped = sweep_unwrap_phase(fit_phase_deg[i], last_phase);
      float residual;

      last_phase = unwrapped;
      if (fit_mag[i] < valid_min_mag)
      {
        continue;
      }

      residual = unwrapped - sweep_rlc_phase_deg(best_model, fit_freq_hz[i], best_f0, best_q);
      sum_x += fit_freq_hz[i];
      sum_y += residual;
      sum_xx += fit_freq_hz[i] * fit_freq_hz[i];
      sum_xy += fit_freq_hz[i] * residual;
      used++;
    }

    if (used > 1U)
    {
      float den = ((float)used * sum_xx) - (sum_x * sum_x);
      if ((den > 0.0f) || (den < 0.0f))
      {
        phase_slope_deg_per_hz = (((float)used * sum_xy) - (sum_x * sum_y)) / den;
        phase_offset_deg = (sum_y - (phase_slope_deg_per_hz * sum_x)) / (float)used;
      }
    }

    sweep_uart_puts("FIT_SYSTEM,type,");
    sweep_uart_print_model(best_model);
    sweep_uart_puts(",K,");
    sweep_uart_print_fixed(best_k, 1000000U);
    sweep_uart_puts(",f0_Hz,");
    sweep_uart_print_fixed(best_f0, 100U);
    sweep_uart_puts(",Q,");
    sweep_uart_print_fixed(best_q, 10000U);
    sweep_uart_puts(",delay_us,");
    sweep_uart_print_fixed((-phase_slope_deg_per_hz / 360.0f) * 1000000.0f, 1000U);
    sweep_uart_puts(",phase0_deg,");
    sweep_uart_print_fixed(phase_offset_deg, 100U);
    sweep_uart_puts(",err,");
    sweep_uart_print_sci(best_err);
    sweep_uart_puts("\r\n");

    sweep_uart_puts("FIT_COEFF,Hs=(n2*s^2+n1*s+n0)/(s^2+a*s+b),n2,");
    if (best_model == FIT_MODEL_HIGHPASS)
    {
      fit_n2 = best_k;
      sweep_uart_print_sci(best_k);
    }
    else
    {
      fit_n2 = 0.0f;
      sweep_uart_print_sci(0.0f);
    }
    sweep_uart_puts(",n1,");
    if (best_model == FIT_MODEL_BANDPASS)
    {
      fit_n1 = best_k * a;
      sweep_uart_print_sci(best_k * a);
    }
    else
    {
      fit_n1 = 0.0f;
      sweep_uart_print_sci(0.0f);
    }
    sweep_uart_puts(",n0,");
    if (best_model == FIT_MODEL_LOWPASS)
    {
      fit_n0 = best_k * b;
      sweep_uart_print_sci(best_k * b);
    }
    else
    {
      fit_n0 = 0.0f;
      sweep_uart_print_sci(0.0f);
    }
    fit_a = a;
    fit_b = b;
    fit_system_valid = 1U;
    iir_coeff_valid = iir_make_coefficients();
    sweep_uart_puts(",a,");
    sweep_uart_print_sci(a);
    sweep_uart_puts(",b,");
    sweep_uart_print_sci(b);
    sweep_uart_puts("\r\n");

    if (iir_coeff_valid != 0U)
    {
      sweep_uart_puts("IIR_COEFF,b0,");
      sweep_uart_print_sci(iir_b0);
      sweep_uart_puts(",b1,");
      sweep_uart_print_sci(iir_b1);
      sweep_uart_puts(",b2,");
      sweep_uart_print_sci(iir_b2);
      sweep_uart_puts(",a1,");
      sweep_uart_print_sci(iir_a1);
      sweep_uart_puts(",a2,");
      sweep_uart_print_sci(iir_a2);
      sweep_uart_puts("\r\n");
    }
    else
    {
      sweep_uart_puts("IIR_COEFF_ERROR\r\n");
    }
  }
}

static void uart1_cmd_rx_start(void)
{
  uart1_start2_match = 0U;
  uart1_start2_requested = 0U;
  (void)HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
}

static void iir_config_timer(void)
{
  uint32_t arr = (uint32_t)((SWEEP_TIM2_CLOCK_HZ / (float)IIR_SAMPLE_RATE_HZ) + 0.5f);

  if (arr > 0U)
  {
    arr--;
  }
  if (arr < 1U)
  {
    arr = 1U;
  }

  HAL_TIM_Base_Stop(&htim2);
  __HAL_TIM_SET_PRESCALER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  htim2.Instance->EGR = TIM_EGR_UG;
}

static uint8_t iir_make_coefficients(void)
{
  float c = 2.0f * (float)IIR_SAMPLE_RATE_HZ;
  float c2 = c * c;
  float n2 = fit_n2;
  float n1 = fit_n1;
  float n0 = fit_n0;
  float a = fit_a;
  float b = fit_b;
  float d0;
  float d1;
  float d2;
  float m0;
  float m1;
  float m2;

  iir_coeff_valid = 0U;
  if (fit_system_valid == 0U)
  {
    return 0U;
  }

  d0 = c2 + (a * c) + b;
  d1 = (-2.0f * c2) + (2.0f * b);
  d2 = c2 - (a * c) + b;
  m0 = (n2 * c2) + (n1 * c) + n0;
  m1 = (-2.0f * n2 * c2) + (2.0f * n0);
  m2 = (n2 * c2) - (n1 * c) + n0;

  if ((d0 > -1.0e-9f) && (d0 < 1.0e-9f))
  {
    return 0U;
  }

  iir_b0 = m0 / d0;
  iir_b1 = m1 / d0;
  iir_b2 = m2 / d0;
  iir_a1 = d1 / d0;
  iir_a2 = d2 / d0;
  iir_coeffs[0] = iir_b0;
  iir_coeffs[1] = iir_b1;
  iir_coeffs[2] = iir_b2;
  iir_coeffs[3] = -iir_a1;
  iir_coeffs[4] = -iir_a2;
  arm_biquad_cascade_df1_init_f32(&iir_biquad, 1U, iir_coeffs, iir_state);
  iir_coeff_valid = 1U;

  return 1U;
}

static void iir_load_fixed_coefficients(void)
{
  iir_b0 = 1.109647e-1f;
  iir_b1 = 0.0f;
  iir_b2 = -1.109647e-1f;
  iir_a1 = -1.724141f;
  iir_a2 = 7.585034e-1f;
  iir_coeffs[0] = iir_b0;
  iir_coeffs[1] = iir_b1;
  iir_coeffs[2] = iir_b2;
  iir_coeffs[3] = -iir_a1;
  iir_coeffs[4] = -iir_a2;
  arm_biquad_cascade_df1_init_f32(&iir_biquad, 1U, iir_coeffs, iir_state);
  iir_coeff_valid = 1U;

  sweep_uart_puts("IIR_FIXED_COEFF,b0,");
  sweep_uart_print_sci(iir_b0);
  sweep_uart_puts(",b1,");
  sweep_uart_print_sci(iir_b1);
  sweep_uart_puts(",b2,");
  sweep_uart_print_sci(iir_b2);
  sweep_uart_puts(",a1,");
  sweep_uart_print_sci(iir_a1);
  sweep_uart_puts(",a2,");
  sweep_uart_print_sci(iir_a2);
  sweep_uart_puts("\r\n");
}

__WEAK void arm_biquad_cascade_df1_init_f32(arm_biquad_casd_df1_inst_f32 *S,
                                            uint8_t numStages,
                                            const float32_t *pCoeffs,
                                            float32_t *pState)
{
  S->numStages = numStages;
  S->pCoeffs = pCoeffs;
  S->pState = pState;

  for (uint32_t i = 0; i < (4U * (uint32_t)numStages); i++)
  {
    pState[i] = 0.0f;
  }
}

__WEAK void arm_biquad_cascade_df1_f32(const arm_biquad_casd_df1_inst_f32 *S,
                                       const float32_t *pSrc,
                                       float32_t *pDst,
                                       uint32_t blockSize)
{
  const float32_t *pCoeffs = S->pCoeffs;
  float32_t *pState = S->pState;

  for (uint32_t stage = 0; stage < (uint32_t)S->numStages; stage++)
  {
    float32_t b0 = pCoeffs[(stage * 5U) + 0U];
    float32_t b1 = pCoeffs[(stage * 5U) + 1U];
    float32_t b2 = pCoeffs[(stage * 5U) + 2U];
    float32_t a1 = pCoeffs[(stage * 5U) + 3U];
    float32_t a2 = pCoeffs[(stage * 5U) + 4U];
    float32_t x1 = pState[(stage * 4U) + 0U];
    float32_t x2 = pState[(stage * 4U) + 1U];
    float32_t y1 = pState[(stage * 4U) + 2U];
    float32_t y2 = pState[(stage * 4U) + 3U];

    for (uint32_t n = 0; n < blockSize; n++)
    {
      float32_t x = (stage == 0U) ? pSrc[n] : pDst[n];
      float32_t y = (b0 * x) + (b1 * x1) + (b2 * x2) + (a1 * y1) + (a2 * y2);

      x2 = x1;
      x1 = x;
      y2 = y1;
      y1 = y;
      pDst[n] = y;
    }

    pState[(stage * 4U) + 0U] = x1;
    pState[(stage * 4U) + 1U] = x2;
    pState[(stage * 4U) + 2U] = y1;
    pState[(stage * 4U) + 3U] = y2;
  }
}

static void iir_start_from_fit(void)
{
  sweep_uart_puts("IIR_ENTER\r\n");
  if (iir_coeff_valid == 0U)
  {
    sweep_uart_puts("IIR_NO_COEFF\r\n");
    return;
  }

  if (iir_running != 0U)
  {
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  }

  for (uint32_t i = 0; i < 4U; i++)
  {
    iir_state[i] = 0.0f;
  }
  arm_biquad_cascade_df1_init_f32(&iir_biquad, 1U, iir_coeffs, iir_state);
  iir_fmac_ready = 0U;
  iir_process_flags = 0U;
  iir_overrun = 0U;
  iir_half_irq_count = 0U;
  iir_full_irq_count = 0U;
  iir_dac_half_irq_count = 0U;
  iir_dac_full_irq_count = 0U;
  iir_dac_error_count = 0U;
  iir_fmac_error_count = 0U;
  iir_processed_count = 0U;
  iir_next_debug_count = 4U;
  iir_adc_min = 65535U;
  iir_adc_max = 0U;
  iir_dac_min = 4095U;
  iir_dac_max = 0U;

  for (uint32_t i = 0; i < IIR_DMA_SAMPLES; i++)
  {
    iir_adc_buf[i] = 32768U;
    iir_dac_buf[i] = 2048U;
  }
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_CleanDCache_by_Addr((uint32_t *)iir_dac_buf, (int32_t)sizeof(iir_dac_buf));
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)iir_adc_buf, (int32_t)sizeof(iir_adc_buf));
  }

#if (IIR_USE_FMAC != 0U)
  if (iir_fmac_configure_from_coefficients() == 0U)
  {
    sweep_uart_puts("IIR_FAIL,FMAC_CONFIG\r\n");
    return;
  }
#endif

  sweep_uart_puts("IIR_STAGE,BUF_OK\r\n");
  if (hdma_adc1.Init.Mode != DMA_CIRCULAR)
  {
    (void)HAL_DMA_DeInit(&hdma_adc1);
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      sweep_uart_puts("IIR_FAIL,DMA_INIT\r\n");
      return;
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
  }

  (void)HAL_DMA_DeInit(&hdma_dac1_ch1);
  hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    sweep_uart_puts("IIR_FAIL,DAC_DMA_INIT\r\n");
    return;
  }
  __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);

  iir_config_timer();
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_CIRCULAR);
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD, ADC_OVR_DATA_OVERWRITTEN);
  iir_running = 1U;

  sweep_uart_puts("IIR_STAGE,ADC_START\r\n");
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)iir_adc_buf, IIR_DMA_SAMPLES) != HAL_OK)
  {
    iir_running = 0U;
    sweep_uart_puts("IIR_FAIL,ADC_START,err,");
    sweep_uart_print_uint(HAL_ADC_GetError(&hadc1));
    sweep_uart_puts("\r\n");
    return;
  }
  sweep_uart_puts("IIR_STAGE,DAC_START\r\n");
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)iir_dac_buf, IIR_DMA_SAMPLES, DAC_ALIGN_12B_R) != HAL_OK)
  {
    iir_running = 0U;
    sweep_uart_puts("IIR_FAIL,DAC_START\r\n");
    return;
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  sweep_uart_puts("IIR_STAGE,TIM_START\r\n");
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    iir_running = 0U;
    sweep_uart_puts("IIR_FAIL,TIM_START\r\n");
    return;
  }

  sweep_uart_puts("IIR_START,Fs,");
  sweep_uart_print_uint(IIR_SAMPLE_RATE_HZ);
  sweep_uart_puts(",block,");
  sweep_uart_print_uint(IIR_BLOCK_SAMPLES);
  sweep_uart_puts("\r\n");
#if (IIR_DEBUG_ADC_TO_DAC != 0U)
  sweep_uart_puts("IIR_MODE,ADC_TO_DAC_DEBUG\r\n");
#else
  sweep_uart_puts("IIR_MODE,SOFTWARE_IIR\r\n");
#endif

  sweep_uart_puts("IIR_DAC_STATE,cr,");
  sweep_uart_print_uint((uint32_t)DAC1->CR);
  sweep_uart_puts(",sr,");
  sweep_uart_print_uint((uint32_t)DAC1->SR);
  sweep_uart_puts(",dhr,");
  sweep_uart_print_uint((uint32_t)(DAC1->DHR12R1 & 0x0FFFU));
  sweep_uart_puts(",dor,");
  sweep_uart_print_uint((uint32_t)(DAC1->DOR1 & 0x0FFFU));
  sweep_uart_puts(",ndtr,");
  sweep_uart_print_uint((uint32_t)__HAL_DMA_GET_COUNTER(&hdma_dac1_ch1));
  sweep_uart_puts("\r\n");
}

static void iir_process_pending(void)
{
  uint8_t flags;

  __disable_irq();
  flags = iir_process_flags;
  if (flags == 0U)
  {
    __enable_irq();
    return;
  }

  if ((flags & 0x01U) != 0U)
  {
    iir_process_flags &= (uint8_t)~0x01U;
    __enable_irq();
    iir_process_block(0U);
    return;
  }

  if ((flags & 0x02U) != 0U)
  {
    iir_process_flags &= (uint8_t)~0x02U;
    __enable_irq();
    iir_process_block(IIR_BLOCK_SAMPLES);
    return;
  }

  __enable_irq();
}

static uint16_t iir_float_to_dac(float value)
{
  float dac = 2048.0f + (value * 2047.0f);

  if (dac < 0.0f)
  {
    dac = 0.0f;
  }
  else if (dac > 4095.0f)
  {
    dac = 4095.0f;
  }

  return (uint16_t)(dac + 0.5f);
}

static int16_t iir_float_to_fmac_q15(float value)
{
  if (value > 0.9999695f)
  {
    iir_fmac_coeff_clipped = 1U;
    value = 0.9999695f;
  }
  else if (value < -1.0f)
  {
    iir_fmac_coeff_clipped = 1U;
    value = -1.0f;
  }

  if (value >= 0.0f)
  {
    return (int16_t)((value * 32768.0f) + 0.5f);
  }
  return (int16_t)((value * 32768.0f) - 0.5f);
}

static uint8_t iir_fmac_configure_from_coefficients(void)
{
  FMAC_FilterConfigTypeDef config = {0};

  iir_fmac_coeff_clipped = 0U;
  iir_fmac_coeff_b[0] = iir_float_to_fmac_q15(iir_b0);
  iir_fmac_coeff_b[1] = iir_float_to_fmac_q15(iir_b1);
  iir_fmac_coeff_b[2] = iir_float_to_fmac_q15(iir_b2);
  iir_fmac_coeff_a[0] = iir_float_to_fmac_q15(-iir_a1);
  iir_fmac_coeff_a[1] = iir_float_to_fmac_q15(-iir_a2);

  iir_fmac_zero[0] = 0;
  iir_fmac_zero[1] = 0;
  iir_fmac_zero[2] = 0;

  (void)HAL_FMAC_FilterStop(&hfmac);
  if (HAL_FMAC_DeInit(&hfmac) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    return 0U;
  }

  config.InputBaseAddress = 0U;
  config.InputBufferSize = 128U;
  config.InputThreshold = FMAC_THRESHOLD_1;
  config.CoeffBaseAddress = 128U;
  config.CoeffBufferSize = 8U;
  config.OutputBaseAddress = 136U;
  config.OutputBufferSize = 120U;
  config.OutputThreshold = FMAC_THRESHOLD_1;
  config.pCoeffB = iir_fmac_coeff_b;
  config.CoeffBSize = IIR_FMAC_COEFF_B_SIZE;
  config.pCoeffA = iir_fmac_coeff_a;
  config.CoeffASize = IIR_FMAC_COEFF_A_SIZE;
  config.InputAccess = FMAC_BUFFER_ACCESS_POLLING;
  config.OutputAccess = FMAC_BUFFER_ACCESS_POLLING;
  config.Clip = FMAC_CLIP_ENABLED;
  config.Filter = FMAC_FUNC_IIR_DIRECT_FORM_1;
  config.P = IIR_FMAC_COEFF_B_SIZE;
  config.Q = IIR_FMAC_COEFF_A_SIZE;
  config.R = 0U;

  if (HAL_FMAC_FilterConfig(&hfmac, &config) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_FilterPreload(&hfmac, iir_fmac_zero, IIR_FMAC_COEFF_A_SIZE,
                             iir_fmac_zero, IIR_FMAC_COEFF_A_SIZE) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_FilterStart(&hfmac, NULL, NULL) != HAL_OK)
  {
    return 0U;
  }

  iir_fmac_ready = 1U;
  return 1U;
}

static uint8_t iir_fmac_process_block(uint32_t offset)
{
  uint16_t input_size = IIR_BLOCK_SAMPLES;
  uint16_t output_size = IIR_BLOCK_SAMPLES;

  if (iir_fmac_ready == 0U)
  {
    return 0U;
  }

  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    iir_fmac_input[i] = (int16_t)((int32_t)iir_adc_buf[offset + i] - 32768);
  }

  if (HAL_FMAC_ConfigFilterOutputBuffer(&hfmac, iir_fmac_output, &output_size) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_AppendFilterData(&hfmac, iir_fmac_input, &input_size) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_PollFilterData(&hfmac, 10U) != HAL_OK)
  {
    return 0U;
  }
  if ((input_size != IIR_BLOCK_SAMPLES) || (output_size != IIR_BLOCK_SAMPLES))
  {
    return 0U;
  }

  return 1U;
}

static void iir_process_block(uint32_t offset)
{
  uint16_t adc_min = 65535U;
  uint16_t adc_max = 0U;
  uint16_t dac_min = 4095U;
  uint16_t dac_max = 0U;

  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_InvalidateDCache_by_Addr((uint32_t *)&iir_adc_buf[offset],
                                 (int32_t)(IIR_BLOCK_SAMPLES * sizeof(iir_adc_buf[0])));
  }

  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    uint16_t adc_sample = iir_adc_buf[offset + i];

    if (adc_sample < adc_min)
    {
      adc_min = adc_sample;
    }
    if (adc_sample > adc_max)
    {
      adc_max = adc_sample;
    }

    iir_float_buf[i] = ((float32_t)adc_sample - 32768.0f) / 32768.0f;
  }

#if (IIR_DEBUG_ADC_TO_DAC != 0U)
  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    uint16_t dac_sample = (uint16_t)(iir_adc_buf[offset + i] >> 4);

    if (dac_sample > 4095U)
    {
      dac_sample = 4095U;
    }
    if (dac_sample < dac_min)
    {
      dac_min = dac_sample;
    }
    if (dac_sample > dac_max)
    {
      dac_max = dac_sample;
    }
    iir_dac_buf[offset + i] = dac_sample;
  }
#elif (IIR_USE_FMAC != 0U)
  if (iir_fmac_process_block(offset) != 0U)
  {
    for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
    {
      int32_t dac_value = ((int32_t)iir_fmac_output[i] + 32768) >> 4;
      uint16_t dac_sample;

      if (dac_value < 0)
      {
        dac_sample = 0U;
      }
      else if (dac_value > 4095)
      {
        dac_sample = 4095U;
      }
      else
      {
        dac_sample = (uint16_t)dac_value;
      }

      if (dac_sample < dac_min)
      {
        dac_min = dac_sample;
      }
      if (dac_sample > dac_max)
      {
        dac_max = dac_sample;
      }
      iir_dac_buf[offset + i] = dac_sample;
    }
  }
  else
  {
    iir_fmac_error_count++;
    arm_biquad_cascade_df1_f32(&iir_biquad, iir_float_buf, iir_float_buf, IIR_BLOCK_SAMPLES);

    for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
    {
      uint16_t dac_sample = iir_float_to_dac(iir_float_buf[i]);

      if (dac_sample < dac_min)
      {
        dac_min = dac_sample;
      }
      if (dac_sample > dac_max)
      {
        dac_max = dac_sample;
      }
      iir_dac_buf[offset + i] = dac_sample;
    }
  }
#else
  arm_biquad_cascade_df1_f32(&iir_biquad, iir_float_buf, iir_float_buf, IIR_BLOCK_SAMPLES);

  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    uint16_t dac_sample = iir_float_to_dac(iir_float_buf[i]);

    if (dac_sample < dac_min)
    {
      dac_min = dac_sample;
    }
    if (dac_sample > dac_max)
    {
      dac_max = dac_sample;
    }
    iir_dac_buf[offset + i] = dac_sample;
  }
#endif

  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_CleanDCache_by_Addr((uint32_t *)&iir_dac_buf[offset],
                            (int32_t)(IIR_BLOCK_SAMPLES * sizeof(iir_dac_buf[0])));
  }

  if (adc_min < iir_adc_min)
  {
    iir_adc_min = adc_min;
  }
  if (adc_max > iir_adc_max)
  {
    iir_adc_max = adc_max;
  }
  if (dac_min < iir_dac_min)
  {
    iir_dac_min = dac_min;
  }
  if (dac_max > iir_dac_max)
  {
    iir_dac_max = dac_max;
  }
  iir_processed_count++;
  iir_debug_report_once();
}

static void iir_debug_report_once(void)
{
  if (iir_processed_count >= iir_next_debug_count)
  {
    iir_next_debug_count = iir_processed_count + IIR_DEBUG_REPORT_BLOCKS;
    sweep_uart_puts("IIR_DBG,half,");
    sweep_uart_print_uint(iir_half_irq_count);
    sweep_uart_puts(",full,");
    sweep_uart_print_uint(iir_full_irq_count);
    sweep_uart_puts(",dac_half,");
    sweep_uart_print_uint(iir_dac_half_irq_count);
    sweep_uart_puts(",dac_full,");
    sweep_uart_print_uint(iir_dac_full_irq_count);
    sweep_uart_puts(",dac_err,");
    sweep_uart_print_uint(iir_dac_error_count);
    sweep_uart_puts(",fmac_err,");
    sweep_uart_print_uint(iir_fmac_error_count);
    sweep_uart_puts(",proc,");
    sweep_uart_print_uint(iir_processed_count);
    sweep_uart_puts(",adc_min,");
    sweep_uart_print_uint(iir_adc_min);
    sweep_uart_puts(",adc_max,");
    sweep_uart_print_uint(iir_adc_max);
    sweep_uart_puts(",dac_min,");
    sweep_uart_print_uint(iir_dac_min);
    sweep_uart_puts(",dac_max,");
    sweep_uart_print_uint(iir_dac_max);
    sweep_uart_puts(",sr,");
    sweep_uart_print_uint((uint32_t)DAC1->SR);
    sweep_uart_puts(",dhr,");
    sweep_uart_print_uint((uint32_t)(DAC1->DHR12R1 & 0x0FFFU));
    sweep_uart_puts(",dor,");
    sweep_uart_print_uint((uint32_t)(DAC1->DOR1 & 0x0FFFU));
    sweep_uart_puts(",ndtr,");
    sweep_uart_print_uint((uint32_t)__HAL_DMA_GET_COUNTER(&hdma_dac1_ch1));
    sweep_uart_puts("\r\n");
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    const char *cmd = UART1_CMD_START2;

    if (uart1_rx_byte == (uint8_t)cmd[uart1_start2_match])
    {
      uart1_start2_match++;
      if (uart1_start2_match >= UART1_CMD_START2_LEN)
      {
        uart1_start2_requested = 1U;
        uart1_start2_match = 0U;
      }
    }
    else
    {
      uart1_start2_match = (uart1_rx_byte == (uint8_t)cmd[0]) ? 1U : 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    if (iir_running != 0U)
    {
      iir_full_irq_count++;
      if ((iir_process_flags & 0x02U) != 0U)
      {
        iir_overrun = 1U;
      }
      iir_process_flags |= 0x02U;
    }
    else
    {
      adc_done = 1U;
    }
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (iir_running != 0U))
  {
    iir_half_irq_count++;
    if ((iir_process_flags & 0x01U) != 0U)
    {
      iir_overrun = 1U;
    }
    iir_process_flags |= 0x01U;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_error = 1U;
  }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac_rounds++;
    if (iir_running != 0U)
    {
      iir_dac_full_irq_count++;
    }
  }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if ((hdac->Instance == DAC1) && (iir_running != 0U))
  {
    iir_dac_half_irq_count++;
  }
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    iir_dac_error_count++;
    __HAL_DAC_CLEAR_FLAG(hdac, DAC_FLAG_DMAUDR1);
  }
}

static float sweep_sinf(float x)
{
  return sinf(x);
}

static float sweep_cosf(float x)
{
  return cosf(x);
}

static float sweep_sqrtf(float x)
{
  float normalized;
  float scale;
  float y;

  if (x <= 0.0f)
  {
    return 0.0f;
  }

  normalized = x;
  scale = 1.0f;
  while (normalized > 4.0f)
  {
    normalized *= 0.25f;
    scale *= 2.0f;
  }
  while (normalized < 0.25f)
  {
    normalized *= 4.0f;
    scale *= 0.5f;
  }

  y = normalized;
  for (uint32_t i = 0; i < 6U; i++)
  {
    y = 0.5f * (y + (normalized / y));
  }

  return y * scale;
}

static float sweep_atanf(float x)
{
  float sign = 1.0f;

  if (x < 0.0f)
  {
    sign = -1.0f;
    x = -x;
  }

  if (x > 1.0f)
  {
    return sign * ((0.5f * PI_F) - sweep_atanf(1.0f / x));
  }

  return sign * (x / (1.0f + (0.280872f * x * x)));
}

static float sweep_atan2f(float y, float x)
{
  if (x > 0.0f)
  {
    return sweep_atanf(y / x);
  }
  if (x < 0.0f)
  {
    if (y >= 0.0f)
    {
      return sweep_atanf(y / x) + PI_F;
    }
    return sweep_atanf(y / x) - PI_F;
  }
  if (y > 0.0f)
  {
    return 0.5f * PI_F;
  }
  if (y < 0.0f)
  {
    return -0.5f * PI_F;
  }
  return 0.0f;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
