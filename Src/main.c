/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "mcp4131.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t channels[6];
} ADC_MeasurementData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PER_ADC_CHANNEL_COUNT 3U
#define MAX_SAMPLES   50  // Cantidad máxima de muestras por periodo
#define MAX_RMS       10    // Cantidad muestras RMS por canal para enviar por UART
#define HYST          50    // Histéresis para cruce (cuentas ADC)
#define OFFSET        2048  // adc de 2^12 = 4096 bits -> offset = 2^12 / 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ADC multimode DMA buffer: each entry is a 32-bit word where
 * lower 16 bits = ADC1 sample, upper 16 bits = ADC2 sample.
 */
static uint32_t adc_dma_buf[PER_ADC_CHANNEL_COUNT];
static ADC_MeasurementData_t adcIncData;

static float sample_buffer[6][MAX_SAMPLES];
static uint16_t sample_index = 0;

static float v1_last_sample = 0;
static uint8_t period_started = 0;

static float rms_result[6][MAX_RMS];
static uint16_t rms_index = 0;
volatile uint8_t uartReady = 1;
volatile uint8_t adcDataReady = 0;

char msg[128];
/* Transmit buffer for RMS message - must persist while DMA transmits */
static char rms_tx_buf[128];

/* MCP4131 Digital Potentiometer handles */
static MCP4131_HandleTypeDef hpot3;  /* CS3 */
static MCP4131_HandleTypeDef hpot4;  /* CS4 */
static MCP4131_HandleTypeDef hpot5;  /* CS5 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float calculate_rms(float *buffer, uint16_t samples);
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
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  HAL_TIM_Base_Start(&htim3); // Start Timer3 (Trigger Source For ADC1)

  HAL_ADC_Start(&hadc2);
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dma_buf, PER_ADC_CHANNEL_COUNT) != HAL_OK) {
    /* Start Error */
    Error_Handler();
  }

  /* Initialize MCP4131 digital potentiometers */
  MCP4131_Init(&hpot3, &hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin);
  MCP4131_Init(&hpot4, &hspi1, SPI1_CS4_GPIO_Port, SPI1_CS4_Pin);
  MCP4131_Init(&hpot5, &hspi1, SPI1_CS5_GPIO_Port, SPI1_CS5_Pin);

  uint8_t wiper_value = 0;
  uint8_t pot_index = 0;  /* Which potentiometer to update this cycle (0, 1, 2) */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Update one potentiometer per cycle via DMA (round-robin to avoid bus conflicts) */
    switch (pot_index) {
      case 0:
        if (MCP4131_IsReady(&hpot3)) {
          MCP4131_WriteWiper_DMA(&hpot3, wiper_value);
          pot_index = 1;
        }
        break;
      case 1:
        if (MCP4131_IsReady(&hpot4)) {
          MCP4131_WriteWiper_DMA(&hpot4, wiper_value);
          pot_index = 2;
        }
        break;
      case 2:
        if (MCP4131_IsReady(&hpot5)) {
          MCP4131_WriteWiper_DMA(&hpot5, wiper_value);
          pot_index = 0;
          /* All 3 pots updated, increment wiper value for next round */
          wiper_value = (wiper_value + 1) % (MCP4131_WIPER_MAX + 1);
        }
        break;
    }

    // Esperar a que haya datos nuevos de ADC
    if (!adcDataReady) {
      continue;
    }

    // Copiar datos ADC de forma atómica para evitar carreras con ISR
    ADC_MeasurementData_t adcData;
    __disable_irq();
    adcData = adcIncData;
    adcDataReady = 0;
    __enable_irq();

    // === 1) Usar v1 como señal de tensión para cruce por cero ===
    float v1_sample = (float)adcData.channels[0] - OFFSET; // Canal 0 de ADC1

    // === 2) Guardar en buffer para RMS ===
    if (sample_index < MAX_SAMPLES) {
      for (int channel = 0; channel < 6; channel++) {
        sample_buffer[channel][sample_index] = (float)adcData.channels[channel] - OFFSET;
      }
      sample_index++;
    }

    // === 3) Detector de cruce por cero con histéresis ===
    // cruce ascendente (negativo a positivo)
    if (!period_started) {
      if (v1_last_sample < -HYST && v1_sample > HYST) {
        // comienzo de periodo
        for (int channel = 0; channel < 6; channel++) {
          sample_buffer[channel][sample_index] =
              (float)adcData.channels[channel] - OFFSET;
        }
        sample_index++;
        period_started = 1;
      }
    } else {
      // si se cruza nuevamente → fin de periodo
      if (v1_last_sample < -HYST && v1_sample > HYST) {
        // === FIN DEL PERIODO ===

        // 4) Calcular RMS del periodo
        for (int channel = 0; channel < 6; channel++) {
          rms_result[channel][rms_index] = calculate_rms(sample_buffer[channel], sample_index);
        }

        if (rms_index < (MAX_RMS - 1)) {
          rms_index++;
        } else {
          // Buffer RMS lleno
          rms_index = 0;
        }

        // limpiar buffer para siguiente periodo
        sample_index = 0;
        period_started = 1; // sigue en modo midiendo

        // Enviar resultados RMS por UART si está listo
        int len = 0;
        for (uint32_t ch = 0; ch < (PER_ADC_CHANNEL_COUNT * 2) && len < (int)sizeof(rms_tx_buf); ++ch) {
          len += snprintf(rms_tx_buf + len, sizeof(rms_tx_buf) - (size_t)len, "RMS%u:%u ",
                          (unsigned int)ch, (unsigned int)rms_result[ch][rms_index]);
        }
        if (len < (int)sizeof(rms_tx_buf)) {
          len += snprintf(rms_tx_buf + len, sizeof(rms_tx_buf) - (size_t)len, "\r\n");
        }
        if (len > 0 && uartReady) {
          HAL_UART_Transmit_DMA(&huart1, (uint8_t *)rms_tx_buf, (uint16_t)len);
          uartReady = 0;
        }
      }
    }

    v1_last_sample = v1_sample;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance != ADC1) {
    return;
  }

  /* Unpack DMA multimode buffer into adcData struct (6 channels)
   * lower 16 bits = ADC1 sample -> channels[0..PER_ADC_CHANNEL_COUNT-1]
   * upper 16 bits = ADC2 sample -> channels[PER_ADC_CHANNEL_COUNT..(2*PER_ADC_CHANNEL_COUNT-1)]
   */
  for (uint32_t i = 0; i < PER_ADC_CHANNEL_COUNT; ++i) {
    uint32_t packed = adc_dma_buf[i];
    uint16_t s1 = (uint16_t)(packed & 0xFFFFU);
    uint16_t s2 = (uint16_t)((packed >> 16) & 0xFFFFU);

    adcIncData.channels[i] = s1;
    adcIncData.channels[i + PER_ADC_CHANNEL_COUNT] = s2;
  }

  /* Prepare a single message with all 6 channels taken from adcData */
  int len = 0;
  for (uint32_t ch = 0; ch < (PER_ADC_CHANNEL_COUNT * 2) && len < (int)sizeof(msg); ++ch) {
    len += snprintf(msg + len, sizeof(msg) - (size_t)len, "CH%u:%u ",
                    (unsigned int)ch, (unsigned int)adcIncData.channels[ch]);
  }
  if (len < (int)sizeof(msg)) {
    len += snprintf(msg + len, sizeof(msg) - (size_t)len, "\r\n");
  }
  if (len > 0 && uartReady) {
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)msg, (uint16_t)len);
    uartReady = 0;
  }

  /* Signal main loop that new ADC data is ready */
  adcDataReady = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uartReady = 1;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    /* Check which potentiometer completed and release its CS */
    if (hpot3.busy && hpot3.hspi == hspi) {
      MCP4131_TxCpltCallback(&hpot3);
    } else if (hpot4.busy && hpot4.hspi == hspi) {
      MCP4131_TxCpltCallback(&hpot4);
    } else if (hpot5.busy && hpot5.hspi == hspi) {
      MCP4131_TxCpltCallback(&hpot5);
    }
  }
}

float calculate_rms(float *buffer, uint16_t samples)
{
    if (samples == 0)
        return 0.0f;

    float sum = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
        sum += buffer[i] * buffer[i];

    return sqrtf(sum / samples);
}

/* USER CODE END 4 */

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
