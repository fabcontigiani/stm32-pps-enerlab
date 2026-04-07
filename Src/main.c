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
#include "mcp4131.c"
#include "ftoa.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t channels[8];
} ADC_MeasurementData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PER_ADC_CHANNEL_COUNT 4U
#define TOTAL_CHANNELS        6U
#define TOTAL_PHASES          3U
#define MAX_SAMPLES           120  // Cantidad máxima de muestras por periodo
#define MAX_RMS               128    // Cantidad muestras RMS por canal
#define MAX_RMS_PROM          10    // Cantidad de valores RMS promediados
#define HYST                  40    // Histéresis para cruce (cuentas ADC)
#define I_MAX                 1638  // 80% = 0.8 * 4095 / 2
#define I_MIN                 512 // 25% = 0.25 * 4095 / 2 
#define TOTAL_GAIN_CURRENT    7U
#define ZERO_CROSS_TIMEOUT    200U // Conteo de muestras ADC antes de iniciar medición sin cruce por cero
/*
#define V1_GAIN               (222.0f / (0.6505f * 4095.f))
#define V2_GAIN               (222.0f / (0.6505f * 4095.f))
#define V3_GAIN               (222.0f / (0.6580f * 4095.f))
#define I1_GAIN               (10.51f / (0.164f * 4095.f))
#define I2_GAIN               (10.46f / (0.164f * 4095.f))
#define I3_GAIN               (10.51f / (0.168f * 4095.f))
*/
#define V1_GAIN 1
#define V2_GAIN 1
#define V3_GAIN 1
#define I1_GAIN 1
#define I2_GAIN 1
#define I3_GAIN 1

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

char msg[128];
/* Transmit buffer for RMS message - must persist while DMA transmits */
static char rms_tx_buf[128];

/* MCP4131 Digital Potentiometer handles */
static MCP4131_HandleTypeDef hpot3;  /* CS3 */
static MCP4131_HandleTypeDef hpot4;  /* CS4 */
static MCP4131_HandleTypeDef hpot5;  /* CS5 */

/*flags*/
volatile uint8_t flag_adc_ready = 0;
volatile uint8_t uartReady = 1;
static uint8_t adc_calibrated = 0;
static uint8_t en_region_alta = 0;
static uint8_t muestreo = 0;
static uint8_t primer_periodo = 1;
static uint8_t calculos_ready = 0;
static uint16_t zero_cross_timeout_counter = 0;

static uint8_t banderaMedicion = 0;

/*buffers*/
static int16_t sample_buffer[TOTAL_CHANNELS][MAX_SAMPLES];  //se almacenan muestras de un periodo
static float rms_buffer[TOTAL_CHANNELS][MAX_RMS];           //se almacenan valores RMS de un periodo
static float P_buffer[TOTAL_PHASES][MAX_RMS];               //se almacenan valores de potencia activa por periodo

static float rms_prom_buffer[TOTAL_CHANNELS][MAX_RMS_PROM];      //se almacenan valores RMS promediados
static float P_prom_buffer[TOTAL_PHASES][MAX_RMS_PROM];          //se almacenan valores de potencia activa promediados

static float rms_total[TOTAL_CHANNELS];
static float P_total[TOTAL_PHASES];
static float S[TOTAL_PHASES];
static float FP[TOTAL_PHASES];

static float rms_real[TOTAL_CHANNELS];                      //valores RMS convertidos a voltaje y corriente 

static int16_t i_max[TOTAL_PHASES] = {0,0,0};  //se almacena el pico maximo de corriente de cada fase por periodo
static int16_t i_min[TOTAL_PHASES] = {0,0,0};  //se almacena el pico minimo de corriente de cada fase por periodo


static int8_t wiper[TOTAL_PHASES] = {6, 6, 6};
static uint8_t cambio_wiper[TOTAL_PHASES] = {0, 0, 0};
static uint8_t count_cambio_wiper[TOTAL_PHASES] = {0, 0, 0};
const uint8_t wiper_position[TOTAL_GAIN_CURRENT] = {64, 85, 102, 113, 120, 124, 126};
const uint8_t wiper_position_reverse[TOTAL_GAIN_CURRENT] = {64, 43, 26, 15, 8, 4, 2};
static float gain_table[TOTAL_PHASES] = {1, 1, 1};


static const uint8_t valid_channels[TOTAL_CHANNELS] = {0,1,2,6,4,5};

/*indices*/
static uint8_t sample_index = 0;
static int16_t rms_index = -1;
static int16_t rms_prom_index = -1;


static float vdda = 3.3f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float calculate_rms(int16_t *buffer, uint8_t samples);
static float adc_to_voltage(float adc_value, uint8_t phase);
static float adc_to_current(float adc_value, float gain, uint8_t phase);
void AdjustCurrentGain_Wiper(void);
//uint8_t reverse_vector(const uint8_t *vector, uint8_t index);
float calculate_gain(uint8_t wiper_position, uint8_t invertido);

static float calculate_active_power(int16_t *buffer_tension, int16_t *buffer_corriente, uint8_t samples);
static float calculate_mean(float *buffer, uint8_t samples);

void vdda_calibrated(void);

  

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
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dma_buf, PER_ADC_CHANNEL_COUNT);

  /* Initialize MCP4131 digital potentiometers */
  MCP4131_Init(&hpot3, &hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin);
  MCP4131_Init(&hpot4, &hspi1, SPI1_CS4_GPIO_Port, SPI1_CS4_Pin);
  MCP4131_Init(&hpot5, &hspi1, SPI1_CS5_GPIO_Port, SPI1_CS5_Pin);


  int16_t v1 = 0; // Tension fase 1 -> referencia para cruce por cero
  uint8_t cruce_ascendente = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Calibra Vdda y termina el ciclo
    if(!adc_calibrated && flag_adc_ready) {
      vdda_calibrated();
      continue;
    }

    // Esperar a que haya datos nuevos de ADC
    if(!flag_adc_ready) {
      continue;
    }

    // Copiar datos ADC de forma atómica para evitar carreras con ISR
    ADC_MeasurementData_t adcData;
    __disable_irq();
    adcData = adcIncData;
    flag_adc_ready = 0;
    __enable_irq();

    // === 1. Leer muestra ADC (referencia para inicio de periodo) ===
    v1 = adcData.channels[0] - adcData.channels[7];

    // === 2. Detección de cruce por cero con histéresis ===
    cruce_ascendente = 0;

    if (!en_region_alta && (v1 > HYST)) {
      en_region_alta = 1;
      cruce_ascendente = 1;
    }
    else if (en_region_alta && (v1 < -HYST)) {
      en_region_alta = 0;
    }

    // Si no hay cruce por cero durante un tiempo, forzar inicio de medición
    if (!muestreo && !cruce_ascendente) {
      if (++zero_cross_timeout_counter >= ZERO_CROSS_TIMEOUT) {
        zero_cross_timeout_counter = 0;
        muestreo = 1;
        sample_index = 0;
      }
    } else if (cruce_ascendente) {
      zero_cross_timeout_counter = 0;
    }

    // === 3. Gestión de inicio / fin de período ===
    if (cruce_ascendente){
      if (!muestreo) {
        // ---- INICIO DE PERÍODO ----
        muestreo = 1;
        sample_index = 0;
      }
      else {
        // ---- FIN DE PERÍODO ----
        muestreo = 0;

        if (primer_periodo) {
          // descartar el primer período
          primer_periodo = 0;
        }
        else {
          rms_index = (rms_index + 1) % MAX_RMS;

          if(!rms_index){
            count_cambio_wiper[0] = 0;
            count_cambio_wiper[1] = 0;
            count_cambio_wiper[2] = 0;
          }

          // Evalua saturacion o cambio de wiper
          AdjustCurrentGain_Wiper();

          // Si no se cambia wiper de la fase ph
          // Calcula Vrms, Irms, Pot activa, Pot aparente y cos(phi)
          for(uint8_t ph = 0; ph < TOTAL_PHASES; ph++) {
            if(!cambio_wiper[ph]){
              rms_buffer[ph][rms_index - count_cambio_wiper[ph]] = calculate_rms(sample_buffer[ph],sample_index);
              rms_buffer[ph + TOTAL_PHASES][rms_index - count_cambio_wiper[ph]] = calculate_rms(sample_buffer[ph + TOTAL_PHASES],sample_index);

              rms_real[ph] = adc_to_voltage(rms_buffer[ph][rms_index - count_cambio_wiper[ph]], ph);
              rms_real[ph+TOTAL_PHASES] = adc_to_current(rms_buffer[ph + TOTAL_PHASES][rms_index - count_cambio_wiper[ph]],gain_table[ph], ph);
              /*
              if(ph == 1){
                rms_real[ph + TOTAL_PHASES] = adc_to_current(rms_buffer[ph + TOTAL_PHASES][rms_index - count_cambio_wiper[ph]],gain_table[ph], ph);
              }else{
                rms_real[ph + TOTAL_PHASES] = adc_to_current(rms_buffer[ph + TOTAL_PHASES][rms_index - count_cambio_wiper[ph]],gain_table[ph], ph);
              }
              */

              rms_buffer[ph + TOTAL_PHASES][rms_index - count_cambio_wiper[ph]] = adc_to_current(rms_buffer[ph + TOTAL_PHASES][rms_index - count_cambio_wiper[ph]],gain_table[ph], ph);
              P_buffer[ph][rms_index - count_cambio_wiper[ph]] = calculate_active_power(sample_buffer[ph], sample_buffer[ph+TOTAL_PHASES], sample_index);
              P_buffer[ph][rms_index - count_cambio_wiper[ph]] = adc_to_voltage(P_buffer[ph][rms_index - count_cambio_wiper[ph]], ph);
              P_buffer[ph][rms_index - count_cambio_wiper[ph]] = adc_to_current(P_buffer[ph][rms_index - count_cambio_wiper[ph]], gain_table[ph], ph);

              int len = 0;
              for (uint32_t ch = 0; ch < (TOTAL_CHANNELS) && len < (int)sizeof(rms_tx_buf); ++ch) {
                char rms_str[24];
                ftoa(rms_str, rms_real[ch], NULL);
                len += snprintf(rms_tx_buf + len, sizeof(rms_tx_buf) - (size_t)len, "RMS%u:%s ",
                                (unsigned int)ch, rms_str);
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

          if(rms_index == MAX_RMS - 1){
            rms_prom_index = (rms_prom_index + 1) % MAX_RMS_PROM;
            
            for(uint8_t ph = 0; ph < TOTAL_PHASES; ph++) {
              rms_prom_buffer[ph][rms_prom_index] = calculate_mean(rms_buffer[ph], 1 + rms_index - count_cambio_wiper[ph]);
              rms_prom_buffer[ph + TOTAL_PHASES][rms_prom_index] = calculate_mean(rms_buffer[ph + TOTAL_PHASES], 1 + rms_index - count_cambio_wiper[ph + TOTAL_PHASES]);

              //rms_real[ph] = adc_to_voltage(rms_prom_buffer[ph][rms_prom_index]);

              P_prom_buffer[ph][rms_prom_index] = calculate_mean(P_buffer[ph], 1 + rms_index - count_cambio_wiper[ph]);
            }
          }

          if(rms_prom_index == MAX_RMS_PROM - 1){
            for (uint8_t ph = 0; ph < TOTAL_PHASES; ph++) {
              rms_total[ph] = calculate_mean(rms_prom_buffer[ph], MAX_RMS_PROM);
              rms_total[ph + TOTAL_PHASES] = calculate_mean(rms_prom_buffer[ph + TOTAL_PHASES], MAX_RMS_PROM);

              rms_total[ph] = adc_to_voltage(rms_total[ph], ph);
              
              P_total[ph] = calculate_mean(P_prom_buffer[ph], MAX_RMS_PROM);

              S[ph] = rms_total[ph] * rms_total[ph + TOTAL_PHASES];

              FP[ph] = P_total[ph] / S[ph];
              rms_prom_index = 0;

            }
            //rms_prom_index = 0;
            calculos_ready = 1; // listo para enviar por UART
            adc_calibrated = 0; // 0 para volver a calibrar Vdda

          }
        }
      }
    }

    // === 4. Acumulación de muestras SOLO dentro del período ===
    if (muestreo) {
      if (sample_index < MAX_SAMPLES) {
        for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++) {
          //en el buffer "valid_channels" se tienen los canales de tension y corriente (se omite ch3 con VREFINT y ch7 con OFFSET)
          sample_buffer[ch][sample_index] = (int16_t)adcData.channels[valid_channels[ch]] - (int16_t)adcData.channels[7];
        }

        // Cálculo de valores máximos y mínimos para cada fase (canales de corriente)
        if (sample_index == 0){
          // Primera muestra del período -> reset de picos de corriente
          for (uint8_t ph = 0; ph < TOTAL_PHASES; ph++){
            i_max[ph] = 0;
            i_min[ph] = 0;
          }
        }else{        //UART FABRI
          // Actualiza máximos y mínimos de corriente durante el período
          for (uint8_t ph = 0; ph < TOTAL_PHASES; ph++){
            if(sample_buffer[ph+TOTAL_PHASES][sample_index] > i_max[ph]){
              i_max[ph] = sample_buffer[ph+TOTAL_PHASES][sample_index];
            }else if(sample_buffer[ph+TOTAL_PHASES][sample_index] < i_min[ph]){
              i_min[ph] = sample_buffer[ph+TOTAL_PHASES][sample_index];
            }
          }
        }
        // Avanza al siguiente sample del período
        sample_index++;
      }
      else {
        // seguridad: overflow del buffer
        muestreo = 0;
        sample_index = 0;
      }
    }

  




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /* Unpack DMA multimode buffer into adcData struct (8 channels)
   * lower 16 bits = ADC1 sample -> channels[0..PER_ADC_CHANNEL_COUNT-1]
   * upper 16 bits = ADC2 sample -> channels[PER_ADC_CHANNEL_COUNT..(2*PER_ADC_CHANNEL_COUNT-1)]
   */
  for (uint32_t i = 0; i < PER_ADC_CHANNEL_COUNT; i++) {
    uint32_t packed = adc_dma_buf[i];
    adcIncData.channels[i] = (uint16_t)(packed & 0xFFFF);
    adcIncData.channels[i + PER_ADC_CHANNEL_COUNT] = (uint16_t)((packed >> 16) & 0xFFFF);
  }

/*
  // Prepare a single message with all 6 channels taken from adcData
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
*/

  /* Signal main loop that new ADC data is ready */
  flag_adc_ready = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    uartReady = 1;
  }
}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    // Check which potentiometer completed and release its CS
    if (hpot3.busy && hpot3.hspi == hspi) {
      MCP4131_TxCpltCallback(&hpot3);
    } else if (hpot4.busy && hpot4.hspi == hspi) {
      MCP4131_TxCpltCallback(&hpot4);
    } else if (hpot5.busy && hpot5.hspi == hspi) {
      MCP4131_TxCpltCallback(&hpot5);
    }
  }
}


static float calculate_rms(int16_t *buffer, uint8_t samples) {
  if (samples == 0) return 0.0f;

  float sum = 0.0f;
  for (uint16_t i = 0; i < samples; i++) {
      sum += buffer[i] * buffer[i];
  }
  return (sqrtf((float) sum / samples));
}

void vdda_calibrated(void) {
  ADC_MeasurementData_t adcData;
  __disable_irq();
  adcData = adcIncData;
  flag_adc_ready = 0;
  __enable_irq();

  uint16_t adc_vrefint = adcData.channels[3];

  vdda = (1.21 * 4095.0f) / adc_vrefint;

  adc_calibrated = 1;
}

static float adc_to_voltage(float adc_value, uint8_t phase)
{
  // ECUACION: V = adc_value * (vdda/4095) * (Vi/Vo)

  switch(phase){
    case 0:
      return (float)adc_value * vdda * V1_GAIN;
      break;
    case 1:
      return (float)adc_value * vdda * V2_GAIN;
      break;
    case 2:
      return (float)adc_value * vdda * V3_GAIN;
      break;
  }
  // calculo antiguo, funciona bien
  //return ((float)adc_value * vdda * 0.08153905715f);//adc_value*(197.7/0.5842)*(vdda/4095)
}

static float adc_to_current(float adc_value, float gain, uint8_t phase)
{
  switch(phase){
    case 0:
      return (float) (adc_value * vdda / gain) * I1_GAIN;
      break;
    case 1:
      return (float) (adc_value * vdda / gain) * I2_GAIN;
      break;
    case 2:
      return (float) (adc_value * vdda / gain) * I3_GAIN;
      break;
  }
    //return (float) ((adc_value * vdda * 2000.f) / (gain * 33.f * 4095.f));
}

void AdjustCurrentGain_Wiper(void){
  for(uint8_t phase = 0; phase < TOTAL_PHASES; phase++){
    cambio_wiper[phase] = 0;

    // Condicionales para detectar si se debe cambiar el wiper
    if(i_max[phase] > I_MAX || i_min[phase] < -I_MAX){
      cambio_wiper[phase] = 1;
      wiper[phase]--;
      if(wiper[phase] < 0){
        wiper[phase] = 0;
      }
    }else if(i_max[phase] < I_MIN && i_min[phase] > -I_MIN){
      cambio_wiper[phase] = 1;
      wiper[phase]++;
      if(wiper[phase] > 6){
        wiper[phase] = 6;
        cambio_wiper[phase] = 0;
      }
    }
    
    if(rms_index == 0){
      cambio_wiper[phase] = 1;
      count_cambio_wiper[phase]++;
    }

    // Comunicacion con Pote Digital 
    if(cambio_wiper[phase]){
      count_cambio_wiper[phase]++;
      
      switch(phase){
        case 1:
          if (MCP4131_IsReady(&hpot3)) {
            MCP4131_WriteWiper_DMA(&hpot3,wiper_position_reverse[wiper[phase]]);
            gain_table[phase] = calculate_gain(wiper_position_reverse[wiper[phase]], 1);
          }
          break;
        case 2:
          if (MCP4131_IsReady(&hpot4)) {
            MCP4131_WriteWiper_DMA(&hpot4,wiper_position[wiper[phase]]);
            gain_table[phase] = calculate_gain(wiper_position[wiper[phase]], 0);
          }
          break;
        case 0:
          if (MCP4131_IsReady(&hpot5)) {
            MCP4131_WriteWiper_DMA(&hpot5,wiper_position_reverse[wiper[phase]]);
            gain_table[phase] = calculate_gain(wiper_position_reverse[wiper[phase]], 1);
          }
          break;
      }
    }
  }
}

/*
uint8_t reverse_vector(const uint8_t *vector, uint8_t index){
  return (vector[TOTAL_GAIN_CURRENT- 1 - index]);
}
*/

float calculate_gain(uint8_t wiper_position, uint8_t invertido){
  if(invertido){
    return (float) (128.f - wiper_position)/wiper_position;
  }else{
    return (float) wiper_position/(128.f - wiper_position);
  }
}


static float calculate_active_power(int16_t *buffer_tension, int16_t *buffer_corriente, uint8_t samples){
  if (samples == 0) return 0.0f;

  float acc = 0.0f;
  for (uint16_t n = 0; n < samples; n++) {
    acc += (float)buffer_tension[n] * (float)buffer_corriente[n];
  }
  return (acc / (float) samples);
}

static float calculate_mean(float *buffer, uint8_t samples)
{
  if (samples == 0) return 0.0f;

  float acc = 0.0f;
  for (uint16_t i = 0; i < samples; i++) {
      acc += buffer[i];
  }

  return acc / (float)samples;
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
