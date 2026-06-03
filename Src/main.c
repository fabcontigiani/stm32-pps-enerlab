/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * CAMBIOS RESPECTO A LA VERSION ANTERIOR:
  *
  * [1] POTENCIA ACTIVA P — ahora se calcula con producto directo muestra×muestra
  *     (calculate_active_power) en lugar de DFT bin único.
  *     Ventajas:
  *       - Captura fundamental + armónicos (P total, IEEE 1459-2010)
  *       - El signo es correcto por naturaleza (positivo = consume, negativo = genera)
  *       - No requiere negación manual
  *
  * [2] POTENCIA REACTIVA Q — se mantiene DFT bin k=1 (solo fundamental)
  *     pero se elimina la negación arbitraria. El signo queda determinado
  *     por la convención e^{-jwt} de calculate_single_bin_dft():
  *       Q > 0 : carga inductiva (I retrasada respecto a V)
  *       Q < 0 : carga capacitiva / generación con adelanto de corriente
  *
  * [3] POTENCIA APARENTE S — siempre positiva: S = Vrms * Irms
  *     No se puede calcular con signo porque es una magnitud escalar.
  *
  * [4] FACTOR DE POTENCIA FP — ahora FP = P / S (IEEE 1459-2010)
  *     Rango: [-1, +1]
  *       FP > 0 : consumo neto (cuadrantes I y IV)
  *       FP < 0 : generación neta (cuadrantes II y III)
  *     Se agrega FP_angulo = atan2(Q, P) * (180/pi) para el ángulo de fase
  *     completo en [-180°, +180°], que indica el cuadrante exacto.
  *
  * [5] CONVERSIÓN DE UNIDADES para P:
  *     calculate_active_power() devuelve ADC²  (cuentas_V × cuentas_I).
  *     Se aplica la misma escala que antes:
  *       P_real = P_adc2 * vdda² * Kv * Ki / G
  *     implementado en la nueva función adc2_to_power().
  *
  * [6] AGC CON HISTÉRESIS Y SUBIDA GRADUAL — AdjustCurrentGain_Wiper()
  *
  *     Problema 1 resuelto — subida gradual de ganancia:
  *       Si I_MIN_NOISE < i_max < I_MIN_H durante PER_SUBIDA_GAIN períodos
  *       consecutivos → wiper++ (sube un nivel, gradual).
  *       I_MIN_NOISE separa "señal débil real" de "ruido puro".
  *
  *     Problema 2 resuelto — count_cambio_wiper proporcional al salto:
  *       Si wiper baja más de 1 nivel (reset a 0 desde nivel alto),
  *       count_cambio_wiper = salto_niveles + 1 para descartar los
  *       períodos de establecimiento del MCP4131 + OPAMP.
  *
  *     Problema 3 resuelto — histéresis de doble umbral:
  *       Zona ALTA:  i_max > I_MAX    → bajar    (sin histéresis, reacción inmediata)
  *                   i_max < I_MAX_H  → volver OK (I_MAX_H = I_MAX * 0.85)
  *       Zona BAJA:  i_max < I_MIN    → contar períodos bajos
  *                   i_max > I_MIN_H  → volver OK (I_MIN_H = I_MIN * 1.20)
  *       La histéresis evita el chattering cuando la señal oscila cerca
  *       de los umbrales duros.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t channels[8];
} ADC_MeasurementData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION "1.4.1"

#define PER_ADC_CHANNEL_COUNT 4U
#define TOTAL_CHANNELS        6U
#define TOTAL_PHASES          3U
#define MAX_SAMPLES           120 // Cantidad máxima de muestras por periodo
#define MAX_RMS               128 // Cantidad muestras RMS por canal
#define MAX_RMS_PROM          3   // Cantidad de valores RMS promediados 10 = 50 segundos
#define HYST                  40  // Histéresis para cruce (cuentas ADC)

#define I_MAX            1945   /* 95% de fondo de escala = 0.95 * 4095/2    */
#define I_MIN            204    /* 10% de fondo de escala = 0.10 * 4095/2    */
#define I_MIN_NOISE      61     /* < 3% de fondo de escala: ruido puro       */ 

/* --- Períodos para subida gradual de ganancia ---
 *
 * PER_SUBIDA_GAIN: número de períodos consecutivos con señal baja pero
 *   presente (I_MIN_NOISE < i_max < I_MIN) antes de subir un nivel de
 *   ganancia. Evita subir por transitorios cortos.
 *
 * PER_NO_SIGNAL: períodos con señal < I_MIN_NOISE para resetear a G=1×.
 *   Se mantiene en 1 para reacción rápida ante pérdida total de señal.
 */
#define PER_SUBIDA_GAIN     3U
#define TOTAL_GAIN_CURRENT  7U  // cantidad de niveles de ganancia de corriente
#define TIMEOUT_MAX         500U// cantidad de periodos para timeout de cruce por cero 
#define PER_NO_SIGNAL       1U  // cantidad de periodos sin señal para resetear wiper a ganancia mínima

#define CONV_RAD_TO_DEG     57.2957795131f // 180 / pi

const float V1_GAIN = 0.08205239034f;
const float V2_GAIN = 0.08265239034f;
const float V3_GAIN = 0.08255239034f;
const float I1_GAIN = 0.01457762941325099933f;
const float I2_GAIN = 0.01490253612335897156f;
const float I3_GAIN = 0.01518113489719877252f;

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
static char rms_tx_buf[1024];

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
static uint8_t cruce_ascendente = 0;

/*buffers*/
static int16_t sample_buffer[TOTAL_CHANNELS][MAX_SAMPLES];  //se almacenan muestras de un periodo
static float rms_buffer[TOTAL_CHANNELS][MAX_RMS];           //se almacenan valores RMS de un periodo

static float P_buffer[TOTAL_PHASES][MAX_RMS];               //se almacenan valores de potencia activa por periodo
static float Q_buffer[TOTAL_PHASES][MAX_RMS];               //se almacenan valores de potencia reactiva por periodo

static float rms_prom_buffer[TOTAL_CHANNELS][MAX_RMS_PROM];      //se almacenan valores RMS promediados
static float P_prom_buffer[TOTAL_PHASES][MAX_RMS_PROM];          //se almacenan valores de potencia activa promediados
static float Q_prom_buffer[TOTAL_PHASES][MAX_RMS_PROM];          //se almacenan valores de potencia reactiva promediados

static float rms_total[TOTAL_CHANNELS];
static float P_total[TOTAL_PHASES];
static float Q_total[TOTAL_PHASES];
static float S[TOTAL_PHASES];
static float FP[TOTAL_PHASES];
static float phi[TOTAL_PHASES]; //angulo de fase completo en [-180, +180] grados
static float FP_calculado[TOTAL_PHASES]; //factor de potencia calculado a partir del ángulo de fase, para comparación con FP=P/S

static float rms_real[TOTAL_CHANNELS];                      //valores RMS convertidos a voltaje y corriente 

static int16_t i_max[TOTAL_PHASES] = {0,0,0};  //se almacena el pico maximo de corriente de cada fase por periodo
static int16_t i_min[TOTAL_PHASES] = {0,0,0};  //se almacena el pico minimo de corriente de cada fase por periodo


static int8_t wiper[TOTAL_PHASES] = {6, 6, 6};
static uint8_t cambio_wiper[TOTAL_PHASES] = {0, 0, 0};
static uint8_t count_cambio_wiper[TOTAL_PHASES] = {0, 0, 0};
const uint8_t wiper_position[TOTAL_GAIN_CURRENT] = {64, 85, 102, 113, 120, 124, 126};
//const uint8_t wiper_position_reverse[TOTAL_GAIN_CURRENT] = {64, 43, 26, 15, 8, 4, 2};
static float gain_table[TOTAL_PHASES] = {1, 1, 1};

static uint8_t count_noSignal[TOTAL_PHASES] = {0, 0, 0}; // contador de periodos sin señal para cada fase

/* [CAMBIO 6] contador de períodos con señal baja pero presente (para subida gradual) */
static uint8_t count_señalBaja[TOTAL_PHASES] = {0, 0, 0};

static const uint8_t valid_channels[TOTAL_CHANNELS] = {0,1,2,4,5,6};

/*indices*/
static uint8_t sample_index = 0;
static int16_t rms_index = -1;
static int16_t rms_prom_index = -1;

static uint16_t timeout = 0;
static uint16_t timer_cont = 0;

static float vdda = 3.3f;
static int16_t v1 = 0; // Tension fase 1 -> referencia para cruce por cero


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float calculate_rms(int16_t *buffer, uint8_t samples);
static float adc_to_voltage(float adc_value, uint8_t phase);
static float adc_to_current(float adc_value, float gain, uint8_t phase);

static float adc2_to_power(float adc2_value, float gain, uint8_t phase);

void AdjustCurrentGain_Wiper(void);
//uint8_t reverse_vector(const uint8_t *vector, uint8_t index);
float calculate_gain(uint8_t wiper_position, uint8_t invertido);

static float calculate_active_power(int16_t *buffer_v, int16_t *buffer_i, uint8_t samples);
static void calculate_single_bin_dft(int16_t *buffer, uint8_t samples, float *real, float *imag);
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


  v1 = 0; // Tension fase 1 -> referencia para cruce por cero
  cruce_ascendente = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Timeout de cruce por cero para evitar quedarse esperando si la TENSION se pierde o es nula
    muestreo a 5kHz (ts = 0,2ms) 
    eligiendo un timeout de T (TIMEOUT_MAX)
    numero de muestras: n = T/ts = 500/0,2 = 2500
    */
    if(timer_cont > TIMEOUT_MAX){
      timeout = 1;
      flag_adc_ready = 1; // fuerza a esperar nueva muestra de ADC para iniciar nuevo periodo
      adc_calibrated = 1;
    }


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

    cruce_ascendente = 0; //flag de cruce por cero ascendente de un periodo de tension

    if(timeout){
      timeout = 0;
      en_region_alta = 0;
      v1 = HYST + 4; // fuerza cruce por cero para iniciar nuevo periodo
      muestreo = 1;
      sample_index = 0;
    }

    // === 2. Detección de cruce por cero con histéresis ===
    if (!en_region_alta && (v1 > HYST)) {
      en_region_alta = 1;
      cruce_ascendente = 1;
      timer_cont = 0; // reset del timer para timeout de cruce por cero
    }
    else if (en_region_alta && (v1 < -HYST)) {
      en_region_alta = 0;
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

          // Evalua saturacion o cambio de wiper
          AdjustCurrentGain_Wiper();

          // Si no se cambia wiper de la fase ph
          // Calcula Vrms, Irms, Pot activa, Pot aparente y Pot reactiva
          // periodo a periodo
          for(uint8_t ph = 0; ph < TOTAL_PHASES; ph++) {
            if(!cambio_wiper[ph]){
              int idx = (int)rms_index - (int)count_cambio_wiper[ph];
              if (idx < 0) idx += MAX_RMS; // wrap-around seguro

              rms_buffer[ph][idx] = calculate_rms(sample_buffer[ph], sample_index);

              //sample_buffer[ph + TOTAL_PHASES][idx] = - sample_buffer[ph + TOTAL_PHASES][idx];

              rms_buffer[ph + TOTAL_PHASES][idx] = calculate_rms(sample_buffer[ph + TOTAL_PHASES], sample_index);

              rms_buffer[ph + TOTAL_PHASES][idx] = adc_to_current(rms_buffer[ph + TOTAL_PHASES][idx], gain_table[ph], ph);

              rms_real[ph] = adc_to_voltage(rms_buffer[ph][idx], ph);

              rms_real[ph + TOTAL_PHASES] = rms_buffer[ph + TOTAL_PHASES][idx];

              /* ----------------------------------------------------------------
               * POTENCIA ACTIVA P
               * Método: producto directo muestra×muestra (IEEE 1459-2010)
               *   P = (1/N) * sum(v[n] * i[n])
               *
               * La función devuelve ADC² → se convierte con adc2_to_power()
               * que aplica: P_real = P_adc2 * vdda² * Kv * Ki / G
               * ---------------------------------------------------------------- */
              float P_adc2 = calculate_active_power(
                  sample_buffer[ph],
                  sample_buffer[ph + TOTAL_PHASES],
                  sample_index);

              P_buffer[ph][idx] = adc2_to_power(P_adc2, gain_table[ph], ph);

              /* ----------------------------------------------------------------
               * POTENCIA REACTIVA Q
               * Método: DFT bin k=1 (solo armonico fundamental).
               *
               * Convención de signo (con e^{-jwt}, factor 2/N):
               *   Q > 0 → corriente retrasada (carga inductiva)
               *   Q < 0 → corriente adelantada (carga capacitiva o generación)
               *
               * Q = 0.5 * (Vim*Ire - Vre*Iim)
               * ---------------------------------------------------------------- */
              float v_re, v_im, i_re, i_im;
              calculate_single_bin_dft(sample_buffer[ph], sample_index, &v_re, &v_im);
              calculate_single_bin_dft(sample_buffer[ph + TOTAL_PHASES], sample_index, &i_re, &i_im);

              float Q_adc2 = 0.5f * (v_im * i_re - v_re * i_im); /* sin negación */

              Q_buffer[ph][idx] = adc2_to_power(Q_adc2, gain_table[ph], ph);
            }    
          }

          /* --- Nivel 2: promedio de MAX_RMS períodos --- */
          if(rms_index == MAX_RMS - 1){
            rms_prom_index = (rms_prom_index + 1) % MAX_RMS_PROM;

            for(uint8_t ph = 0; ph < TOTAL_PHASES; ph++) {
              uint8_t n_valid = 1 + rms_index - count_cambio_wiper[ph];

              rms_prom_buffer[ph][rms_prom_index] = calculate_mean(rms_buffer[ph], n_valid);

              rms_prom_buffer[ph + TOTAL_PHASES][rms_prom_index] = calculate_mean(rms_buffer[ph + TOTAL_PHASES], n_valid);

              //rms_real[ph] = adc_to_voltage(rms_prom_buffer[ph][rms_prom_index]);

              P_prom_buffer[ph][rms_prom_index] = calculate_mean(P_buffer[ph], n_valid);
              Q_prom_buffer[ph][rms_prom_index] = calculate_mean(Q_buffer[ph], n_valid);
            }
          }

          /* --- Nivel 3: promedio final y cálculo de S, FP --- */
          if(rms_prom_index == MAX_RMS_PROM - 1){
            rms_prom_index = -1;

            for (uint8_t ph = 0; ph < TOTAL_PHASES; ph++){
              rms_total[ph] = adc_to_voltage(calculate_mean(rms_prom_buffer[ph], MAX_RMS_PROM), ph);

              rms_total[ph + TOTAL_PHASES] = calculate_mean(rms_prom_buffer[ph + TOTAL_PHASES], MAX_RMS_PROM);

              if(rms_total[ph + TOTAL_PHASES] < 0.2f){
                rms_total[ph + TOTAL_PHASES] = 0;
              }

              P_total[ph] = calculate_mean(P_prom_buffer[ph], MAX_RMS_PROM);
              Q_total[ph] = calculate_mean(Q_prom_buffer[ph], MAX_RMS_PROM);


              S[ph] = rms_total[ph] * rms_total[ph + TOTAL_PHASES];

              /* ----------------------------------------------------------------
               * FACTOR DE POTENCIA FP
               *
               * FP = P / S    (IEEE 1459-2010, rango [-1, +1])
               *   FP > 0 : flujo neto de energía hacia la carga (consumo)
               *   FP < 0 : flujo neto de energía hacia la red (generación)
               *
               * phi = atan2(Q, P)  (rango [-180°, +180°])
               *   Cuadrante I   (P>0, Q>0): consumo inductivo       [0°,  90°]
               *   Cuadrante II  (P<0, Q>0): generación inductiva    [90°, 180°]
               *   Cuadrante III (P<0, Q<0): generación capacitiva  [-180°,-90°]
               *   Cuadrante IV  (P>0, Q<0): consumo capacitivo     [-90°,  0°]
               * ---------------------------------------------------------------- */
              if (S[ph] < 0.001f) {
                /* Sin señal: evitar división por cero */
                FP[ph] = 0.0f;
                phi[ph] = 0.0f;
              } else {
                phi[ph] = acosf(P_total[ph] / S[ph]);

                FP[ph] = cosf(phi[ph]);
                
                // Fabri:
                // Banda muerta para Q cercano a cero (+/- 5VAr)
                if (Q_total[ph] <= 5.0f && Q_total[ph] >= -5.0f) {
                // No se tiene suficiente certeza sobre el signo de Q cuando está cerca de cero, tratar como resistivo puro
                  phi[ph] = 0.0f;
                }
                else if(Q_total[ph] < -5.0f){
                  phi[ph] = -phi[ph]; // Corrige el signo del ángulo para cargas capacitivas (Q<0)
                }
                
                // Fabri:
                // Banda muerta para P cercano a cero (+/- 5W)
                if (P_total[ph] <= 5.0f && P_total[ph] >= -5.0f) {
                  // No se tiene suficiente certeza sobre el signo de P cuando está cerca de cero, tratar como sin señal
                  FP[ph] = 0.0f;
                }
                else if(P_total[ph] < -5.0f){
                  // Corrige el signo del factor de potencia para generación (P<0)
                  FP[ph] = -FP[ph]; 
                }

                phi[ph] *= CONV_RAD_TO_DEG;
              }

                

                  
            }
            calculos_ready = 1; // listo para enviar por UART
            adc_calibrated = 0; // 0 para volver a calibrar Vdda
          }
        }
      }
    }

    // === 4. Acumulación de muestras SOLO dentro del período ===
    if (muestreo) {
      if (sample_index < MAX_SAMPLES) {
        for (uint8_t ch = 0; ch < TOTAL_PHASES; ch++) {
          //en el buffer "valid_channels" se tienen los canales de tension y corriente
          //adcData.channels[7] es el offset común para tensión y corriente, que se resta para centrar en cero
          sample_buffer[ch][sample_index] = (int16_t)adcData.channels[ch] - (int16_t)adcData.channels[7];
          sample_buffer[ch + TOTAL_PHASES][sample_index] = -((int16_t)adcData.channels[ch + TOTAL_PHASES + 1] - (int16_t)adcData.channels[7]);
        }

        // Cálculo de valores máximos y mínimos para cada fase (canales de corriente)
        if (sample_index == 0){
          // Primera muestra del período -> reset de picos de corriente
          for (uint8_t ph = 0; ph < TOTAL_PHASES; ph++){
            i_max[ph] = 0;
            i_min[ph] = 0;
          }
        }else{        
          // Actualiza máximos y mínimos de corriente durante el período
          for (uint8_t ph = 0; ph < TOTAL_PHASES; ph++){
            if(sample_buffer[ph+TOTAL_PHASES][sample_index] > i_max[ph]){
              i_max[ph] = sample_buffer[ph+TOTAL_PHASES][sample_index];
            }else if(sample_buffer[ph+TOTAL_PHASES][sample_index] < i_min[ph]){
              i_min[ph] = sample_buffer[ph+TOTAL_PHASES][sample_index];
            }
          }
        }
        sample_index++; // Avanza al siguiente sample del período
      } else {
        // seguridad: overflow del buffer
        muestreo = 0;
        sample_index = 0;
      }
    }
  
    // === 5. Envío por UART ===
    if (calculos_ready) {
      calculos_ready = 0;

      int len = snprintf(
        rms_tx_buf, sizeof(rms_tx_buf),
        "{\"version\":\"%s\","
        "\"rms\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
        "\"p\":[%.3f,%.3f,%.3f],"
        "\"q\":[%.3f,%.3f,%.3f],"
        "\"s\":[%.3f,%.3f,%.3f],"
        "\"fp\":[%.4f,%.4f,%.4f],"
        "\"phi\":[%.2f,%.2f,%.2f]}\r\n",
        FIRMWARE_VERSION,
        rms_total[0], rms_total[1], rms_total[2],
        rms_total[3], rms_total[4], rms_total[5],
        P_total[0],   P_total[1],   P_total[2],
        Q_total[0],   Q_total[1],   Q_total[2],
        S[0],         S[1],         S[2],
        FP[0],        FP[1],        FP[2],
        phi[0], phi[1], phi[2]
      );

      if (len > 0 && len < (int)sizeof(rms_tx_buf) && uartReady) {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)rms_tx_buf, (uint16_t)len);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        uartReady = 0;
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

  timer_cont++;
  flag_adc_ready = 1; /* Signal main loop that new ADC data is ready */
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
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
      sum += (float)buffer[i] * (float)buffer[i];
  }
  return (sqrtf(sum / (float)samples));
}

void vdda_calibrated(void) {
  ADC_MeasurementData_t adcData;
  __disable_irq();
  adcData = adcIncData;
  //flag_adc_ready = 0;
  __enable_irq();

  uint16_t adc_vrefint = adcData.channels[3];

  vdda = (1.21f * 4095.0f) / (float)adc_vrefint;

  adc_calibrated = 1;
}

static float adc_to_voltage(float adc_value, uint8_t phase)
{
  // ECUACION: V = adc_value * (vdda/4095) * (Vi/Vo)

  switch (phase) {
    case 0: return adc_value * vdda * V1_GAIN;
    case 1: return adc_value * vdda * V2_GAIN;
    case 2: return adc_value * vdda * V3_GAIN;
    default: return 0.0f;
  }
}

static float adc_to_current(float adc_value, float gain, uint8_t phase)
{
  // ECUACION: I = adc_value * (vdda/4095) * (Ii/Io) / gain_variable

  switch (phase) {
    case 0: return (adc_value * vdda / gain) * I1_GAIN;
    case 1: return (adc_value * vdda / gain) * I2_GAIN;
    case 2: return (adc_value * vdda / gain) * I3_GAIN;
    default: return 0.0f;
  }
}

/* -----------------------------------------------------------------------
 * adc2_to_power — convierte ADC² a Watts o VAr
 *
 * La calculate_single_bin_dft() y calculate_active_power() trabajan sobre muestras crudas ADC,
 * por lo que el resultado está en unidades ADC_V × ADC_I.
 *
 * La conversión correcta es:
 *   P_real [W] = P_adc2 × (vdda × Kv) × (vdda × Ki / G)
 *              = P_adc2 × vdda² × Kv × Ki / G
 *
 * Esto es equivalente a aplicar adc_to_voltage() y adc_to_current()
 * como factores de escala (ambas funciones son lineales sin offset),
 * por lo que se puede factorizar directamente.
 * ----------------------------------------------------------------------- */
static float adc2_to_power(float adc2_value, float gain, uint8_t phase) {
  /* Extraer solo el factor de escala de cada conversión:
   *   adc_to_voltage(x, ph)  = x * vdda * Kv   → factor_v = vdda * Kv
   *   adc_to_current(x, g, ph) = x * vdda * Ki / g → factor_i = vdda * Ki / g
   */
  float factor_v, factor_i;
  switch (phase) {
    case 0: factor_v = vdda * V1_GAIN; factor_i = vdda * I1_GAIN / gain; break;
    case 1: factor_v = vdda * V2_GAIN; factor_i = vdda * I2_GAIN / gain; break;
    case 2: factor_v = vdda * V3_GAIN; factor_i = vdda * I3_GAIN / gain; break;
    default: return 0.0f;
  }
  return adc2_value * factor_v * factor_i;
}

// calcula la ganancia real a partir de la posición del wiper
float calculate_gain(uint8_t wiper_position, uint8_t invertido){
  if(invertido){
    return (float) (128.0f - wiper_position)/wiper_position;
  }else{
    return (float) wiper_position/(128.0f - wiper_position);
  }
}

/* --- Potencia activa por producto directo (IEEE 1459-2010) --- */
static float calculate_active_power(int16_t *buffer_tension, int16_t *buffer_corriente, uint8_t samples) {
  if (samples == 0) return 0.0f;

  float acc = 0.0f;
  for (uint16_t n = 0; n < samples; n++) {
    acc += (float)buffer_tension[n] * (float)buffer_corriente[n];
  }
  return (acc / (float) samples);
}

static float calculate_mean(float *buffer, uint8_t samples) {
  if (samples == 0) return 0.0f;

  float acc = 0.0f;
  for (uint16_t i = 0; i < samples; i++) {
      acc += buffer[i];
  }

  return acc / (float)samples;
}

static void calculate_single_bin_dft(int16_t *buffer, uint8_t samples, float *real, float *imag) {
  if (samples == 0) {
    *real = 0.0f;
    *imag = 0.0f;
    return;
  }
  float sum_re = 0.0f;
  float sum_im = 0.0f;
  float omega = 2.0f * 3.1415926535f / (float)samples;
  for (uint16_t n = 0; n < samples; n++) {
    float angle = omega * (float)n;
    sum_re += (float)buffer[n] * cosf(angle);
    sum_im += (float)buffer[n] * sinf(angle);
  }
  *real = 2.0f * sum_re / (float)samples;
  *imag = -2.0f * sum_im / (float)samples;
}

/* -----------------------------------------------------------------------
 * AdjustCurrentGain_Wiper — AGC con histéresis y subida gradual [CAMBIO 6]
 *
 * Máquina de estados por fase:
 *
 *  ZONA ALTA  (i_max > I_MAX):
 *    → wiper-- inmediato (protección de saturación, máxima prioridad)
 *    → count_cambio_wiper = salto_niveles + 1  (tiempo de establecimiento)
 *    → reset contadores de señal baja
 *
 *  ZONA OK  (I_MIN ≤ i_max ≤ I_MAX):
 *    → sin cambio de wiper
 *    → reset de todos los contadores
 *
 *  ZONA BAJA con señal presente  (I_MIN_NOISE < i_max < I_MIN):
 *    → count_señalBaja++
 *    → si count_señalBaja ≥ PER_SUBIDA_GAIN → wiper++ (subida gradual)
 *    → evita subir por transitorios cortos de un período
 *
 *  ZONA RUIDO / sin señal  (i_max ≤ I_MIN_NOISE):
 *    → count_noSignal++
 *    → si count_noSignal ≥ PER_NO_SIGNAL → wiper = 0 (reset a G=7.5×)
 *    → reset count_señalBaja
 *
 *  HISTÉRESIS:
 *    Zona alta:  entra con I_MAX
 *    Zona baja:  entra con I_MIN
 *    Zona ruido: entra con I_MIN_NOISE (sin histéresis, reset inmediato)
 *
 *  count_cambio_wiper:
 *    Bajada de 1 nivel  → 2 períodos descartados (1 settle MCP + 1 OPAMP)
 *    Bajada de N niveles → N + 1 períodos descartados
 *    Subida de 1 nivel  → 2 períodos descartados
 * ----------------------------------------------------------------------- */
void AdjustCurrentGain_Wiper(void) {
  for (uint8_t phase = 0; phase < TOTAL_PHASES; phase++) {
    cambio_wiper[phase] = 0;

    int16_t ipeak = (i_max[phase] > -i_min[phase])
                    ? i_max[phase] : -i_min[phase];

    if (!rms_index) {
      ipeak = 0;
      count_cambio_wiper[phase] = 3;
    }

    /* ---- ZONA ALTA: saturación → bajar ganancia inmediatamente ---- */
    if (ipeak > I_MAX) {
    /* reset contadores de señal baja */
      count_noSignal[phase]   = 0;
      count_señalBaja[phase]  = 0;

      if (wiper[phase] > 0) {
        int8_t wiper_prev = wiper[phase];
        wiper[phase]--;
        cambio_wiper[phase] = 1;
        /* descarte proporcional al salto para garantizar establecimiento */
        count_cambio_wiper[phase] = (uint8_t)(wiper_prev - wiper[phase]) + 1;
      }
    }

    /* ---- ZONA OK con histéresis: i_max entre I_MIN e I_MAX ---- */
    else if (ipeak >= I_MIN) {
      count_noSignal[phase]   = 0;
      count_señalBaja[phase]  = 0;
      /* sin cambio de wiper */
    }

    /* ---- ZONA BAJA con señal presente: subir ganancia gradualmente ---- */
    else if (ipeak >= I_MIN_NOISE){
      /* ipeak ∈ (I_MIN_NOISE, I_MIN) — señal débil pero real */
      count_noSignal[phase] = 0;
      count_señalBaja[phase]++;

      if (count_señalBaja[phase] >= PER_SUBIDA_GAIN) {
        count_señalBaja[phase] = 0;

        if (wiper[phase] < TOTAL_GAIN_CURRENT - 1) {
          wiper[phase]++;
          cambio_wiper[phase]         = 1;
          count_cambio_wiper[phase]   = 2; /* 1 settle MCP + 1 OPAMP */
        }
        /* si ya está en máxima ganancia: no cambiar, no amplificar ruido */
      }
    }

    /* ---- ZONA RUIDO: señal inferior al umbral de ruido ---- */
    else {
      count_señalBaja[phase] = 0;
      count_noSignal[phase]++;

      if (count_noSignal[phase] >= PER_NO_SIGNAL) {
        count_noSignal[phase] = 0;

        if (wiper[phase] != 3) {
          /* reset a G=7.5× para no amplificar ruido y a su vez poder registrar una pequeña señal cuando se conecte algo */
          count_cambio_wiper[phase] = (uint8_t)(wiper[phase]) + 1;
          wiper[phase]    = 3;
          cambio_wiper[phase] = 1;
        }
      }
    }

    /* ---- Comunicación SPI con MCP4131 ---- */
    if (cambio_wiper[phase]) {
      switch (phase) {
        case 0:
          if (MCP4131_IsReady(&hpot3)) {
            MCP4131_WriteWiper_DMA(&hpot3, wiper_position[wiper[phase]]);
            gain_table[phase] = calculate_gain(wiper_position[wiper[phase]], 0);
          } break;
        case 1:
          if (MCP4131_IsReady(&hpot4)) {
            MCP4131_WriteWiper_DMA(&hpot4, wiper_position[wiper[phase]]);
            gain_table[phase] = calculate_gain(wiper_position[wiper[phase]], 0);
          } break;
        case 2:
          if (MCP4131_IsReady(&hpot5)) {
            MCP4131_WriteWiper_DMA(&hpot5, wiper_position[wiper[phase]]);
            gain_table[phase] = calculate_gain(wiper_position[wiper[phase]], 0);
          } break;
      }
    }
  }
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
