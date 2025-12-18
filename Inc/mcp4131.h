/**
 * @file    mcp4131.h
 * @brief   MCP4131 7-bit Digital Potentiometer Driver for STM32 HAL
 * @details Single-channel SPI digital potentiometer with 129 tap positions (0-128)
 *          Based on Microchip MCP4131 datasheet and Arduino library patterns
 *          Supports DMA transmission for non-blocking operation
 */

#ifndef MCP4131_H
#define MCP4131_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/** @defgroup MCP4131_Memory_Addresses MCP4131 Memory Map Addresses
 * @{
 */
#define MCP4131_ADDR_WIPER0         0x00  /**< Volatile Wiper 0 */
#define MCP4131_ADDR_WIPER1         0x01  /**< Volatile Wiper 1 (MCP4132 only) */
#define MCP4131_ADDR_NV_WIPER0      0x02  /**< Non-Volatile Wiper 0 */
#define MCP4131_ADDR_NV_WIPER1      0x03  /**< Non-Volatile Wiper 1 */
#define MCP4131_ADDR_TCON           0x04  /**< Volatile TCON Register */
#define MCP4131_ADDR_STATUS         0x05  /**< Status Register */
/** @} */

/** @defgroup MCP4131_Commands MCP4131 Command Bits
 * @{
 */
#define MCP4131_CMD_WRITE           0x00  /**< Write Data command (00) */
#define MCP4131_CMD_READ            0x03  /**< Read Data command (11) */
/** @} */

/** @defgroup MCP4131_Wiper_Limits MCP4131 Wiper Value Limits
 * @{
 */
#define MCP4131_WIPER_MIN           0     /**< Minimum wiper position (Wiper = B) */
#define MCP4131_WIPER_MAX           128   /**< Maximum wiper position (Wiper = A) */
#define MCP4131_WIPER_MID           64    /**< Middle wiper position */
/** @} */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief MCP4131 Handle Structure
 */
typedef struct {
    SPI_HandleTypeDef *hspi;      /**< Pointer to SPI handle */
    GPIO_TypeDef      *cs_port;   /**< Chip Select GPIO Port */
    uint16_t           cs_pin;    /**< Chip Select GPIO Pin */
    uint8_t            tx_buf[2]; /**< DMA transmit buffer (must persist during transfer) */
    volatile uint8_t   busy;      /**< Transfer in progress flag */
} MCP4131_HandleTypeDef;

/**
 * @brief MCP4131 Status enumeration
 */
typedef enum {
    MCP4131_OK       = 0x00,  /**< Operation successful */
    MCP4131_ERROR    = 0x01,  /**< Generic error */
    MCP4131_BUSY     = 0x02,  /**< SPI busy */
    MCP4131_TIMEOUT  = 0x03   /**< Operation timeout */
} MCP4131_StatusTypeDef;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize MCP4131 device handle
 * @param  hmcp Pointer to MCP4131 handle
 * @param  hspi Pointer to SPI handle
 * @param  cs_port GPIO port for chip select
 * @param  cs_pin GPIO pin for chip select
 * @retval MCP4131_StatusTypeDef
 */
MCP4131_StatusTypeDef MCP4131_Init(MCP4131_HandleTypeDef *hmcp, 
                                    SPI_HandleTypeDef *hspi,
                                    GPIO_TypeDef *cs_port, 
                                    uint16_t cs_pin);

/**
 * @brief  Write wiper value to MCP4131 using DMA (non-blocking)
 * @param  hmcp Pointer to MCP4131 handle
 * @param  value Wiper position (0-128)
 * @retval MCP4131_StatusTypeDef
 */
MCP4131_StatusTypeDef MCP4131_WriteWiper_DMA(MCP4131_HandleTypeDef *hmcp, uint8_t value);

/**
 * @brief  Write wiper value to MCP4131 (blocking)
 * @param  hmcp Pointer to MCP4131 handle
 * @param  value Wiper position (0-128)
 * @retval MCP4131_StatusTypeDef
 */
MCP4131_StatusTypeDef MCP4131_WriteWiper(MCP4131_HandleTypeDef *hmcp, uint8_t value);

/**
 * @brief  Read current wiper value from MCP4131 (blocking)
 * @param  hmcp Pointer to MCP4131 handle
 * @param  value Pointer to store read wiper position
 * @retval MCP4131_StatusTypeDef
 */
MCP4131_StatusTypeDef MCP4131_ReadWiper(MCP4131_HandleTypeDef *hmcp, uint16_t *value);

/**
 * @brief  Check if MCP4131 DMA transfer is complete
 * @param  hmcp Pointer to MCP4131 handle
 * @retval true if ready for new transfer, false if busy
 */
bool MCP4131_IsReady(MCP4131_HandleTypeDef *hmcp);

/**
 * @brief  SPI TX Complete callback handler - call from HAL_SPI_TxCpltCallback
 * @param  hmcp Pointer to MCP4131 handle
 */
void MCP4131_TxCpltCallback(MCP4131_HandleTypeDef *hmcp);

/**
 * @brief  Set wiper to minimum position (0)
 * @param  hmcp Pointer to MCP4131 handle
 * @retval MCP4131_StatusTypeDef
 */
MCP4131_StatusTypeDef MCP4131_SetMin(MCP4131_HandleTypeDef *hmcp);

/**
 * @brief  Set wiper to maximum position (128)
 * @param  hmcp Pointer to MCP4131 handle
 * @retval MCP4131_StatusTypeDef
 */
MCP4131_StatusTypeDef MCP4131_SetMax(MCP4131_HandleTypeDef *hmcp);

/**
 * @brief  Set wiper to middle position (64)
 * @param  hmcp Pointer to MCP4131 handle
 * @retval MCP4131_StatusTypeDef
 */
MCP4131_StatusTypeDef MCP4131_SetMid(MCP4131_HandleTypeDef *hmcp);

#ifdef __cplusplus
}
#endif

#endif /* MCP4131_H */
