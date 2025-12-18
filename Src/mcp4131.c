/**
 * @file    mcp4131.c
 * @brief   MCP4131 7-bit Digital Potentiometer Driver for STM32 HAL
 * @details Implementation of SPI communication with MCP4131
 *          Supports both blocking and DMA transmission modes
 *
 * MCP4131 SPI Protocol:
 * - Command Byte: [AD3][AD2][AD1][AD0][C1][C0][D9][D8]
 *   - AD3:AD0 = Memory address (0x00 for Wiper 0)
 *   - C1:C0   = Command bits (00=Write, 11=Read)
 *   - D9:D8   = Data bits 9:8 (for 10-bit devices, ignored for 7-bit)
 * - Data Byte: [D7][D6][D5][D4][D3][D2][D1][D0]
 *
 * For Write: Send command byte with data, then data byte
 * For Read:  Send command byte, receive data in response
 */

#include "mcp4131.h"

/* Private defines -----------------------------------------------------------*/
#define MCP4131_SPI_TIMEOUT    100U  /* SPI timeout in ms */

/* Private macros ------------------------------------------------------------*/

/**
 * @brief Build command byte for MCP4131
 * @param addr Memory address (4 bits)
 * @param cmd  Command (2 bits)
 * @param data Upper 2 bits of data (for 10-bit devices)
 */
#define MCP4131_BUILD_CMD(addr, cmd, data) \
    (uint8_t)(((addr) << 4) | ((cmd) << 2) | ((data) & 0x03))

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Assert chip select (active low)
 * @param  hmcp Pointer to MCP4131 handle
 */
static inline void MCP4131_CS_Low(MCP4131_HandleTypeDef *hmcp)
{
    HAL_GPIO_WritePin(hmcp->cs_port, hmcp->cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief  Deassert chip select (active low)
 * @param  hmcp Pointer to MCP4131 handle
 */
static inline void MCP4131_CS_High(MCP4131_HandleTypeDef *hmcp)
{
    HAL_GPIO_WritePin(hmcp->cs_port, hmcp->cs_pin, GPIO_PIN_SET);
}

/* Exported functions --------------------------------------------------------*/

MCP4131_StatusTypeDef MCP4131_Init(MCP4131_HandleTypeDef *hmcp,
                                    SPI_HandleTypeDef *hspi,
                                    GPIO_TypeDef *cs_port,
                                    uint16_t cs_pin)
{
    if (hmcp == NULL || hspi == NULL || cs_port == NULL) {
        return MCP4131_ERROR;
    }

    hmcp->hspi    = hspi;
    hmcp->cs_port = cs_port;
    hmcp->cs_pin  = cs_pin;
    hmcp->busy    = 0;

    /* Ensure CS is high (deselected) initially */
    MCP4131_CS_High(hmcp);

    return MCP4131_OK;
}

MCP4131_StatusTypeDef MCP4131_WriteWiper_DMA(MCP4131_HandleTypeDef *hmcp, uint8_t value)
{
    HAL_StatusTypeDef hal_status;

    if (hmcp == NULL) {
        return MCP4131_ERROR;
    }

    /* Check if previous transfer is still in progress */
    if (hmcp->busy) {
        return MCP4131_BUSY;
    }

    /* Clamp value to valid range (0-128 for 7-bit pot) */
    if (value > MCP4131_WIPER_MAX) {
        value = MCP4131_WIPER_MAX;
    }

    /* Build command: Address=0x00 (Wiper0), Command=0x00 (Write), Data[9:8]=0 */
    hmcp->tx_buf[0] = MCP4131_BUILD_CMD(MCP4131_ADDR_WIPER0, MCP4131_CMD_WRITE, 0);
    hmcp->tx_buf[1] = value;

    /* Mark as busy before starting transfer */
    hmcp->busy = 1;

    /* Assert CS and start DMA transfer */
    MCP4131_CS_Low(hmcp);
    hal_status = HAL_SPI_Transmit_DMA(hmcp->hspi, hmcp->tx_buf, 2);

    if (hal_status != HAL_OK) {
        /* Transfer failed to start, release CS and clear busy flag */
        MCP4131_CS_High(hmcp);
        hmcp->busy = 0;
        return MCP4131_ERROR;
    }

    return MCP4131_OK;
}

MCP4131_StatusTypeDef MCP4131_WriteWiper(MCP4131_HandleTypeDef *hmcp, uint8_t value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buf[2];

    if (hmcp == NULL) {
        return MCP4131_ERROR;
    }

    /* Clamp value to valid range (0-128 for 7-bit pot) */
    if (value > MCP4131_WIPER_MAX) {
        value = MCP4131_WIPER_MAX;
    }

    /* Build command: Address=0x00 (Wiper0), Command=0x00 (Write), Data[9:8]=0 */
    tx_buf[0] = MCP4131_BUILD_CMD(MCP4131_ADDR_WIPER0, MCP4131_CMD_WRITE, 0);
    tx_buf[1] = value;

    /* Perform SPI transaction */
    MCP4131_CS_Low(hmcp);
    hal_status = HAL_SPI_Transmit(hmcp->hspi, tx_buf, 2, MCP4131_SPI_TIMEOUT);
    MCP4131_CS_High(hmcp);

    if (hal_status != HAL_OK) {
        if (hal_status == HAL_TIMEOUT) {
            return MCP4131_TIMEOUT;
        }
        return MCP4131_ERROR;
    }

    return MCP4131_OK;
}

MCP4131_StatusTypeDef MCP4131_ReadWiper(MCP4131_HandleTypeDef *hmcp, uint16_t *value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    if (hmcp == NULL || value == NULL) {
        return MCP4131_ERROR;
    }

    /* Build command: Address=0x00 (Wiper0), Command=0x03 (Read), Data[9:8]=0x03 (dummy 1s) */
    tx_buf[0] = MCP4131_BUILD_CMD(MCP4131_ADDR_WIPER0, MCP4131_CMD_READ, 0x03);
    tx_buf[1] = 0xFF;  /* Dummy byte to clock out response */

    /* Perform SPI transaction */
    MCP4131_CS_Low(hmcp);
    hal_status = HAL_SPI_TransmitReceive(hmcp->hspi, tx_buf, rx_buf, 2, MCP4131_SPI_TIMEOUT);
    MCP4131_CS_High(hmcp);

    if (hal_status != HAL_OK) {
        if (hal_status == HAL_TIMEOUT) {
            return MCP4131_TIMEOUT;
        }
        return MCP4131_ERROR;
    }

    /* 
     * Response format for MCP4131 (7-bit):
     * - First byte: [X][X][X][X][X][X][CMDERR][D8]
     * - Second byte: [D7][D6][D5][D4][D3][D2][D1][D0]
     * For 7-bit device, D8 is always 0, value is in D7:D0
     */
    *value = (uint16_t)(((rx_buf[0] & 0x01) << 8) | rx_buf[1]);

    return MCP4131_OK;
}

bool MCP4131_IsReady(MCP4131_HandleTypeDef *hmcp)
{
    if (hmcp == NULL) {
        return false;
    }
    return (hmcp->busy == 0);
}

void MCP4131_TxCpltCallback(MCP4131_HandleTypeDef *hmcp)
{
    if (hmcp == NULL) {
        return;
    }
    
    /* Deassert CS after DMA transfer completes */
    MCP4131_CS_High(hmcp);
    
    /* Clear busy flag */
    hmcp->busy = 0;
}

MCP4131_StatusTypeDef MCP4131_SetMin(MCP4131_HandleTypeDef *hmcp)
{
    return MCP4131_WriteWiper_DMA(hmcp, MCP4131_WIPER_MIN);
}

MCP4131_StatusTypeDef MCP4131_SetMax(MCP4131_HandleTypeDef *hmcp)
{
    return MCP4131_WriteWiper_DMA(hmcp, MCP4131_WIPER_MAX);
}

MCP4131_StatusTypeDef MCP4131_SetMid(MCP4131_HandleTypeDef *hmcp)
{
    return MCP4131_WriteWiper_DMA(hmcp, MCP4131_WIPER_MID);
}
