/****************************************************************************
* Title                 :   ADC Application
* Filename              :   adc_app.h
* Author                :   JWB
* Origin Date           :   06/07/2012
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  08/17/13    XXXXXXXXXXX         JWB      Interface Created.
*
*****************************************************************************/
/** @file file_here.h
 *  @brief What is does
 *
 *  @date 25 Aug 2015
 *  @author Richard Lowe
 *  @copyright GNU Public License
 *
 *  @version .1 - Initial testing and verification
 *
 *  @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    GPS Click
 *   SW:              ARM 4.5.2
 *
 */

#ifndef NRF_DEFS_H_
#define NRF_DEFS_H_

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/



/******************************************************************************
* Typedefs
*******************************************************************************/
/**
 * @brief An enum describing the radio's irq sources.
 *
 */
typedef enum
{
    NRF_MAX_RT = 4,     /**< Max retries interrupt */
    NRF_TX_DS,          /**< TX data sent interrupt */
    NRF_RX_DR           /**< RX data received interrupt */
} nrf_irq_source_t;

/* Operation mode definitions */
/**
 * @brief An enum describing the radio's power mode.
 *
 */
typedef enum
{
    NRF_PTX,            /**< Primary TX operation */
    NRF_PRX             /**< Primary RX operation */
} nrf_operation_mode_t;

/**
 * @brief  An enum describing the radio's power mode.
 *
 */
typedef enum
{
    NRF_PWR_DOWN,       /**< Device power-down */
    NRF_PWR_UP          /**< Device power-up */
} nrf_pwr_mode_t;

/**
 * @brief An enum describing the radio's output power mode's.
 *
 */
typedef enum
{
    NRF_18DBM,          /**< Output power set to -18dBm */
    NRF_12DBM,          /**< Output power set to -12dBm */
    NRF_6DBM,           /**< Output power set to -6dBm  */
    NRF_0DBM            /**< Output power set to 0dBm   */
} nrf_output_power_t;

/**
 * @brief An enum describing the radio's on-air datarate.
 *
 */
typedef enum
{
    NRF_1MBPS,          /**< Datarate set to 1 Mbps  */
    NRF_2MBPS           /**< Datarate set to 2 Mbps  */
} nrf_datarate_t;

/**
 * @brief An enum describing the radio's PLL mode.
 *
 */
typedef enum
{
    NRF_PLL_UNLOCK,     /**< PLL unlocked, normal operation  */
    NRF_PLL_LOCK        /**< PLL locked, test mode  */
} nrf_pll_mode_t;

/**
 * @brief An enum describing the radio's LNA mode.
 *
 */
typedef enum
{
    NRF_LNA_LCURR,      /**< LNA set to low current mode */
    NRF_LNA_HCURR       /**< LNA set to high current mode */
} nrf_lna_mode_t;

/**
 * @brief An enum describing the radio's CRC mode.
 *
 */
typedef enum
{
    NRF_CRC_OFF,        /**< CRC check disabled */
    NRF_CRC_8BIT = 2,   /**< CRC check set to 8-bit */
    NRF_CRC_16BIT       /**< CRC check set to 16-bit */
} nrf_crc_mode_t;

/**
 * @brief An enum describing the read/write payload command.
 *
 */
typedef enum
{
    NRF_TX_PLOAD = 7,   /**< TX payload definition */
    NRF_RX_PLOAD,       /**< RX payload definition */
    NRF_ACK_PLOAD
} nrf_pload_command_t;

/**
 * @brief Structure containing the radio's address map.
 * Pipe0 contains 5 unique address bytes,
 * while pipe[1..5] share the 4 MSB bytes, set in pipe1.
 * <p><b> - Remember that the LSB byte for all pipes have to be unique! -</b>
 */
// nRF24L01 Address struct
typedef struct
{
    uint8_t p0[5];     /**< Pipe0 address, 5 bytes */
    uint8_t p1[5];     /**< Pipe1 address, 5 bytes, 4 MSB bytes shared for pipe1 to pipe5 */
    uint8_t p2[1];     /**< Pipe2 address, 1 byte */
    uint8_t p3[1];     /**< Pipe3 address, 1 byte */
    uint8_t p4[1];     /**< Pipe3 address, 1 byte */
    uint8_t p5[1];     /**< Pipe3 address, 1 byte */
    uint8_t tx[5];     /**< TX address, 5 byte */
} nrf_addr_map_t;


/**
 * @brief An enum describing the nRF24L01 pipe addresses and TX address.
 *
 */
typedef enum
{
    NRF_PIPE0,              /**< Select pipe0 */
    NRF_PIPE1,              /**< Select pipe1 */
    NRF_PIPE2,              /**< Select pipe2 */
    NRF_PIPE3,              /**< Select pipe3 */
    NRF_PIPE4,              /**< Select pipe4 */
    NRF_PIPE5,              /**< Select pipe5 */
    NRF_TX,                 /**< Refer to TX address*/
    NRF_ALL = 0xFF          /**< Close or open all pipes*/
                  /**< @see nrf_set_address @see nrf_get_address
                   @see nrf_open_pipe  @see nrf_close_pipe */
} nrf_address_t;

/**
 * @brief An enum describing the radio's address width.
 *
 */
typedef enum
{
    NRF_AW_3BYTES = 3,      /**< Set address width to 3 bytes */
    NRF_AW_4BYTES,          /**< Set address width to 4 bytes */
    NRF_AW_5BYTES           /**< Set address width to 5 bytes */
} nrf_address_width_t;

/**
 * @brief Enumerates the different states the radio may
 * be in.
 */
//typedef enum
//{
//  NRF_STANDBY,             /**< Radio is idle */
//  NRF_POWERDOWN,
//  NRF_MAX_RETRIES,      /**< Maximum number of retries have occured */
//  NRF_TX_DATA_SENT,     /**< Data is sent */
//  NRF_RX_DATA_RECEIVED, /**< Data recieved */
//  NRF_TX_ACK_PAYLOAD,   /**< Ack payload recieved */
//  NRF_BUSY              /**< Radio is busy */
//} nrf_state_t;


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif



#ifdef __cplusplus
} // extern "C"
#endif

#endif // NRF_REG_H__
/*** End of File **************************************************************/
