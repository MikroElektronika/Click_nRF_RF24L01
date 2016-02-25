/****************************************************************************
* Title                 :   NRF24L01 Hardware Access Layer
* Filename              :   nrf24l01_hal.h
* Author                :   RBL
* Origin Date           :   06/01/2016
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  06/01/16         .1            RBL      Interface Created.
*
*****************************************************************************/
/** 
 * @file nrf24l01_hal.h
 * @brief Hardware Access Layer for the 24l01 radio.
 *
 * @date 8 Jan 2016
 * @author Richard Lowe
 * @copyright GNU Public License
 *
 * @version .1 - Initial testing and verification
 *
 * @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    NRF C click
 *   SW:              ARM 4.7
 *
 */
#ifndef NRF_HAL_H_
#define NRF_HAL_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include "nrf24l01_defs.h"

#if defined( __GNUC__ )
	#if defined( STM32F042x6 )
		#include "stm32f0xx_hal.h"
	#endif
#endif

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


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the HAL layer
 *
 * @param PTX or PRX mode
 *
 * @return int result of init
 * @retval -1 failed
 * @retval 0 successful
 */
int nrf_hal_init( nrf_operation_mode_t mode );

/**
 * @brief nrf_hal_listen
 */
void nrf_hal_listen( void );

/**
 * @brief nrf_hal_ignore
 */
void nrf_hal_ignore( void );

/**
 * @brief nrf_hal_write_reg
 *
 * @param address
 * @param value
 * @return
 */
uint8_t nrf_hal_write_reg( uint8_t address, uint8_t value );

/**
 * @brief Basis function read_reg.
 *
 * Use this function to read the contents
 * of one radios register.
 *
 * @param reg Register to read
 * @return Register contents
*/
uint8_t nrf_hal_read_reg( uint8_t address );

/**
 * @brief nrf_hal_write
 *
 * @param address
 * @param data_in
 * @param count
 * @return
 */
int nrf_hal_write( uint8_t address, uint8_t *data_in, uint8_t count );

/**
 * @brief nrf_hal_read
 *
 * @param address
 * @param data_out
 * @param count
 * @return
 */
int nrf_hal_read( uint8_t address, uint8_t *data_out, uint8_t count );

/**
 * @brief nrf_hal_delay
 *
 * @param ms
 */
void nrf_hal_delay( uint16_t ms );

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/
