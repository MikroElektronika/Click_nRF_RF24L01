/****************************************************************************
* Title                 :   NRF Common Functions
* Filename              :   nrf_common.h
* Author                :   RBL
* Origin Date           :   24/12/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  25/12/15    .1                RBL      Interface Created.
*
*****************************************************************************/
/** 
 * @file nrf_common.h
 * @brief What is does
 *
 * @date 25 Dec 2015
 * @author Richard Lowe
 * @copyright GNU Public License
 *
 * @version .1 - Initial testing and verification
 *
 * @note Test configuration:
 *   MCU:             STM32F407ZG
 *   Dev.Board:       MikroMedia STM 7
 *   Oscillator:      150 Mhz internal
 *   Ext. Modules:
 *   SW:              ARM 4.5.2
 *
 */
#ifndef NRF_COMMON_H__
#define NRF_COMMON_H__

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
/** Swaps the upper byte with the lower byte in a 16 bit variable */
#define SWAP(x) ((((x)&0xFF)<<8)|(((x)>>8)&0xFF))

/** The upper 8 bits of a 16 bit value */
#define MSB(a) ((a & 0xFF00) >> 8)
/** The lower 8 bits (of a 16 bit value) */
#define LSB(a) ((a & 0xFF))

/** Leaves the minimum of the two arguments */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/** Leaves the maximum of the two arguments */
#define MAX(a, b) ((a) < (b) ? (b) : (a))

#define BIT_0 0x01 /**< The value of bit 0 */
#define BIT_1 0x02 /**< The value of bit 1 */
#define BIT_2 0x04 /**< The value of bit 2 */
#define BIT_3 0x08 /**< The value of bit 3 */
#define BIT_4 0x10 /**< The value of bit 4 */
#define BIT_5 0x20 /**< The value of bit 5 */
#define BIT_6 0x40 /**< The value of bit 6 */
#define BIT_7 0x80 /**< The value of bit 7 */

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
extern "C"{
#endif



#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/