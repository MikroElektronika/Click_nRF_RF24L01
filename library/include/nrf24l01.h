/****************************************************************************
* Title                 :   NRF API Interface
* Filename              :   nrf.h
* Author                :   RBL
* Origin Date           :   12/01/2016
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description 
*  12/01/16         .1           RBL      Interface Created.
*
*****************************************************************************/
/**
 * @file nrf.h
 * @brief API interface to NRF24L01 radio.  These higher level functions
 * provide an easy interface to generic functions and operation.
 *
 * @date 12 Jan 2016
 * @author Richard Lowe
 * @copyright GNU Public License
 *
 * @version .1 - Initial testing and verification
 *
 * @note Test configuration:
 *   MCU:             STM32F407VG
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      150 Mhz internal
 *   Ext. Modules:    nRF C Click
 *   SW:              ARM 4.7
 *
 */

#ifndef NRF_H_
#define NRF_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "nrf24l01_hw.h"
#include "nrf24l01_defs.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/
/** Defines how many retransmitts that should be performed */
#define NRF_RETRANSMITS 15

/** Defines the payload length the radio should use */
#define NRF_PAYLOAD_LENGTH 4

/** Defines the retransmit delay. Should be a multiple of 250. If the
 * RF_PAYLOAD_LENGTH is larger than 18, a higher retransmitt delay need to
 * be set. This is because both the original package and ACK payload will
 * be of this size. When the ACK payload exeedes 18 byte, it will not be able
 * to recieve the full ACK in the ordinary 250 mikroseconds, so the delay
 * will need to be increased. */
#if (NRF_PAYLOAD_LENGTH <= 18)
#define NRF_RETRANS_DELAY 250
#else
#define NRF_RETRANS_DELAY 500
#endif

/** Defines the channel the radio should operate on*/
#define NRF_CHANNEL 40

/** Defines the time it takes for the radio to come up to operational mode */
#define NRF_POWER_UP_DELAY 2


/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
typedef enum {
    RF_IDLE,
    RF_MAX_RT,
    RF_TX_DS,
    RF_RX_DR,
    RF_TX_AP,
    RF_BUSY
} nrf24l01_status_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief Enables radio in shock burst mode with default settings
 *
 * @param address - address of primary Pipe 0
 * @param operational_mode - Enables either PRX or PTX modes
 */
int nrf_sb_init( nrf_addr_map_t *address, nrf_operation_mode_t operational_mode );

/**
 * @brief nrf_esb_init
 *
 * @param address
 * @param operational_mode
 */
int nrf_esb_init( nrf_addr_map_t *address, nrf_operation_mode_t operational_mode );

/**
 * @brief nrf_pl_init
 *
 * @param address
 * @param operational_mode
 */
int nrf_esb_bidirection_init( nrf_addr_map_t *address, nrf_operation_mode_t operational_mode );

/**
 * @brief nrf_send_data
 *
 * @param address
 * @param data_out
 * @param count
 * @return
 */
uint8_t nrf_send_data( uint8_t *address, uint8_t *data_out , uint8_t count);

/**
 * @brief nrf_recieve_data
 *
 * @param address
 * @param data_in
 * @param count
 * @return
 */
uint8_t nrf_recieve_data( nrf_address_t *address, uint8_t *data_in );

/**
 *
 */
void nrf_acknowledged( void );

/**
 * @brief nrf_is_interrupted
 *
 * @param source
 * @return
 */
bool nrf_is_interrupted( nrf_irq_source_t source );

/**
 *
 */
void nrf_timer_tick( void );

//nrf24l01_status_t nrf24l01_get_status (void);

//void nrf24l01_set_status (nrf24l01_status_t new_status);

//uint8_t nrf24l01_get_pload_byte (uint8_t byte_index);

//void nrf24l01_send_packet(uint8_t *packet, uint8_t length);


#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/
