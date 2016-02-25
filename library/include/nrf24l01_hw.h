/****************************************************************************
* Title                 :   NRF Hardware Interface
* Filename              :   nrf_hw.h
* Author                :   RBL
* Origin Date           :   18/12/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  18/12/15           .1         RBL        Interface Created.
*
*****************************************************************************/
/**
 * @file nrf_hw.h
 * @brief Hardware interface to Nordic NRF24L01
 *
 * Interface for nRF HAL
 * @defgroup nordic_nrf nRF HAL
 * @{
 * @ingroup nordic_hal
 * Interface for Nordic Semiconductor's radio chip.
 *
 * @date 25 Aug 2015
 * @author Richard Lowe
 * @copyright GNU Public License
 *
 * @version .1 - Initial testing and verification
 *
 * @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    GPS Click
 *   SW:              ARM 4.5.2
 *
 */
#ifndef NRF_HW_H_
#define NRF_HW_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "nrf24l01_defs.h"
#include "nrf24l01_hal.h"


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
extern "C"{
#endif

/** @name  << SETUP FUNCTIONS >> */
//@{
/* Setup function prototypes */

int nrf_hw_init( nrf_operation_mode_t operational_mode );

/**
 * @brief Enable or disable interrupt for radio.
 *
 * Use this function to enable or disable
 * one of the interrupt sources for the radio.
 * This function only changes state for selected
 * int_type, the rest of the interrupt sources
 * are left unchanged.
 *
 * @param int_source Radio interrupt Source.
 * @param irq_state Enable or Disable.
*/
void nrf_set_irq_mode( nrf_irq_source_t int_source, bool irq_state );

/**
 * @brief Read current interrupt mode for selected interrupt source.
 *
 * Use this function to get the interrupt source's mode,
 * either enabled or disabled.
 *
 * @param int_source Interrupt source to get mode from
 *
 * @return Interrupt Mode
 * @retval FALSE Interrupt disabled
 * @retval TRUE Interrupt enabled
*/
bool nrf_get_irq_mode( uint8_t int_source );

/**
 * @brief Read then clears all interrupt flags.
 *
 * Use this function to get the interrupt flags and clear them in the same operation.
 * Reduced radio interface activity and speed optimized.
 *
 * @return  Interrupt_flags
 * @retval 0x10 Max Retransmit interrupt
 * @retval 0x20 TX Data sent interrupt
 * @retval 0x40 RX Data received interrupt
*/
uint8_t nrf_get_clear_irq_flags( void );

/**
 * @brief Clear one selected interrupt flag.
 *
 * Use this function to clear one @a spesific interrupt flag.
 * Other interrupt flags are left unchanged.
 *
 * @param int_source Interrupt source of which flag to clear
*/
void nrf_clear_irq_flag( nrf_irq_source_t int_source );

/**
 * @brief Read all interrupt flags.
 *
 * Use this function to get the interrupt flags. This function is similar
 * to nrf_get_clear_irq_flags with the exception that it does <I><B>NOT</B></I> clear
 * the irq_flags.
 *
 * @return Interrupt_flags
 * @retval 0x10 Max Retransmit interrupt
 * @retval 0x20 TX Data sent interrupt
 * @retval 0x40 RX Data received interrupt
*/
uint8_t nrf_get_irq_flags( void );



/**
 * @brief Enables the ACK payload feature
 */
void nrf_enable_ack_pl( void );

/**
 * @brief Disables the ACK payload feature
 */
void nrf_disable_ack_pl( void );

/**
 * @brief Enables the dynamic payload feature
 */
void nrf_enable_dynamic_pl( void );

/**
 * @brief Disables the dynamic payload feature
 */
void nrf_disable_dynamic_pl( void );

/**
 * @brief Sets the dynamic payload features for the RX pipes
 *
 * The input parameter contains is a byte where the bit values tells weather
 * the pipe uses the ack payload feature or not. For example if bit 0 is set
 * then Pipe 0 uses ack payload.
 *
 * @param setup - byte value with bit set for pipes that uses the dynamic
 * payload feature
 */
void nrf_setup_dyn_pl( uint8_t setup );

/**
 * @brief Enables the no-ack feature
 */
void nrf_enable_dynamic_ack( void );

/**
 * @brief Disables the no-ack feature
 */
void nrf_disable_dynamic_ack( void );


/**
 * @brief Set the CRC mode used by the radio.
 *
 * Use this function to set the CRC mode; CRC disabled, 1 or 2 bytes.
 *
 * @param crc_mode CRC mode to use
*/
void nrf_set_crc_mode( nrf_crc_mode_t crc_mode );

/**
 * @brief Get CRC mode.
 *
 * Use this function to check which CRC mode is used.
 *
 * @return CRC_mode
 * @retval 0x00 CRC_OFF
 * @retval 0x02 CRC_8BIT
 * @retval 0x03 CRC_16BIT
*/
uint8_t nrf_get_crc_mode( void );

/**
 * @brief Open radio pipe(s) and enable/ disable auto acknowledge.
 *
 * Use this function to open one or all pipes,
 * with or without auto acknowledge.
 *
 * @param pipe_num Radio pipe to open
 * @param auto_ack Auto_Ack ON/OFF
 * @see nrf_address
*/
void nrf_open_pipe( nrf_address_t pipe_num, bool auto_ack );

/**
 * @brief Close radio pipe(s).
 * Use this function to close one pipe or all pipes.
 *
 * @param pipe_num Pipe# number to close
*/
void nrf_close_pipe( nrf_address_t pipe_num );

/**
 * @brief Get pipe status.
 * Use this function to check status for a selected pipe.
 *
 * @param  pipe_num Pipe number to check status for
 *
 * @return Pipe_Status
 * @retval 0x00 Pipe is closed, autoack disabled
 * @retval 0x01 Pipe is open, autoack disabled
 * @retval 0x03 Pipe is open, autoack enabled
*/
uint8_t nrf_get_pipe_status( nrf_address_t pipe_num );

/**
 * @brief Set radio's RX address and TX address.
 * Use this function to set a RX address, or to set the TX address.
 * Beware of the difference for single and multibyte address registers.
 *
 * @param address Which address to set
 * @param *addr Buffer from which the address is stored in
*/
void nrf_set_address( nrf_address_t address, uint8_t *addr );

/**
 * @brief Get address for selected pipe.
 *
 * Use this function to get address for selected pipe.
 *
 * @param address Which address to get, Pipe- or TX-address
 * @param *addr buffer in which address bytes are written.
 * <BR><BR>For pipes containing only LSB byte of address, this byte is returned
 * in the<BR> *addr buffer.
 *
 * @return Address_Width in bytes
*/
uint8_t nrf_get_address( nrf_address_t address, uint8_t *addr );

/**
 * @brief Set radio's address width.
 *
 * Use this function to define the radio's address width,
 * referes to both RX and TX.
 *
 * @param address_width Address with in bytes
*/
void nrf_set_address_width( nrf_address_width_t address_width );

/**
 * @brief Get address width for radio.
 * Use this function to get the address width used for
 * the radio, both RX and TX.
 *
 * @return Address_Width in bytes
*/
uint8_t nrf_get_address_width( void );

/**
 * @brief Set auto acknowledge parameters.
 *
 * Use this function to set retransmit and retransmit delay
 * parameters.
 *
 * @param retr Number of retransmit, 0 equ retransmit OFF
 * @param delay Retransmit delay in µs
*/
void nrf_set_auto_retr( uint8_t retr, uint16_t delay );

/**
 * @brief Get auto retransmit parameters.
 *
 * Use this function to get the auto retransmit parameters,
 * retrans count and retrans delay.
 *
 * @return AutoRetrans Parameters
 *
 * @retval UpperNibble Retransmit Delay
 * @retval LowerNibble Retransmit Count
*/
uint8_t nrf_get_auto_retr_status( void );

/**
 * @brief Get radio's transmit attempts status.
 * Use this function to get number of retransmit
 * attempts and number of packet lost.
 *
 * @return Retransmit attempts counters
*/
uint8_t nrf_get_transmit_attempts( void );

/**
 * @brief Get packet lost counter
 * Use this function to get the packet(s) counter.
 *
 * @return packet lost counter
*/
uint8_t nrf_get_packet_lost_ctr( void );

/**
 * @brief Set payload width for selected pipe.
 * Use this function to set the number of bytes expected
 * on a selected pipe.
 *
 * @param pipe_num Pipe number to set payload width for
 * @param pload_width number of bytes expected
*/
void nrf_set_rx_pload_width( uint8_t pipe_num, uint8_t pload_width );

/**
 * @brief Get RX payload width for selected pipe.
 * Use this function to get the expected payload
 * width for selected ppe number.
 *
 * @param pipe_num Pipe number to get payload width for
 *
 * @return Payload_Width in bytes
*/
uint8_t nrf_get_rx_pload_width( uint8_t pipe_num );
//@}

/** @name  << OPERATION FUNCTIONS >> */
//@{
/* Operation function prototypes */

/**
 * @brief Set radio's operation mode.
 * Use this function to enter PTX (primary TX)
 * or PRX (primary RX).
 *
 * @param op_mode Operation mode
*/
void nrf_set_operation_mode( nrf_operation_mode_t op_mode );

/**
 * @brief Get radio's current operation mode.
 * Use this function to get the radio's current
 * operation mode, PTX or PRX.
 *
 * @return Operation_Mode
 * @retval 0x00 Primary RX (PRX)
 * @retval 0x01 Primary TX (PTX)
*/
uint8_t nrf_get_operation_mode( void );

/**
 * @brief nrf_start_active_rx
 */
void nrf_start_active_rx( void );

/**
 * @brief nrf_enter_standby_mode
 *
 * Standby-I is used to minimize average current consumption while maintaining
 * short start up times. In this mode part of the crystal oscillator is
 * active. This is the mode the nRF24L01 returns to from TX or RX mode when
 * CE is set low.
 */
void nrf_enter_standby_mode( void );

/**
 * @brief Set radio's power mode.
 * Use this function to power_up or power_down radio.
 *
 * @param pwr_mode POWER_UP or POWER_DOWN
*/
void nrf_set_power_mode( nrf_pwr_mode_t pwr_mode );

/**
 * @brief Get radio's current power mode.
 * Use this function to get the radio's currnet
 * power mode, POWER_UP or POWER_DOWN.
 *
 * @return Power_Mode
 * @retval 0x00 POWER_DOWN
 * @retval 0x01 POWER_UP
*/
uint8_t nrf_get_power_mode( void );

/**
 * @brief Set radio's RF channel.
 *
 * Use this function to select which RF channel to use.
 *
 * @param channel RF channel
*/
void nrf_set_rf_channel( uint8_t channel );

/**
 * @brief Get radio's current RF channel.
 *
 * Use this function to get the radio's current
 * selected RF channel
 *
 * @return RF channel
*/
uint8_t nrf_get_rf_channel();

/**
 * @brief Set radio's TX output power.
 * Use this function set the radio's TX output power.
 *
 * @param power Radio's TX output power
*/
void nrf_set_output_power( nrf_output_power_t power );

/**
 * @brief Get radio's current TX output power.
 *
 * Use this function to get the radio's current
 * TX output power setting.
 *
 * @return TX_power_output
 * @retval 0x00 -18dBm
 * @retval 0x01 -12dBm
 * @retval 0x02 -6dBm
 * @retval 0x03 0dBm
*/
uint8_t nrf_get_output_power( void );

/**
 * @brief Set radio's on-air datarate.
 * Use this function to select radio's on-air
 * datarate.
 *
 * @param datarate On-air datarate
*/
void nrf_set_datarate( nrf_datarate_t datarate );

/**
 * @brief Get radio's current on-air datarate.
 * Use this function to get the radio's current
 * on-air datarate setting.
 *
 * @return On-air datarate
 * @retval 0x00 1Mbps selected
 * @retval 0x01 2Mbps selected
*/
uint8_t nrf_get_datarate( void );

/**
 * @brief Delay function
 *
 * @param[in] uint16_t ms - time delay in milliseconds
 */
void nrf_delay( uint16_t ms );

/* Status functions prototypes */

/**
 * @brief Get radio's TX FIFO status.
 * Use this function to get the radio's TX
 * FIFO status.
 *
 * @return TX FIFO status
 * @retval 0x00 TX FIFO NOT empty, but NOT full
 * @retval 0x01 FIFO empty
 * @retval 0x02 FIFO full
 *
*/
uint8_t nrf_get_tx_fifo_status( void );

/**
 * @brief Check for TX FIFO empty.
 * Use this function to check if TX FIFO
 * is empty.
 *
 * @return TX FIFO empty bit
 * @retval FALSE TX FIFO NOT empty
 * @retval TRUE TX FIFO empty
 *
*/
bool nrf_tx_fifo_empty( void );

/**
 * @brief Check for TX FIFO full.
 * Use this function to check if TX FIFO
 * is full.
 *
 * @return TX FIFO full bit
 * @retval FALSE TX FIFO NOT full
 * @retval TRUE TX FIFO full
 *
*/
bool nrf_tx_fifo_full( void );

/**
 * @brief Get radio's RX FIFO status.
 * Use this function to get the radio's TX
 * FIFO status.
 *
 * @return RX FIFO status
 * @retval 0x00 RX FIFO NOT empty, but NOT full
 * @retval 0x01 RX FIFO empty
 * @retval 0x02 RX FIFO full
 *
*/
uint8_t nrf_get_rx_fifo_status( void );

/**
 * @brief Check for RX FIFO empty.
 * Use this function to check if RX FIFO
 * is empty.
 *
 * Reads STATUS register to check this, not FIFO_STATUS
 *
 * @return RX FIFO empty bit
 * @retval FALSE RX FIFO NOT empty
 * @retval TRUE RX FIFO empty
 *
*/
bool nrf_rx_fifo_empty( void );

/**
 * @brief Check for RX FIFO full.
 * Use this function to check if RX FIFO
 * is full.
 *
 * @return RX FIFO full bit
 * @retval FALSE RX FIFO NOT full
 * @retval TRUE RX FIFO full
 *
*/
bool nrf_rx_fifo_full( void );

/**
 * @brief Flush RX FIFO.
 * Use this function to flush the radio's
 * RX FIFO.
 *
*/
void nrf_flush_rx( void );

/**
 * @brief Flush TX FIFO.
 * Use this function to flush the radio's
 * TX FIFO.
 *
*/
void nrf_flush_tx( void );

/**
 * @brief Get the carrier detect flag.
 * Use this function to get the carrier detect flag,
 * used to detect stationary disturbance on selected
 * RF channel.
 *
 * @return Carrier Detect
 * @retval FALSE Carrier NOT Detected
 * @retval TRUE Carrier Detected
*/
bool nrf_get_carrier_detect( void );

/* Data operation prototypes */

/**
 * @brief Get RX data source.
 *
 * Use this function to read which RX pipe data
 * was received on for current top level FIFO data packet.
 *
 * @return pipe number of current packet present
*/
nrf_address_t nrf_get_rx_data_source( void );

/**
 * @brief Read RX payload.
 *
 * Use this function to read top level payload
 * available in the RX FIFO.
 *
 * @param  *rx_pload pointer to buffer in which RX payload are stored
 * @return pipe number (MSB byte) and packet length (LSB byte)
*/
uint16_t nrf_read_rx_pload( uint8_t *rx_pload );

/**
 * @brief Write TX payload to radio.
 *
 * Use this function to write a packet of
 * TX payload into the radio.
 * <I>length</I> number of bytes, which are stored in <I>*tx_pload</I>.
 *
 * @param *tx_pload pointer to buffer in which TX payload are present
 * @param length number of bytes to write
*/
void nrf_write_tx_pload( uint8_t *tx_pload, uint8_t length );

/**
 * @brief Reuse TX payload.
 * Use this function to set that the radio is using
 * the last transmitted payload for the next packet as well.
 *
*/
void nrf_reuse_tx( void );

/**
 * @brief Get status of reuse TX function.
 * Use this function to check if reuse TX payload is
 * activated
 *
 * @return Reuse TX payload mode
 * @retval FALSE Not activated
 * @retval TRUE Activated
*/
bool nrf_get_reuse_tx_status( void );

//@}

/** @name  << TEST FUNCTIONS >> */
//@{
/* Test functions prototypes */

/**
 * @brief Set radio's PLL mode.
 * Use this function to either LOCK
 * or UNLOCK the radio's PLL.
 *
 * @param pll_mode PLL locked, TRUE or FALSE
*/
void nrf_set_pll_mode( nrf_pll_mode_t pll_mode );

/**
 * @brief Get PLL mode.
 * Use this function to get the radio's
 * current PLL mode.
 *
 * @return PLL_mode
*/
nrf_pll_mode_t nrf_get_pll_mode( void );

/**
 * @brief Set radio's LNA gain mode.
 * Use this function to either use HI
 * current or LOW current mode for the radio.
 *
 * @param lna_gain LNA gain mode
*/
void nrf_set_lna_gain( nrf_lna_mode_t lna_gain );

/**
 * @brief Get LNA gain mode.
 * Use this function to get the radio's
 * current LNA gain mode.
 *
 * @return LNA gain mode
 * @retval 0 LNA LOW current
 * @retval 1 LNA HI current
*/
nrf_lna_mode_t nrf_get_lna_gain( void );

/**
@} */

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/
