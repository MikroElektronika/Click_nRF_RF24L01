/*******************************************************************************
* Title                 :   NRF24L01 Hardware Layer
* Filename              :   nrf_l01_hw.c
* Author                :   RBL
* Origin Date           :   21/12/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  29/12/15         .1              RBL     Module Created.
*  29/01/16         .2              RBL     Fixed multi-reg write
*
*******************************************************************************/
/**
 * @file nrf_l01_hw.c
 * @brief Hardware layer for accessing NRF24L01+ radio
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "nrf_common.h"
#include "nrf24l01_hw.h"
#include "nrf24l01_hal.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#define SET_BIT(pos) ((uint8_t) (1<<( (uint8_t) (pos) )))
#define UINT8(t) ((uint8_t) (t))

/******************************************************************************
* Module Typedefs
*******************************************************************************/


/******************************************************************************
* Module Variable Definitions
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
/**
 * @brief Basis function write_reg.
 *
 * Use this function to write a new value to
 * a radio register.
 *
 * @param reg Register to write
 * @param value New value to write
 * @return Status register
*/
static uint8_t nrf_write_reg( uint8_t reg, uint8_t value );

/**
 * @brief Basis function, read_multibyte register
 *
 * Use this function to read multiple bytes from
 * a multibyte radio-register
 *
 * @param reg Multibyte register to read from
 * @param *pbuf Pointer to buffer in which to store read bytes to
 *
 * @return pipe# of received data (MSB), if operation used by a nrf_read_rx_pload
 * @return length of read data (LSB), either for nrf_read_rx_pload or
 * for nrf_get_address.
*/
static uint16_t nrf_read_multibyte_reg( uint8_t reg, uint8_t *pbuf );

/**
 * @brief Basis function, write_multibyte register.
 *
 * Use this function to write multiple bytes to
 * a multiple radio register.
 *
 * @param reg Register to write
 * @param *pbuf pointer to buffer in which data to write is
 * @param length \# of bytes to write
*/
static void nrf_write_multibyte_reg( uint8_t reg, uint8_t *pbuf,
                                     uint8_t length );

/**
 * @brief Write ack payload
 *
 * Writes the payload that will be transmitted with the ack on the given pipe.
 *
 * @param pipe Pipe that transmits the payload
 * @param tx_pload Pointer to the payload data
 * @param length Size of the data to transmit
 */
static void nrf_write_ack_pload( uint8_t pipe, uint8_t *tx_pload,
                                 uint8_t length );

/**
 * @brief Reads the payload width of the received ack payload
 *
 * @return Payload width of the received ack payload
 */
static inline uint8_t nrf_read_rx_pl_w( void );

/**
 * @brief Activate features
 *
 * Sends the ACTIVATE command to the RF tranceiver. By calling this function
 * once it is possible to use the functions related to the FEATURE register.
 */
static inline void nrf_lock_unlock( void );

/**
 * @brief is_locked
 *
 * Checks for feature lock.
 *
 * @return
 * @retval true - locked
 * @retval flase - unlocked
 */
static inline bool is_locked( void );

/**
 * @brief No Operation command.
 *
 * Use this function to receive the radio's
 * status register.
 *
 * @return Status register
*/
static inline uint8_t nrf_nop( void );

/**
 * @brief nrf_get_fifo_status
 *
 * @return
 */
static inline uint8_t nrf_get_fifo_status( void );

/******************************************************************************
* Function Definitions
*******************************************************************************/
/** @name  << BASIC FUNCTIONS >> */

/**
 * @brief Basis function write_reg.
 *
 * Use this function to write a new value to
 * a radio register.
 *
 * @param reg Register to write
 * @param value New value to write
 * @return Status register
*/
static uint8_t nrf_write_reg( uint8_t reg, uint8_t value )
{
    uint8_t retval;

    if( reg < WRITE_REG ) // i.e. this is a register access
        retval = nrf_hal_write_reg( WRITE_REG + reg, value );
    else            // single byte cmd OR future command/register access
    {
        if( !( reg == FLUSH_TX ) && !( reg == FLUSH_RX ) && !( reg == REUSE_TX_PL ) &&
                !( reg == NOP ) )
            retval = nrf_hal_write_reg( reg, value );
        else          // single byte L01 command
            retval = nrf_hal_write_reg( reg, value );
    }

    return retval;
}

/**
 * @brief Basis function, read_multibyte register .
 * Use this function to read multiple bytes from
 * a multibyte radio-register
 *
 * @param reg Multibyte register to read from
 * @param *pbuf Pointer to buffer in which to store read bytes to
 *
 * @return pipe# of received data (MSB), if operation used by a nrf_read_rx_pload
 * @return length of read data (LSB), either for nrf_read_rx_pload or
 * for nrf_get_address.
*/
static uint16_t nrf_read_multibyte_reg( uint8_t reg, uint8_t *pbuf )
{
    uint8_t length, address;

    switch( reg )
    {
        case NRF_PIPE0:
        case NRF_PIPE1:
        case NRF_TX:
            length = nrf_get_address_width();
            address = RX_ADDR_P0 + reg;
            //nrf_hal_read( RX_ADDR_P0 + reg, pbuf, nrf_get_address_width() );
            break;
        case NRF_RX_PLOAD:
            if( ( reg = nrf_get_rx_data_source() ) < 7 )
            {
                length = nrf_read_rx_pl_w();
                address = RD_RX_PLOAD;
            } else
                length = 0;
            break;
        default:
            length = 0;
            break;
    }

    nrf_hal_read( address, pbuf, length );

    return ( ( ( uint16_t ) reg << 8 ) | length );
}

/**
 * @brief Basis function, write_multibyte register.
 *
 * Use this function to write multiple bytes to radio register.
 *
 * @param reg Register to write
 * @param *pbuf pointer to buffer in which data to write is
 * @param length \# of bytes to write
*/
static void nrf_write_multibyte_reg( uint8_t reg, uint8_t *pbuf,
                                     uint8_t length )
{
    switch( reg )
    {
        case NRF_PIPE0:
        case NRF_PIPE1:
        case NRF_TX:
            nrf_hal_write( WRITE_REG + RX_ADDR_P0 + reg, pbuf, nrf_get_address_width() );
            break;
        case NRF_TX_PLOAD:
            nrf_hal_write( WR_TX_PLOAD, pbuf, length );
            break;
        default:
            break;
    }
}

static inline void nrf_write_ack_pload( uint8_t pipe, uint8_t *tx_pload,
                                 uint8_t length )
{
    nrf_hal_write( WR_ACK_PLOAD | pipe, tx_pload, length );
}

static inline uint8_t nrf_read_rx_pl_w()
{
    return nrf_hal_read_reg( RD_RX_PLOAD_W );
}

static inline void nrf_lock_unlock()
{
    nrf_hal_write_reg( LOCK_UNLOCK, 0x73 );
}

static inline bool is_locked()
{
    return ( nrf_hal_read_reg( FEATURE ) == 0x00 &&
             ( nrf_hal_read_reg( DYNPD ) == 0x00 ) )
           ? true : false;
}

// Used to read status register
static inline uint8_t nrf_nop()
{
    return nrf_write_reg( NOP, 0 );
}

static inline uint8_t nrf_get_fifo_status()
{
    return nrf_hal_read_reg( FIFO_STATUS );
}


/****************************************************************************
 * Public Functions
 ***************************************************************************/
/****************************************
 ********** Setup Functions *************
 ***************************************/
int nrf_hw_init( nrf_operation_mode_t operational_mode )
{
    return nrf_hal_init( operational_mode );
}

// Interrupt functions
void nrf_set_irq_mode( nrf_irq_source_t int_source, bool irq_state )
{
    // TODO: Check this logic.  If true it appears to clear the bit
    if( irq_state )
        nrf_write_reg( CONFIG,
                       nrf_hal_read_reg( CONFIG ) & ~SET_BIT( int_source ) );
    else
        nrf_write_reg( CONFIG,
                       nrf_hal_read_reg( CONFIG ) | SET_BIT( int_source ) );
}

bool nrf_get_irq_mode( uint8_t int_type )
{
    if( nrf_hal_read_reg( CONFIG ) & SET_BIT( int_type ) )
        return false;
    else
        return true;
}

uint8_t nrf_get_clear_irq_flags( void )
{
    return nrf_write_reg( STATUS,
                          ( BIT_6 | BIT_5 | BIT_4 ) ) & ( BIT_6 | BIT_5 | BIT_4 );
}

void nrf_clear_irq_flag( nrf_irq_source_t int_source )
{
    nrf_write_reg( STATUS, SET_BIT( int_source ) );
}

uint8_t nrf_get_irq_flags( void )
{
    return nrf_nop() & ( BIT_6 | BIT_5 | BIT_4 );
}

void nrf_enable_ack_pl( void )
{
    if( is_locked() )
        nrf_lock_unlock();

    nrf_write_reg( FEATURE, ( nrf_hal_read_reg( FEATURE ) | 0x02 ) );

    nrf_lock_unlock();
}

void nrf_disable_ack_pl( void )
{
    if( is_locked() )
        nrf_lock_unlock();

    nrf_write_reg( FEATURE, ( nrf_hal_read_reg( FEATURE ) & ~0x02 ) );

    nrf_lock_unlock();
}

void nrf_enable_dynamic_pl( void )
{
    if( is_locked() )
        nrf_lock_unlock();

    nrf_write_reg( FEATURE, ( nrf_hal_read_reg( FEATURE ) | 0x04 ) );

    nrf_lock_unlock();
}

void nrf_disable_dynamic_pl( void )
{
    if( is_locked() )
        nrf_lock_unlock();

    nrf_write_reg( FEATURE, ( nrf_hal_read_reg( FEATURE ) & ~0x04 ) );

    nrf_lock_unlock();
}

void nrf_setup_dyn_pl( uint8_t setup )
{
    if( is_locked() )
        nrf_lock_unlock();

    nrf_write_reg( DYNPD, setup & ~0xC0 );

    nrf_lock_unlock();
}

void nrf_enable_dynamic_ack( void )
{
    if( is_locked() )
        nrf_lock_unlock();

    nrf_write_reg( FEATURE, ( nrf_hal_read_reg( FEATURE ) | 0x01 ) );

    nrf_lock_unlock();
}

void nrf_disable_dynamic_ack( void )
{
    if( is_locked() )
        nrf_lock_unlock();

    nrf_write_reg( FEATURE, ( nrf_hal_read_reg( FEATURE ) & ~0x01 ) );

    nrf_lock_unlock();
}

// CRC Functions
void nrf_set_crc_mode( nrf_crc_mode_t crc_mode )
{
    nrf_write_reg( CONFIG,
                   ( nrf_hal_read_reg( CONFIG ) & ~( BIT_3 | BIT_2 ) ) | ( UINT8(
                               crc_mode ) << 2 ) );
}

uint8_t nrf_get_crc_mode( void )
{
    return ( nrf_hal_read_reg( CONFIG ) & ( BIT_3 | BIT_2 ) ) >> CRCO;
}


// Pipe Functions
void nrf_open_pipe( nrf_address_t pipe_num, bool auto_ack )
{
    switch( pipe_num )
    {
        case NRF_PIPE0:
        case NRF_PIPE1:
        case NRF_PIPE2:
        case NRF_PIPE3:
        case NRF_PIPE4:
        case NRF_PIPE5:
            nrf_write_reg( EN_RXADDR,
                           nrf_hal_read_reg( EN_RXADDR ) | SET_BIT( pipe_num ) );

            if( auto_ack )
                nrf_write_reg( EN_AA, nrf_hal_read_reg( EN_AA ) | SET_BIT( pipe_num ) );
            else
                nrf_write_reg( EN_AA, nrf_hal_read_reg( EN_AA ) & ~SET_BIT( pipe_num ) );
            break;

        case NRF_ALL:
            nrf_write_reg( EN_RXADDR, ~( BIT_7 | BIT_6 ) );

            if( auto_ack )
                nrf_write_reg( EN_AA, ~( BIT_7 | BIT_6 ) );
            else
                nrf_write_reg( EN_AA, 0 );
            break;

        default:
            break;
    }
}

void nrf_close_pipe( nrf_address_t pipe_num )
{
    switch( pipe_num )
    {
        case NRF_PIPE0:
        case NRF_PIPE1:
        case NRF_PIPE2:
        case NRF_PIPE3:
        case NRF_PIPE4:
        case NRF_PIPE5:
            nrf_write_reg( EN_RXADDR,
                           nrf_hal_read_reg( EN_RXADDR ) & ~SET_BIT( pipe_num ) );
            nrf_write_reg( EN_AA, nrf_hal_read_reg( EN_AA ) & ~SET_BIT( pipe_num ) );
            break;

        case NRF_ALL:
            nrf_write_reg( EN_RXADDR, 0 );
            nrf_write_reg( EN_AA, 0 );
            break;

        default:
            break;
    }
}


uint8_t nrf_get_pipe_status( nrf_address_t pipe_num )
{
    uint8_t en_rx, en_aa;

    en_rx = nrf_hal_read_reg( EN_RXADDR ) & ( 1 << ( uint8_t )pipe_num );
    en_aa = nrf_hal_read_reg( EN_AA ) & ( 1 << ( uint8_t )pipe_num );

    en_rx >>= ( uint8_t )pipe_num;
    en_aa >>= ( uint8_t )pipe_num;

    return ( en_aa << 1 ) + en_rx;
}


void nrf_set_address( nrf_address_t address, uint8_t *addr )
{
    switch( address )
    {
        case NRF_TX:
        case NRF_PIPE0:
        case NRF_PIPE1:
            nrf_write_multibyte_reg( ( uint8_t )address, addr, 0 );
            break;
        case NRF_PIPE2:
        case NRF_PIPE3:
        case NRF_PIPE4:
        case NRF_PIPE5:
            nrf_write_reg( RX_ADDR_P0 + ( uint8_t ) address, *addr );
            break;
        default:
            break;
    }
}


uint8_t nrf_get_address( nrf_address_t address, uint8_t *addr )
{
    switch( address )
    {
        case NRF_PIPE0:
        case NRF_PIPE1:
        case NRF_TX:
            return nrf_read_multibyte_reg( address, addr );
        default:
            *addr = nrf_hal_read_reg( RX_ADDR_P0 + address );
            return nrf_get_address_width();
    }
}

void nrf_set_address_width( nrf_address_width_t address_width )
{
    nrf_write_reg( SETUP_AW, ( UINT8( address_width ) - 2 ) );
}

uint8_t nrf_get_address_width( void )
{
    return ( nrf_hal_read_reg( SETUP_AW ) + 2 );
}




// Auto re-transmit and number of transmissions
void nrf_set_auto_retr( uint8_t retr, uint16_t delay )
{
    retr = MIN( retr, 15 );
    nrf_write_reg( SETUP_RETR, ( ( ( delay / 250 ) - 1 ) << 4 ) | retr );
}

uint8_t nrf_get_auto_retr_status( void )
{
    return nrf_hal_read_reg( OBSERVE_TX );
}

uint8_t nrf_get_transmit_attempts( void )
{
    return nrf_hal_read_reg( OBSERVE_TX ) & ( BIT_3 | BIT_2 | BIT_1 | BIT_0 );
}

uint8_t nrf_get_packet_lost_ctr( void )
{
    return ( nrf_hal_read_reg( OBSERVE_TX ) & ( BIT_7 | BIT_6 | BIT_5 | BIT_4 ) )
           >> 4;
}

void nrf_set_rx_pload_width( uint8_t pipe_num, uint8_t pload_width )
{
    nrf_write_reg( RX_PW_P0 + pipe_num, pload_width );
}

uint8_t nrf_get_rx_pload_width( uint8_t pipe_num )
{
    return nrf_hal_read_reg( RX_PW_P0 + pipe_num );
}

/******************************************
 ******** Operation Functions *************
 *****************************************/
// Operation Modes and Radio Settings
void nrf_set_operation_mode( nrf_operation_mode_t op_mode )
{
    if( op_mode == NRF_PRX )
        nrf_write_reg( CONFIG,
                       ( nrf_hal_read_reg( CONFIG ) | ( 1 << PRIM_RX ) ) );
    else
        nrf_write_reg( CONFIG,
                       ( nrf_hal_read_reg( CONFIG ) & ~( 1 << PRIM_RX ) ) );
}

uint8_t nrf_get_operation_mode()
{
    return ( nrf_hal_read_reg( CONFIG ) & ( 1 << PRIM_RX ) );
}

void nrf_start_active_rx()
{
   if( nrf_get_operation_mode() & ( 1 << PRIM_RX ) )
       nrf_hal_listen();
}

void nrf_enter_standby_mode()
{
    nrf_hal_ignore();
}

// Power Mode Functions
void nrf_set_power_mode( nrf_pwr_mode_t pwr_mode )
{
    if( pwr_mode == NRF_PWR_UP )
    {
        nrf_write_reg( CONFIG, ( nrf_hal_read_reg( CONFIG ) | ( 1 << PWR_UP ) ) );
    } else {
        nrf_write_reg( CONFIG,
                       ( nrf_hal_read_reg( CONFIG ) & ~( 1 << PWR_UP ) ) );
    }
}

uint8_t nrf_get_power_mode()
{
    return ( nrf_hal_read_reg( CONFIG ) & ( 1 << PWR_UP ) ) >> PWR_UP;
}




// Channel and Channel Power Functions
void nrf_set_rf_channel( uint8_t channel )
{
    nrf_write_reg( RF_CH, channel );
}

uint8_t nrf_get_rf_channel( void )
{
    return nrf_hal_read_reg( RF_CH );
}

void nrf_set_output_power( nrf_output_power_t power )
{
    nrf_write_reg( RF_SETUP,
                   ( nrf_hal_read_reg( RF_SETUP ) & ~( ( 1 << RF_PWR1 ) |
                           ( 1 << RF_PWR0 ) ) ) | ( UINT8( power ) << 1 ) );
}

uint8_t nrf_get_output_power()
{
    return ( nrf_hal_read_reg( RF_SETUP ) & ( ( 1 << RF_PWR1 ) |
             ( 1 << RF_PWR0 ) ) ) >> RF_PWR0;
}


// DataRate Functions
void nrf_set_datarate( nrf_datarate_t datarate )
{
    if( datarate == NRF_1MBPS )
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << RF_DR ) ) );
    else
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << RF_DR ) ) );
}

uint8_t nrf_get_datarate( void )
{
    return ( nrf_hal_read_reg( RF_SETUP ) & ( 1 << RF_DR ) ) >> RF_DR;
}

void nrf_delay( uint16_t ms )
{
    nrf_hal_delay( ms );
}




/******************************************
 ******** Status Functions *************
 *****************************************/
// FIFO Functions
uint8_t nrf_get_tx_fifo_status( void )
{
    return ( ( nrf_get_fifo_status() & ( ( 1 << TX_FIFO_FULL ) |
                                         ( 1 << TX_EMPTY ) ) ) >> 4 );
}

bool nrf_tx_fifo_empty( void )
{
    return ( bool )( ( nrf_get_fifo_status() >> TX_EMPTY ) & 1 );
}

bool nrf_tx_fifo_full( void )
{
    return ( bool )( ( nrf_get_fifo_status() >> TX_FIFO_FULL ) & 1 );
}

uint8_t nrf_get_rx_fifo_status( void )
{
    return ( nrf_get_fifo_status() & ( ( 1 << RX_FULL ) |
                                       ( 1 << RX_EMPTY ) ) );
}

bool nrf_rx_fifo_empty( void )
{
    if( nrf_get_rx_data_source() == 7 )
        return true;
    else
        return false;
}

bool nrf_rx_fifo_full( void )
{
    return ( bool )( ( nrf_get_fifo_status() >> RX_EMPTY ) & 1 );
}

void nrf_flush_rx( void )
{
    nrf_write_reg( FLUSH_RX, 0 );
}

void nrf_flush_tx( void )
{
    nrf_write_reg( FLUSH_TX, 0 );
}

bool nrf_get_carrier_detect( void )
{
    return nrf_hal_read_reg( CD ) & 1;
}


/*************************************
 *********** Data Operations *********
 ************************************/
nrf_address_t nrf_get_rx_data_source( void )
{
    return ( ( nrf_nop() & ( BIT_3 | BIT_2 | BIT_1 ) ) >> 1 );
}

// Transmit and Recieve Functions
// Fixed: returns length==0 and pipe==7 means FIFO empty
uint16_t nrf_read_rx_pload( uint8_t *rx_pload )
{
    return nrf_read_multibyte_reg( UINT8( NRF_RX_PLOAD ), rx_pload );
}

void nrf_write_tx_pload( uint8_t *tx_pload, uint8_t length )
{
    nrf_write_multibyte_reg( UINT8( NRF_TX_PLOAD ), tx_pload, length );
}

// Reuse Functions
void nrf_reuse_tx( void )
{
    nrf_write_reg( REUSE_TX_PL, 0 );
}

bool nrf_get_reuse_tx_status( void )
{
    return ( bool )( ( nrf_get_fifo_status() & ( 1 << TX_REUSE ) ) >>
                     TX_REUSE );
}

/************************************
 ***** Test Functions ***************
 ***********************************/
// PLL Clock Functions
void nrf_set_pll_mode( nrf_pll_mode_t pll_mode )
{
    if( pll_mode == NRF_PLL_LOCK )
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << PLL_LOCK ) ) );
    else
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << PLL_LOCK ) ) );
}

nrf_pll_mode_t nrf_get_pll_mode( void )
{
    return ( nrf_pll_mode_t )( ( nrf_hal_read_reg( RF_SETUP ) &
                                 ( 1 << PLL_LOCK ) ) >> PLL_LOCK );
}

// LNA Gain Functions
void nrf_set_lna_gain( nrf_lna_mode_t lna_gain )
{
    if( lna_gain == NRF_LNA_HCURR )
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) | ( 1 << LNA_HCURR ) ) );
    else
        nrf_write_reg( RF_SETUP,
                       ( nrf_hal_read_reg( RF_SETUP ) & ~( 1 << LNA_HCURR ) ) );
}

nrf_lna_mode_t nrf_get_lna_gain( void )
{
    return ( nrf_lna_mode_t ) ( ( nrf_hal_read_reg( RF_SETUP ) &
                                  ( 1 << LNA_HCURR ) ) >> LNA_HCURR );
}


/*************** END OF FUNCTIONS ***************************************************************************/
