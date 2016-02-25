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
#include "nrf24l01_hw.h"
#include "nrf24l01_hal.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
/** @name - Instruction Set - */
#define NRF_WRITE_REG 0x20  /**< Register write command */
#define RD_RX_PLOAD_W 0x60  /**< Read RX payload command */
#define RD_RX_PLOAD   0x61  /**< Read RX payload command */
#define WR_TX_PLOAD   0xA0  /**< Write TX payload command */
#define WR_ACK_PLOAD  0xA8  /**< Write ACK payload command */
#define WR_NAC_TX_PLOAD 0xB0  /**< Write ACK payload command */
#define FLUSH_TX      0xE1  /**< Flush TX register command */
#define FLUSH_RX      0xE2  /**< Flush RX register command */
#define REUSE_TX_PL   0xE3  /**< Reuse TX payload command */
#define LOCK_UNLOCK   0x50  /**< Lock/unlcok exclusive features */

#define NOP           0xFF  /**< No Operation command, used for reading status register */

/** Register Memory Map - */
#define CONFIG        0x00  /**< nRF24L01 config register */
#define EN_AA         0x01  /**< nRF24L01 enable Auto-Acknowledge register */
#define EN_RXADDR     0x02  /**< nRF24L01 enable RX addresses register */
#define SETUP_AW      0x03  /**< nRF24L01 setup of address width register */
#define SETUP_RETR    0x04  /**< nRF24L01 setup of automatic retransmission register */
#define RF_CH         0x05  /**< nRF24L01 RF channel register */
#define RF_SETUP      0x06  /**< nRF24L01 RF setup register */
#define STATUS        0x07  /**< nRF24L01 status register */
#define OBSERVE_TX    0x08  /**< nRF24L01 transmit observe register */
#define CD            0x09  /**< nRF24L01 carrier detect register */
#define RX_ADDR_P0    0x0A  /**< nRF24L01 receive address data pipe0 */
#define RX_ADDR_P1    0x0B  /**< nRF24L01 receive address data pipe1 */
#define RX_ADDR_P2    0x0C  /**< nRF24L01 receive address data pipe2 */
#define RX_ADDR_P3    0x0D  /**< nRF24L01 receive address data pipe3 */
#define RX_ADDR_P4    0x0E  /**< nRF24L01 receive address data pipe4 */
#define RX_ADDR_P5    0x0F  /**< nRF24L01 receive address data pipe5 */
#define TX_ADDR       0x10  /**< nRF24L01 transmit address */
#define RX_PW_P0      0x11  /**< nRF24L01 \# of bytes in rx payload for pipe0 */
#define RX_PW_P1      0x12  /**< nRF24L01 \# of bytes in rx payload for pipe1 */
#define RX_PW_P2      0x13  /**< nRF24L01 \# of bytes in rx payload for pipe2 */
#define RX_PW_P3      0x14  /**< nRF24L01 \# of bytes in rx payload for pipe3 */
#define RX_PW_P4      0x15  /**< nRF24L01 \# of bytes in rx payload for pipe4 */
#define RX_PW_P5      0x16  /**< nRF24L01 \# of bytes in rx payload for pipe5 */
#define FIFO_STATUS   0x17  /**< nRF24L01 FIFO status register */
#define DYNPD         0x1C  /**< nRF24L01 Dynamic payload setup */
#define FEATURE       0x1D  /**< nRF24L01 Exclusive feature setup */

#define MASK_RX_DR    6     /**< CONFIG register bit 6 */
#define MASK_TX_DS    5     /**< CONFIG register bit 5 */
#define MASK_MAX_RT   4     /**< CONFIG register bit 4 */
#define EN_CRC        3     /**< CONFIG register bit 3 */
#define CRCO          2     /**< CONFIG register bit 2 */
#define PWR_UP        1     /**< CONFIG register bit 1 */
#define PRIM_RX       0     /**< CONFIG register bit 0 */

/** @name RF_SETUP register bit definitions */
//@{
#define PLL_LOCK      4     /**< RF_SETUP register bit 4 */
#define RF_DR         3     /**< RF_SETUP register bit 3 */
#define RF_PWR1       2     /**< RF_SETUP register bit 2 */
#define RF_PWR0       1     /**< RF_SETUP register bit 1 */
#define LNA_HCURR     0     /**< RF_SETUP register bit 0 */
//@}

/* STATUS 0x07 */
/** @name STATUS register bit definitions */
//@{
#define RX_DR         6     /**< STATUS register bit 6 */
#define TX_DS         5     /**< STATUS register bit 5 */
#define MAX_RT        4     /**< STATUS register bit 4 */
#define TX_FULL       0     /**< STATUS register bit 0 */
//@}

/* FIFO_STATUS 0x17 */
/** @name FIFO_STATUS register bit definitions */
//@{
#define TX_REUSE      6     /**< FIFO_STATUS register bit 6 */
#define TX_FIFO_FULL  5     /**< FIFO_STATUS register bit 5 */
#define TX_EMPTY      4     /**< FIFO_STATUS register bit 4 */
#define RX_FULL       1     /**< FIFO_STATUS register bit 1 */
#define RX_EMPTY      0     /**< FIFO_STATUS register bit 0 */
//@}

#define NRF_BIT_0 0x01 /**< The value of bit 0 */
#define NRF_BIT_1 0x02 /**< The value of bit 1 */
#define NRF_BIT_2 0x04 /**< The value of bit 2 */
#define NRF_BIT_3 0x08 /**< The value of bit 3 */
#define NRF_BIT_4 0x10 /**< The value of bit 4 */
#define NRF_BIT_5 0x20 /**< The value of bit 5 */
#define NRF_BIT_6 0x40 /**< The value of bit 6 */
#define NRF_BIT_7 0x80 /**< The value of bit 7 */

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
/** Swaps the upper byte with the lower byte in a 16 bit variable */
#define NRF_SWAP(x) ((((x)&0xFF)<<8)|(((x)>>8)&0xFF))

#define NRF_SET_BIT(pos) ((uint8_t) (1<<( (uint8_t) (pos) )))

/** The upper 8 bits of a 16 bit value */
#define NRF_MSB(a) ((a & 0xFF00) >> 8)
/** The lower 8 bits (of a 16 bit value) */
#define NRF_LSB(a) ((a & 0xFF))

/** Leaves the minimum of the two arguments */
#define NRF_MIN(a, b) ((a) < (b) ? (a) : (b))
/** Leaves the maximum of the two arguments */
#define NRF_MAX(a, b) ((a) < (b) ? (b) : (a))

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
 * @retval false - unlocked
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

    if( reg < NRF_WRITE_REG ) // i.e. this is a register access
        retval = nrf_hal_write_reg( NRF_WRITE_REG + reg, value );
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
    uint8_t length  = 0;
	uint8_t address = 0;

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
            nrf_hal_write( NRF_WRITE_REG + RX_ADDR_P0 + reg, pbuf, nrf_get_address_width() );
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
                       nrf_hal_read_reg( CONFIG ) & ~NRF_SET_BIT( int_source ) );
    else
        nrf_write_reg( CONFIG,
                       nrf_hal_read_reg( CONFIG ) | NRF_SET_BIT( int_source ) );
}

bool nrf_get_irq_mode( uint8_t int_type )
{
    if( nrf_hal_read_reg( CONFIG ) & NRF_SET_BIT( int_type ) )
        return false;
    else
        return true;
}

uint8_t nrf_get_clear_irq_flags( void )
{
    return nrf_write_reg( STATUS,
                          ( NRF_BIT_6 | NRF_BIT_5 | NRF_BIT_4 ) ) & ( NRF_BIT_6 | NRF_BIT_5 | NRF_BIT_4 );
}

void nrf_clear_irq_flag( nrf_irq_source_t int_source )
{
    nrf_write_reg( STATUS, NRF_SET_BIT( int_source ) );
}

uint8_t nrf_get_irq_flags( void )
{
    return nrf_nop() & ( NRF_BIT_6 | NRF_BIT_5 | NRF_BIT_4 );
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
    nrf_write_reg( CONFIG, ( nrf_hal_read_reg( CONFIG ) &
    			~( ( NRF_BIT_3 | NRF_BIT_2 ) | ( uint8_t )( crc_mode << 2 ) ) ) );
}

uint8_t nrf_get_crc_mode( void )
{
    return ( nrf_hal_read_reg( CONFIG ) & ( NRF_BIT_3 | NRF_BIT_2 ) ) >> CRCO;
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
                           nrf_hal_read_reg( EN_RXADDR ) | NRF_SET_BIT( pipe_num ) );

            if( auto_ack )
                nrf_write_reg( EN_AA, nrf_hal_read_reg( EN_AA ) | NRF_SET_BIT( pipe_num ) );
            else
                nrf_write_reg( EN_AA, nrf_hal_read_reg( EN_AA ) & ~NRF_SET_BIT( pipe_num ) );
            break;

        case NRF_ALL:
            nrf_write_reg( EN_RXADDR, ~( NRF_BIT_7 | NRF_BIT_6 ) );

            if( auto_ack )
                nrf_write_reg( EN_AA, ~( NRF_BIT_7 | NRF_BIT_6 ) );
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
                           nrf_hal_read_reg( EN_RXADDR ) & ~NRF_SET_BIT( pipe_num ) );
            nrf_write_reg( EN_AA, nrf_hal_read_reg( EN_AA ) & ~NRF_SET_BIT( pipe_num ) );
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
    nrf_write_reg( SETUP_AW, ( uint8_t )( address_width - 2 ) );
}

uint8_t nrf_get_address_width( void )
{
    return ( nrf_hal_read_reg( SETUP_AW ) + 2 );
}




// Auto re-transmit and number of transmissions
void nrf_set_auto_retr( uint8_t retr, uint16_t delay )
{
    retr = NRF_MIN( retr, 15 );
    nrf_write_reg( SETUP_RETR, ( ( ( delay / 250 ) - 1 ) << 4 ) | retr );
}

uint8_t nrf_get_auto_retr_status( void )
{
    return nrf_hal_read_reg( OBSERVE_TX );
}

uint8_t nrf_get_transmit_attempts( void )
{
    return nrf_hal_read_reg( OBSERVE_TX ) &
    		( NRF_BIT_3 | NRF_BIT_2 | NRF_BIT_1 | NRF_BIT_0 );
}

uint8_t nrf_get_packet_lost_ctr( void )
{
    return ( nrf_hal_read_reg( OBSERVE_TX ) &
    		( NRF_BIT_7 | NRF_BIT_6 | NRF_BIT_5 | NRF_BIT_4 ) ) >> 4;
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
                           ( 1 << RF_PWR0 ) ) ) | ( ( uint8_t )( power ) << 1 ) );
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
    return ( ( nrf_nop() & ( NRF_BIT_3 | NRF_BIT_2 | NRF_BIT_1 ) ) >> 1 );
}

// Transmit and Recieve Functions
// Fixed: returns length==0 and pipe==7 means FIFO empty
uint16_t nrf_read_rx_pload( uint8_t *rx_pload )
{
    return nrf_read_multibyte_reg( ( uint8_t )( NRF_RX_PLOAD ), rx_pload );
}

void nrf_write_tx_pload( uint8_t *tx_pload, uint8_t length )
{
    nrf_write_multibyte_reg( ( uint8_t )( NRF_TX_PLOAD ), tx_pload, length );
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
