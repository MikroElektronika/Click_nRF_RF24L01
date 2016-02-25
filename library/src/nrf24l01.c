/*******************************************************************************
* Title                 :   Nordic nRF24L01+ API
* Filename              :   nrf.c
* Author                :   RBL
* Origin Date           :   11/01/2016
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  11/01/16           .1         RBL        Module Created.
*
*******************************************************************************/
/**
 * @file nrf.c
 * @brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "nrf24l01.h"
#include "nrf24l01_hw.h"
#include "nrf_common.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static uint8_t tx_address[5];
static bool volatile success_tx;
static bool ack_enabled;
static uint32_t volatile timeout;

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
void nrf_sb_init( nrf_addr_map_t *address,
                  nrf_operation_mode_t operational_mode )
{
    nrf_hw_init( operational_mode );
    /* First close all radio pipes. Pipe 0 and 1 open by default */
    nrf_close_pipe( NRF_ALL );
    /* Operates in 16bits CRC mode */
    nrf_set_crc_mode( NRF_CRC_16BIT );
    /* Disables auto retransmit */
    nrf_set_auto_retr( 0, NRF_RETRANS_DELAY );
    /* 5 bytes address width */
    nrf_set_address_width( NRF_AW_5BYTES );
    
    nrf_clear_irq_flag( NRF_MAX_RT );     /**< Max retries interrupt */
    nrf_clear_irq_flag( NRF_TX_DS );      /**< TX data sent interrupt */
    nrf_clear_irq_flag( NRF_RX_DR );
    
    nrf_flush_rx(); nrf_flush_tx(); /* Flush FIFOs */

    if( address )
    {
        //uint8_t *ptr = ( uint8_t * )address;

        /* Set device's addresses */
        nrf_set_address( NRF_PIPE0, address->p0 );
        /* Open pipe0, without/autoack */
        nrf_open_pipe( NRF_PIPE0, false );

        if( address->p1[0] > 0 )
        {
            nrf_set_address( NRF_PIPE1, address->p1 );
            nrf_open_pipe( NRF_PIPE1, false );
        }

        if( address->p2[0] > 0 )
        {
            nrf_set_address( NRF_PIPE2, address->p2 );
            nrf_open_pipe( NRF_PIPE2, false );
        }

        if( address->p3[0] > 0 )
        {
            nrf_set_address( NRF_PIPE3, address->p3 );
            nrf_open_pipe( NRF_PIPE3, false );
        }

        if( address->p4[0] > 0 )
        {
            nrf_set_address( NRF_PIPE4, address->p4 );
            nrf_open_pipe( NRF_PIPE4, false );
        }

        if( address->p5[0] > 0 )
        {
            nrf_set_address( NRF_PIPE5, address->p5 );
            nrf_open_pipe( NRF_PIPE5, false );
        }

        /* Sets recieving address on pipe0 */
        if( operational_mode == NRF_PTX )
        {
            nrf_set_address( NRF_TX, address->p0 );
            nrf_set_operation_mode( NRF_PTX );
        } else {
            nrf_set_address( NRF_TX, address->tx );
            nrf_set_operation_mode( NRF_PRX );
            nrf_set_rx_pload_width( ( uint8_t )NRF_PIPE0, NRF_PAYLOAD_LENGTH );
        }
    }

    /* Operating on static channel, Frequenzy = 2400 + RF_CHANNEL */
    nrf_set_rf_channel( NRF_CHANNEL );

    nrf_set_power_mode( NRF_PWR_UP );

    nrf_delay( NRF_POWER_UP_DELAY );                // Wait for the radio to
    
    if( operational_mode == NRF_PRX ) 
        nrf_start_active_rx();
    else
        nrf_enter_standby_mode();
}


void nrf_esb_init( nrf_addr_map_t *address,
                   nrf_operation_mode_t operational_mode )
{
    if( address )
    {
        /* Open pipe(s), with autoack */
        //uint8_t *ptr = ( uint8_t * )address;
        nrf_sb_init( address, operational_mode );

        /* Open pipe0, with/autoack */
        nrf_open_pipe( NRF_PIPE0, true );

        if( address->p1[0] > 0 )
            nrf_open_pipe( NRF_PIPE1, true );

        if( address->p2[0] > 0 )
            nrf_open_pipe( NRF_PIPE2, true );

        if( address->p3[0] > 0 )
            nrf_open_pipe( NRF_PIPE3, true );

        if( address->p4[0] > 0 )
            nrf_open_pipe( NRF_PIPE4, true );

        if( address->p5[0] > 0 )
            nrf_open_pipe( NRF_PIPE5, true );

        ack_enabled = true;
    }

    nrf_set_auto_retr( NRF_RETRANSMITS, NRF_RETRANS_DELAY );
}


void nrf_esb_bidirection_init( nrf_addr_map_t *address,
                               nrf_operation_mode_t operational_mode )
{
    nrf_esb_init( address, operational_mode );

    nrf_enable_ack_pl();        // Enable ack payload
    nrf_enable_dynamic_pl();    // Enables dynamic payload
    nrf_setup_dyn_pl( 0x3f );   // Sets up dynamic payload on all data pipes.
}


uint8_t nrf_send_data( uint8_t *address, uint8_t *data_out, uint8_t count )
{
    uint32_t time_up;
    
    count = MIN( count, NRF_PAYLOAD_LENGTH );
    success_tx = false;

    if( address != NULL && memcmp( address, tx_address, 5 ) )
        nrf_set_address( NRF_TX, address );
    else
        nrf_write_tx_pload( data_out, count );
    
    time_up = timeout + 13;
    
    while( !success_tx )
    {
       if( timeout > time_up )
       {
           break;
       }
    }
    return count;
}


uint8_t nrf_recieve_data( nrf_address_t *address, uint8_t *data_in )
{
    *address = nrf_get_rx_data_source();
    nrf_read_rx_pload( data_in );

    return nrf_get_rx_pload_width( *address );
}

void nrf_acknowledged()
{
     success_tx = true;
}

bool nrf_is_interrupted( nrf_irq_source_t source )
{
    uint8_t flags = nrf_get_irq_flags();

    if( flags & ( uint8_t )( 1 << source ) )
    {
        nrf_clear_irq_flag( source );
        
        if( source == NRF_MAX_RT )
            nrf_flush_tx();
        return true;
    } else
        return false;
}

void nrf_timer_tick()
{
    timeout++;
}



/*************** END OF FUNCTIONS ***************************************************************************/