/*******************************************************************************
* Title                 :   HAL Implementation
* Filename              :   nrf24l01_hal.c
* Author                :   RBL
* Origin Date           :   12/12/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  12/12/15           .1         RBL      Module Created.
*
*******************************************************************************/
/**
 * @file nrf_hal.c
 * @brief Implementation adapted from Nordic
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "nrf24l01_hal.h"
#include "nrf24l01_defs.h"
#include <stddef.h>

#if defined( __GNUC__ )
#if defined( STM32F0 )
#include"stm32f0xx.h"
#elif defined( STM32F1 )
#include "stm32f1xx.h"
#elif defined( STM32F2 )
#include "stm32f2xx.h"
#elif defined( STM32F3 )
#include "stm32f3xx.h"
#elif defined( STM32F4 )
#include "stm32f4xx.h"
#elif defined( STM32F7 )
#include "stm32f7xx.h"
#endif
#endif

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
#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32 )

static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );

#elif defined( TI )

static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );

#endif

#elif defined( __MIKROC_PRO_FOR_AVR__ )

static unsigned char( *spi_read_p )( unsigned char data_in );
static void( *spi_write_p )( unsigned char data_out );

#elif defined( __MIKROC_PRO_FOR_PIC__ )

static unsigned char( *spi_read_p )( unsigned char _data );
static void ( *spi_write_p )( unsigned char data_out );

#elif defined( __MIKROC_PRO_FOR_PIC32__ )

static unsigned long( *spi_read_p )( unsigned long buffer );
static void ( *spi_write_p )( unsigned long data_out );

#elif defined( __MIKROC_PRO_FOR_DSPIC__ )

static unsigned int( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );

#elif defined( __MIKROC_PRO_FOR_8051__ )


#elif defined( __MIKROC_PRO_FOR_FT90x__ )

static unsigned char( *spi_read_p )( unsigned char dummy );
static void( *spi_write_p )( unsigned char dataOut );
#elif defined ( __GNUC__ )
#if defined( STM32F0 ) || defined( STM32F1 ) || defined( STM32F2 ) || \
	defined( STM32F3 ) || defined( STM32F4 ) || defined( STM32F7 )
static HAL_StatusTypeDef ( *spi_write_p )( SPI_HandleTypeDef *hspi,
										   uint8_t *pData,
										   uint16_t Size,
										   uint32_t Timeout );
static HAL_StatusTypeDef ( *spi_read_p )( SPI_HandleTypeDef *hspi,
										  uint8_t *pData,
										  uint16_t Size,
										  uint32_t Timeout );
#endif
#endif

#if defined( __MIKROC_PRO_FOR_ARM__ )   || \
    defined( __MIKROC_PRO_FOR_AVR__ )   || \
    defined( __MIKROC_PRO_FOR_PIC__ )   || \
    defined( __MIKROC_PRO_FOR_PIC32__ ) || \
    defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
    defined( __MIKROC_PRO_FOR_8051__ )  || \
    defined( __MIKROC_PRO_FOR_FT90x__ )
extern sfr sbit NRF_CS_PIN;
extern sfr sbit NRF_CE_PIN;
#elif defined( __GNUC__ )
	#if defined( STM32F0 ) || defined( STM32F1 ) || defined( STM32F2 ) || \
		defined( STM32F3 ) || defined( STM32F4 ) || defined( STM32F7 )
	extern uint16_t NRF_CS_PIN;
	extern GPIO_TypeDef *NRF_CS_PORT;
	extern uint16_t NRF_CE_PIN;
	extern GPIO_TypeDef *NRF_CE_PORT;
	extern SPI_HandleTypeDef *NRF_SPI_PORT;
	#endif
#endif


/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void CSN_LOW( void );
static void CSN_HIGH( void );
static void CE_LOW( void );
static void CE_HIGH( void );
static void CE_PULSE( void );

/******************************************************************************
* Function Definitions
*******************************************************************************/
static void CSN_LOW()
{
	#if defined( __GNUC__ )
	HAL_GPIO_WritePin( NRF_CS_PORT, NRF_CS_PIN, GPIO_PIN_RESET );
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    NRF_CS_PIN = 0;
	#endif
}

static void CSN_HIGH()
{
	#if defined( __GNUC__ )
	HAL_GPIO_WritePin( NRF_CS_PORT, NRF_CS_PIN, GPIO_PIN_SET );
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
		  defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    NRF_CS_PIN = 1;
#endif
}

static void CE_LOW()
{
	#if defined( __GNUC__ )
	HAL_GPIO_WritePin( NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET );
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    NRF_CE_PIN = 0;
	#endif
}

static void CE_HIGH()
{
    #if defined( __GNUC__ )
	HAL_GPIO_WritePin( NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET );
    #elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
	      defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    NRF_CE_PIN = 1;
    #endif
}

/**
 * @brief Pulses the CE to nRF24L01 for at least 10 us
 */
static void CE_PULSE()
{
    CE_HIGH();
    #if defined( __GNUC__ )
		#if defined( STM32F0 ) || defined( STM32F1 ) || defined( STM32F2 ) || \
			defined( STM32F3 ) || defined( STM32F4 ) || defined( STM32F7 )
    	HAL_Delay( 1 );
		#endif
    #elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    Delay_10us();
    #endif
    CE_LOW();
}


int nrf_hal_init( nrf_operation_mode_t mode )
{
    CSN_HIGH();

    /* nRF24L01 goes into standby-I mode if CE is low. Otherwise next payload
     * in TX FIFO is transmitted. If TX FIFO is empty and CE is still high,
     * nRF24L01 enters standby-II mode.
     *
     * Active RX/TX mode is started by setting CE high.
     */
    if( mode == NRF_PTX ) 
        CE_LOW();
    else 
    	CE_HIGH();

    #if defined( __MIKROC_PRO_FOR_ARM__ )   || \
        defined( __MIKROC_PRO_FOR_AVR__ )   || \
        defined( __MIKROC_PRO_FOR_PIC__ )   || \
        defined( __MIKROC_PRO_FOR_PIC32__ ) || \
        defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
        defined( __MIKROC_PRO_FOR_8051__ )
    spi_read_p = SPI_Rd_Ptr;
    spi_write_p = SPI_Wr_Ptr;
    #elif defined( __MIKROC_PRO_FOR_FT90x__ )
    spi_read_p = SPIM_Rd_Ptr;
    spi_write_p = SPIM_Wr_Ptr;
    #elif defined( __GNUC__ )
		#if defined( STM32F0 ) || defined( STM32F1 ) || defined( STM32F2 ) || \
			defined( STM32F3 ) || defined( STM32F4 ) || defined( STM32F7 )
    	spi_read_p = HAL_SPI_Receive;
    	spi_write_p = HAL_SPI_Transmit;

    	#endif
	#endif
    return ( spi_read_p == NULL || spi_write_p == NULL ) ? -1 : 0;
}


void nrf_hal_listen()
{
    CE_HIGH();
	#if defined( __GNUC__ )
		#if defined( STM32F0 ) || defined( STM32F1 ) || defined( STM32F2 ) || \
			defined( STM32F3 ) || defined( STM32F4 ) || defined( STM32F7 )
    	HAL_Delay( 1 );
		#endif
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    Delay_50us();
    Delay_50us();
    Delay_50us();
	#endif
}

void nrf_hal_ignore()
{
    CE_LOW();
}


uint8_t nrf_hal_write_reg( uint8_t address, uint8_t value )
{
    uint8_t status = 0;

    CSN_LOW();
	#if defined( __GNUC__ )
    //TODO: Needs to be a read/write to get status
    spi_write_p( NRF_SPI_PORT, &address, 1, 0 );
    spi_write_p( NRF_SPI_PORT, &value, 1, 0 );
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    status = spi_read_p( address );
    spi_write_p( value );
	#endif
    CSN_HIGH();

    return status;
}


uint8_t nrf_hal_read_reg( uint8_t address )
{
    uint8_t read_value;

    CSN_LOW();
	#if defined( __GNUC__ )
    spi_write_p( NRF_SPI_PORT, &address, 1, 0 );
    spi_read_p( NRF_SPI_PORT, &read_value, 1, 0 );
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    spi_write_p( address );
    read_value = spi_read_p( 0x00 );
	#endif
    CSN_HIGH();

    return read_value;
}

int nrf_hal_write( uint8_t address, uint8_t *data_in, uint8_t count )
{
    CSN_LOW();
	#if defined( __GNUC__ )
    spi_write_p( NRF_SPI_PORT, &address, 1, 0 );
    spi_write_p( NRF_SPI_PORT, data_in, count, 0 );
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    spi_write_p( address );
    while( count-- )
        spi_write_p( *( data_in++ ) );
	#endif
    CSN_HIGH();

    if( address == 0xA0 )
        CE_PULSE();

    return 0;
}

int nrf_hal_read( uint8_t address, uint8_t *data_out, uint8_t count )
{
    if( count == 0 )
        return 0;

    CSN_LOW();
	#if defined( __GNUC__ )
    spi_write_p( NRF_SPI_PORT, &address, 1, 0 );
    spi_read_p( NRF_SPI_PORT, data_out, count, 0 );
	#elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    spi_write_p( address );

    while( count-- )
        *( data_out++ ) = spi_read_p( 0x00 );
#endif
    CSN_HIGH();

    return 0;
}

void nrf_hal_delay( uint16_t ms )
{
	#if defined( __GNUC__ )
		#if defined( STM32F0 ) || defined( STM32F1 ) || defined( STM32F2 ) || \
			defined( STM32F3 ) || defined( STM32F4 ) || defined( STM32F7 )
    	HAL_Delay( ms );
		#endif
    #elif defined( __MIKROC_PRO_FOR_ARM__ )   || \
	      defined( __MIKROC_PRO_FOR_AVR__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC__ )   || \
		  defined( __MIKROC_PRO_FOR_PIC32__ ) || \
		  defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
		  defined( __MIKROC_PRO_FOR_8051__ )  || \
		  defined( __MIKROC_PRO_FOR_FT90x__ )
    while( ms-- )
    	  Delay_1ms();
  #endif
}


/*************** END OF FUNCTIONS ***************************************************************************/
