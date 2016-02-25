#include "nrf24l01.h"
#include <stdint.h>
#include <stdbool.h>


// TFT module connections
unsigned int TFT_DataPort at GPIOE_ODR;
sbit TFT_RST at GPIOE_ODR.B8;
sbit TFT_RS at GPIOE_ODR.B12;
sbit TFT_CS at GPIOE_ODR.B15;
sbit TFT_RD at GPIOE_ODR.B10;
sbit TFT_WR at GPIOE_ODR.B11;
sbit TFT_BLED at GPIOE_ODR.B9;
// End TFT module connections
static bool update_time;
static bool acknowledge_flag;
static uint32_t volatile data_sent;

#ifdef TRANSMITTER
sbit NRF_CS_PIN at GPIOD_ODR.B13;
sbit NRF_CE_PIN at GPIOC_ODR.B2;
#else
sbit NRF_CS_PIN at GPIOE_ODR.B10;
sbit NRF_CE_PIN at GPIOE_ODR.B7;
#endif

#ifdef TRANSMITTER
static const nrf_addr_map_t addresses = {
    {0xB3, 0xB4, 0xB5, 0xB6, 0xF1}, //RX PIPE 0
    {0},                            //RX PIPE 1
    {0},                            //RX PIPE 2
    {0},                            //RX PIPE 3
    {0},                            //RX PIPE 4
    {0},                            //RX PIPE 5
    {0xB3, 0xB4, 0xB5, 0xB6, 0xF1}  //TX PIPE
};
#else
static const nrf_addr_map_t addresses =
{
    { 0x78, 0x78, 0x78, 0x78, 0x78 }, //RX PIPE 0
    { 0xB3, 0xB4, 0xB5, 0xB6, 0xF1 }, //RX PIPE 1
    { 0xF2 },                         //RX PIPE 2
    { 0xF3 },                         //RX PIPE 3
    { 0xF4 },                         //RX PIPE 4
    { 0xF5 },                         //RX PIPE 5
    { 0 }                             //TX PIPE
};
#endif

static bool volatile process_flag;
static uint8_t lost_packets;

static void tft_initialize( void );
static void tft_update( void );
static void system_init( void );
static void init_timer2( void );
static void init_timer3( void );
static void debug_routine( void );

static void tft_initialize()
{
    TFT_Init_ILI9341_8bit( 320, 280 );
    TFT_BLED = 1;
    TFT_Set_Default_Mode();
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Brush( 1, CL_WHITE, 0, 0, 0, 0 );
    TFT_Set_Font( TFT_defaultFont, CL_BLACK, FO_HORIZONTAL );

    TFT_Fill_Screen( CL_WHITE );
    TFT_Set_Pen( CL_BLACK, 1 );
    TFT_Line( 20, 222, 300, 222 );
    TFT_Write_Text( "EasyMx PRO v7 for STM32", 19, 223 );
    TFT_Write_Text( "www.mikroe.com", 200, 223 );
}

static void tft_update()
{
    char tmp_txt[80];
    TFT_Set_Font( TFT_defaultFont, CL_WHITE, FO_HORIZONTAL );
    TFT_Write_Text( "ERROR", 138, 25 );

    
    update_time = false;
}


static void init_timer2()
{
    RCC_APB1ENR.TIM2EN = 1;
    TIM2_CR1.CEN = 0;
    TIM2_PSC = 2399;
    TIM2_ARR = 62499;
    NVIC_IntEnable(IVT_INT_TIM2);
    TIM2_DIER.UIE = 1;
    TIM2_CR1.CEN = 1;
}

static void init_timer3()
{
    RCC_APB1ENR.TIM3EN = 1;
    TIM3_CR1.CEN = 0;
    TIM3_PSC = 23;
    TIM3_ARR = 62499;
    NVIC_IntEnable(IVT_INT_TIM3);
    TIM3_DIER.UIE = 1;
    TIM3_CR1.CEN = 1;
}

static void system_init()
{
    DisableInterrupts();
    RCC_APB2ENR |= ( 1 << SYSCFGEN );  /* Enable clock for alternate pin functions */
    
    #ifdef TRANSMITTER
     /* APB1 Peripheral clock is 42Mhz.  Max is 8 Mhz on radio */
    SPI3_Init_Advanced( _SPI_FPCLK_DIV8,
                       _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                        _SPI_FIRST_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                        &_GPIO_MODULE_SPI3_PC10_11_12 );

    GPIO_Digital_Output( &GPIOC_BASE, _GPIO_PINMASK_2 );
    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
    GPIO_Digital_Input( &GPIOD_BASE, _GPIO_PINMASK_10 );
    SYSCFG_EXTICR3 |= ( 1 << EXTI100 ) | ( 1 << EXTI101 ); /* Map external interrupt on PD10 */
    EXTI_FTSR |= ( 1 << TR10 );  /* Falling event for pin PD10 */
    //EXTI_RTSR |= ( 1 << TR10 );  /* Rising edge for pin PD10 */
    EXTI_IMR |= ( 1 << MR10 );       /* Mask pin PB1 for interrupt */
    //EXTI_EMR |= ( 1 << MR10 );    /* Event mask on pin PD10 */
    NVIC_IntEnable( IVT_INT_EXTI15_10 );   /* Radio interrupt */
    #else
    /* APB1 Peripheral clock is 42Mhz.  Max is 8 Mhz on radio */
    SPI3_Init_Advanced( _SPI_FPCLK_DIV8,
                        _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                        _SPI_FIRST_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                        &_GPIO_MODULE_SPI3_PC10_11_12 );
    
    GPIO_Digital_Output( &GPIOE_BASE, _GPIO_PINMASK_7 | _GPIO_PINMASK_10 );
    GPIO_Digital_Input( &GPIOB_BASE, _GPIO_PINMASK_1 );
    SYSCFG_EXTICR1 |= ( 1 << EXTI10 ) | ( 1 << EXTI01 ); /* Map external interrupt on PB1 */
    EXTI_FTSR |= ( 1 << TR1 );  /* Falling event for pin PB1 */
    //EXTI_RTSR |= ( 1 << TR1 ) | ( 1 << TR0 );  /* Rising edge for pin PB1 */
    EXTI_IMR |= ( 1 << 1 );       /* Mask pin PB1 for interrupt */
    //EXTI_EMR |= ( 1 << MR1 )| ( 1 << MR0 );    /* Event mask on pin PB1 */
    NVIC_IntEnable( IVT_INT_EXTI1 );   /* Radio interrupt */
    #endif

    nrf_esb_init( &addresses, NRF_PTX );
    
    debug_routine();
    init_timer3();
    #ifdef TRANSMITTER
    init_timer2();
    #endif
    tft_initialize();
}

static void debug_routine()
{
    static uint8_t volatile status, config;

    NRF_CS_PIN = 0;
    status = SPI_Read( 0x00 );
    config = SPI_Read( 0x00 );
    NRF_CS_PIN = 1;
}

void main() 
{
    uint8_t buffer[32];

    nrf_address_t rx_address;
    system_init();
    EnableInterrupts();
    
    while( 1 )
    {
         if( process_flag )
         {
             buffer[0] = nrf_recieve_data( &rx_address, &buffer[1] );
             process_flag = false;
         }
         
         if( update_time )
         {
         
         }
    }
}

#ifdef TRANSMITTER
void radio_ISR() iv IVT_INT_EXTI15_10 ics ICS_AUTO
#else
void radio_ISR() iv IVT_INT_EXTI1 ics ICS_AUTO
#endif
{
    EXTI_PR |= ( 1 << PR10 ); // Clear interrupt flag

    /**< Max retries interrupt */
    if( nrf_is_interrupted( NRF_MAX_RT ) )
    {
        lost_packets++;
    }

    /**< TX data sent interrupt */
    if( nrf_is_interrupted( NRF_TX_DS ) )
    {
        nrf_acknowledged();
        acknowledge_flag = true;
    }

    /**< RX data received interrupt */
    if( nrf_is_interrupted( NRF_RX_DR ) )
    {
        process_flag = true;
    }
}

void timer2_interrupt() iv IVT_INT_TIM2 
{
    static uint32_t count;
    
    TIM2_SR.UIF = 0;
    data_sent = count;
    nrf_send_data( 0, &( count++ ), sizeof( uint32_t ) );
    update_time = true;
}

void timer3_interrupt() iv IVT_INT_TIM3
{
    TIM3_SR.UIF = 0;
    nrf_timer_tick();
}