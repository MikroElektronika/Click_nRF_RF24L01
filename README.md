![mikroe_logo] 
# by [MikroElektronika]
![nrf_image] 
# 2.4Ghz Nordic nRF24L01 radio transceiver for small and energy efficient devices.
# More information about [nRF click] is found here.
---
## Installation
>If installing from package, [Package manager] required to install to your IDE.  

## Looking for a [tutorial?][nRF Tutorial]

### Example
```
static const nrf_addr_map_t addresses = {
    {0xB3, 0xB4, 0xB5, 0xB6, 0xF1}, //RX PIPE 0
    {0},                            //RX PIPE 1
    {0},                            //RX PIPE 2
    {0},                            //RX PIPE 3
    {0},                            //RX PIPE 4
    {0},                            //RX PIPE 5
    {0xB3, 0xB4, 0xB5, 0xB6, 0xF1}  //TX PIPE
};

static void system_init()
{
    ... // initialize peripherals
    nrf_esb_init( &addresses, NRF_PTX );
  
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
    }
}


void radio_ISR() iv IVT_INT_EXTI15_10 ics ICS_AUTO
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

void timer3_interrupt() iv IVT_INT_TIM3
{
    TIM3_SR.UIF = 0;
    nrf_timer_tick();
}
```

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [MikroElektronika]: <http://www.mikroe.com/>
   [mikroe_logo]: <http://www.mikroe.com/img/designs/beta/logo_small.png>
   [nRF click]: <http://www.mikroe.com/click/nrf-c/>
   [nrf_image]: <http://www.mikroe.com/img/development-tools/accessory-boards/click/nrf-c/nrf_c_click_main.png>
   [nRF Tutorial]: <http://learn.mikroe.com/>
   [Package Manager]: <http://www.mikroe.com/package-manager/>

