#include <stdio.h>
#include <unistd.h>
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"
#include <bsp.h>
#include <sysctl.h>
#include <syslog.h>
#include <timer.h>
#include <plic.h>

#include "pin_config.h"
#include "init.h"
#include "AD7606.h"
#include "PWM.h"
#include "loop.h"
#include "monitor.h"

int main(void)
{
    // set cpu freq to 520Mhz
    sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_FREQ);
    printf("Core 0 freq: %d\n", sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));

    // init IO power base on the board design
    io_set_power();
    dmac_init();
    plic_init();
    sysctl_enable_irq();

    AD7606_init();
    PWM_init();

    loop_init();
    // AD7606_loop_init();
    control_loop_init();
    loop_begin();

    monitor_init();

    int16_t i = 0;
    while(i < 16*10)
    {
        AD7606_trggier();
        // for (uint8_t k = 0; k < 16; k++)
        // {
        //     int16_t idx = i%16;
        //     printf("%d %#08x, %f\n", idx, AD7606_rx_buf[idx], (int16_t)(AD7606_rx_buf[idx])/(float)32768.0f * 10);
        //printf("INthemain\n");
        //     i++;
        // }
    }


    

    while(1)
    {
        continue;
    }
    return 0;
}
