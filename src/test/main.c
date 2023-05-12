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

//TODO: two-wire spi test
// Maybe the reset and busy can be omitted.

int main(void)
{
    // set cpu freq to 540Mhz
    sysctl_pll_set_freq(SYSCTL_PLL0, 800000000);
    printf("Core 0 freq: %d\n", sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));

    // init IO power base on the board design
    io_set_power();
    AD7606_init();
    int16_t spi_buf[8];
    int16_t i = 0;

    while(i < 80)
    {
        gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_LOW);
        usleep(4);
        gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_HIGH);
        usleep(8);
        spi_receive_data_standard(AD7606_SPI, AD7606_SPI_CHIP_SELECT, NULL, 0, (uint8_t *)spi_buf, 16);
        printf("%d %#08x, %f\n", i%8, spi_buf[i%8], (int16_t)(spi_buf[i%8])/(float)32768.0f * 10);
        i++;
    }

    while(1)
    {
        continue;
    }
    return 0;
}
