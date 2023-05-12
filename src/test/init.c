#include "pin_config.h"
#include "init.h"

void io_set_power(void)
{
    sysctl_set_power_mode(SYSCTL_POWER_BANK0, POWER_BANK0_SELECT);
    sysctl_set_power_mode(SYSCTL_POWER_BANK1, POWER_BANK1_SELECT);
    sysctl_set_power_mode(SYSCTL_POWER_BANK2, POWER_BANK2_SELECT);
    sysctl_set_power_mode(SYSCTL_POWER_BANK3, POWER_BANK3_SELECT);
    sysctl_set_power_mode(SYSCTL_POWER_BANK4, POWER_BANK4_SELECT);
    sysctl_set_power_mode(SYSCTL_POWER_BANK5, POWER_BANK5_SELECT);
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, POWER_BANK6_SELECT);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, POWER_BANK7_SELECT);
}

void AD7606_init(void)
{
    fpioa_set_function(AD7606_SCLK, FUNC_SPI0_SCLK);
    fpioa_set_function(AD7606_CS, FUNC_SPI0_SS0);
    fpioa_set_function(AD7606_MISO, FUNC_SPI0_D0);
    fpioa_set_function(AD7606_CONVAB, FUNC_GPIOHS0 + AD7606_CONVAB_GPIO_NUM);
    fpioa_set_function(AD7606_BUSY, FUNC_GPIOHS0 + AD7606_BUSY_GPIO_NUM);
    fpioa_set_function(AD7606_RESET, FUNC_GPIOHS0 + AD7606_RESET_GPIO_NUM);

    spi_init(AD7606_SPI, SPI_WORK_MODE_2, SPI_FF_STANDARD, 16, 0);
    uint32_t spi_rate = spi_set_clk_rate(AD7606_SPI, AD7606_SPI_RATE);
    printf("AD7606 spi freq: %u\n", spi_rate);
}
