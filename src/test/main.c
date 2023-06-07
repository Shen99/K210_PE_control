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

//TODO: two-wire spi test
// Maybe the reset and busy can be omitted.

int main(void)
{
    // set cpu freq to 540Mhz
    sysctl_pll_set_freq(SYSCTL_PLL0, 1040 * MHZ);
    printf("Core 0 freq: %d\n", sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));

    // init IO power base on the board design
    io_set_power();
    // AD7606_init();
    PWM_init();
    // int16_t spi_buf[16];
    // int16_t i = 0;

    // while(i < 16*3)
    // {
    //     gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_LOW);
    //     usleep(4);
    //     gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_HIGH);
    //     usleep(8);
    //     spi_receive_data_standard(AD7606_SPI, SPI_CHIP_SELECT_0, NULL, 0, (uint8_t *)spi_buf, 16);
    //     spi_receive_data_standard(AD7606_SPI, SPI_CHIP_SELECT_1, NULL, 0, (uint8_t *)(spi_buf+8), 16);
    //     int16_t idx = i%16;
    //     printf("%d %#08x, %f\n", idx, spi_buf[idx], (int16_t)(spi_buf[idx])/(float)32768.0f * 10);
    //     i++;
    // }

    uint8_t pwm_txf[5*NUM_OF_PWM_CHANNEL] = {0, 0, 0, 0, 0x0U};
    for (int i = 0; i < 5*NUM_OF_PWM_CHANNEL; i++)
    {
        pwm_txf[i] = 0;
    }
    for (int i = 0; i < NUM_OF_PWM_CHANNEL; i++)
    {
        pwm_dev_addr(pwm_txf, i, 0);
        duty_change(pwm_txf, i, 0.1f);
        phase_change(pwm_txf, i, 1.0f);
    }
    for (int i = 8; i < NUM_OF_PWM_CHANNEL; i++)
    {
        pwm_dev_addr(pwm_txf, i, 1);
    }
    phase_change(pwm_txf, 2, 0.5f);
    duty_change(pwm_txf, 3, 0.0f);

    phase_change(pwm_txf, 2+8, 0.5f);

    uint32_t count = 0;
    float pwm1_duty = 0.1f;
    bool pwm3_duty = false;
    while (1)
    {
        if (count % 1000 == 0) {
            duty_change(pwm_txf, 3, pwm3_duty);
            pwm3_duty = !pwm3_duty;
        }
        duty_change(pwm_txf, 1, pwm1_duty);
        pwm1_duty += 0.1f;
        if (pwm1_duty > 1.0f)
        {
            pwm1_duty = 0.0f;
        }
        gpiohs_set_pin(PWM_DATA_SYNC_GPIO_NUM, GPIO_PV_LOW);
        spi_send_data(pwm_txf, 5*NUM_OF_PWM_CHANNEL);
        gpiohs_set_pin(PWM_DATA_SYNC_GPIO_NUM, GPIO_PV_HIGH);
        count++;
        usleep(50);
    }

    while(1)
    {
        continue;
    }
    return 0;
}
