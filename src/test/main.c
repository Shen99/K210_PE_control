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

    int16_t i = 0;
    while(i < 16*3)
    {
        AD7606_trggier();
        int16_t idx = i%16;
        printf("%d %#08x, %f\n", idx, AD7606_rx_buf[idx], (int16_t)(AD7606_rx_buf[idx])/(float)32768.0f * 10);
        i++;
    }

    phase_change(2, 0.5f);
    duty_change(3, 0.0f);

    phase_change(2+8, 0.5f);

    float pwm1_duty = 0.1f;
    bool pwm3_duty = false;
    while (1)
    {
        duty_change(3, pwm3_duty);
        pwm3_duty = !pwm3_duty;
        duty_change(1, pwm1_duty);
        pwm1_duty += 0.1f;
        if (pwm1_duty > 1.0f)
        {
            pwm1_duty = 0.0f;
        }
        gpiohs_set_pin(PWM_DATA_SYNC_GPIO_NUM, GPIO_PV_LOW);
        pwm_update_non_blocking();
        gpiohs_set_pin(PWM_DATA_SYNC_GPIO_NUM, GPIO_PV_HIGH);
        usleep(50);
    }

    while(1)
    {
        continue;
    }
    return 0;
}
