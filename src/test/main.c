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

/**
* Function       timer_timeout_cb
* @author        Yucheng
* @date          2023.06.24
* @brief         定时器中断回调
* @param[in]     ctx
* @param[out]    void
*/
int timer_timeout_cb(void *ctx) {
    uint32_t *tmp = (uint32_t *)(ctx);
    (*tmp)++;
    if ((*tmp)%2)
    {
        rgb_all_on();
    }
    else
    {
        rgb_all_off();
    }
    return 0;
}


/**
* Function       init_timer
* @author        Yucheng
* @date          2023.06.24
* @brief         初始化定时器
初始化定时器，这里使用的是定时器 0 通道 0 ，超时时间为 500 毫秒，定
时器中断回调函数为 timer_timeout_cb ，参数为 g_count 。
* @param[in]     ctx
* @param[out]    void
*/


uint32_t g_count; //不理解这个的作用 2023年6月24日 23:26:30
//q:what's the meaning of g_count
//a:it's a global variable, and it's used to count the number of timer interrupt
//q:why we need to count the number of timer interrupt
//a:because we need to do something in the timer interrupt, and we need to know how many times the timer interrupt has been triggered
//q:what's the meaning of ctx
//a:ctx is a pointer to the g_count

void init_timer(void) {
    /* 定时器初始化 */
    timer_init(TIMER_DEVICE_0);
    /* 设置定时器超时时间，单位为ns */
    timer_set_interval(TIMER_DEVICE_0, TIMER_CHANNEL_0, 500 * 1e6);
    /* 设置定时器中断回调 */
    timer_irq_register(TIMER_DEVICE_0, TIMER_CHANNEL_0, 0, 1, timer_timeout_cb, &g_count);
    /* 使能定时器 */
    timer_set_enable(TIMER_DEVICE_0, TIMER_CHANNEL_0, 1);
}


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

