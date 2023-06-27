#include <stdio.h>
#include <bsp.h>
#include "stdbool.h"
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"

#include "pin_config.h"
#include "AD7606.h"
#include "PWM.h"
#include "monitor.h"
#include "control.h"

volatile bool control_heart_beat = false;

double adc_buf[NUM_OF_AD7606_CHANNEL]; // after each sample, this buffer storage the real voltage

float pwm1_duty = 0.1f;

int control_loop(void *ctx)
{
    pwm_update_non_blocking();

    static volatile bool first_time = true;
    if (first_time)
    {
        first_time = false;
        fpioa_set_function(CONTROL_HEART_BEAT, CONTROL_HEART_BEAT_FUNC);
        gpiohs_set_drive_mode(CONTROL_HEART_BEAT_GPIO_NUM, GPIO_DM_OUTPUT);
        gpiohs_set_pin(CONTROL_HEART_BEAT_GPIO_NUM, control_heart_beat);
        control_heart_beat = !control_heart_beat;
        return 0;
    }

    // control loop heart beat
    gpiohs_set_pin(CONTROL_HEART_BEAT_GPIO_NUM, control_heart_beat);
    control_heart_beat = !control_heart_beat;

    // AD7606 data acquisition
    // while(!AD7606_buf_ready){;}
    for (int i = 0; i < NUM_OF_AD7606_CHANNEL; i++)
    {
        adc_buf[i] = AD7606_buf[i];
    }

    duty_change(1, pwm1_duty);
    pwm1_duty += 0.1f;
    if (pwm1_duty > 1.0f)
    {
        pwm1_duty = 0.0f;
    }

    if (data_lock && sample_cnt != SAMPLE_NUM)
    {
        for (int i = 0; i < NUM_OF_AD7606_CHANNEL; i++)
        {
            sample_buf[sample_cnt][i] = adc_buf[i];
        }
        sample_cnt++;
    }

    if (sample_cnt == SAMPLE_NUM)
    {
        if (stop_transfer)
        {
            sample_cnt = 0;
            return 0;
        }
        data_lock = false;
        sample_cnt = 0;
    }
    usleep(30);
    return 0;
}
