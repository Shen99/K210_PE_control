#include <sysctl.h>
#include <timer.h>
#include <stdio.h>
#include "utils.h"

#include "loop.h"
#include "AD7606.h"

void loop_init()
{
    timer_init(CONTROL_TIMER_DEV_ID);
    timer_init(AD7606_TIMER_DEV_ID);
}

void control_loop_init()
{
    size_t real_interval = timer_set_interval(CONTROL_TIMER_DEV_ID, CONTROL_TIMER_CHAN, CONTROL_TIMER_PERIOD);
    printf("timer 0 period: %ldns\n", real_interval);
    timer_irq_register(CONTROL_TIMER_DEV_ID, CONTROL_TIMER_CHAN, 0, 1, control_loop, NULL);
    timer_set_enable(CONTROL_TIMER_DEV_ID, CONTROL_TIMER_CHAN, 1);
}

void AD7606_loop_init()
{
    size_t real_interval = timer_set_interval(AD7606_TIMER_DEV_ID, AD7606_TIMER_CHAN, AD7606_TIMER_PERIOD);
    printf("timer 0 period: %ldns\n", real_interval);
    timer_irq_register(AD7606_TIMER_DEV_ID, AD7606_TIMER_CHAN, 0, 1, AD7606_trggier_non_blocking, NULL);
    timer_set_enable(AD7606_TIMER_DEV_ID, AD7606_TIMER_CHAN, 1);
}

void loop_begin()
{
    ;
}