#ifndef LOOP_H
#define LOOP_H

#include "control.h"

#define CONTROL_TIMER_DEV_ID 0
#define CONTROL_TIMER_CHAN 0
#define CONTROL_TIMER_PERIOD (CONTROL_PERIOD*GHZ)

#define AD7606_TIMER_DEV_ID 0
#define AD7606_TIMER_CHAN 1
#define AD7606_TIMER_PERIOD (CONTROL_PERIOD*GHZ)


void control_loop_init();
void AD7606_loop_init();
void loop_init();
void loop_begin();

#endif