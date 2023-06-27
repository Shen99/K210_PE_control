#ifndef MONITOR_H
#define MONITOR_H

#include "stdbool.h"

#define SAMPLE_NUM 400 // sample data buffer size
#define NUM_OF_MONITOR_CHANNEL 16

extern volatile bool mtx; // used to sync multicore with data_lock
extern volatile bool data_lock; // used to sync core1 and core2 to print sample data waveform
extern volatile bool stop_transfer; // used to stop waveform transfer, due to overcurrent, ... issues
extern double sample_buf[SAMPLE_NUM][NUM_OF_MONITOR_CHANNEL]; // monitor data buffer
extern uint16_t sample_cnt; // pointer of sample data buffer

void monitor_init();
int core1_main(void *ctx);

#endif