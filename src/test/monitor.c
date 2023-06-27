#include <stdint.h>
#include <bsp.h>

#include "monitor.h"

volatile bool mtx; // used to sync multicore with data_lock
volatile bool data_lock = true; // used to sync core1 and core2 to print sample data waveform
volatile bool stop_transfer = false; // used to stop waveform transfer, due to overcurrent, ... issues
double sample_buf[SAMPLE_NUM][NUM_OF_MONITOR_CHANNEL]; // monitor data buffer
uint16_t sample_cnt = 0; // pointer of sample data buffer


void monitor_init()
{
    register_core1(core1_main, NULL);
}

// periodic send sample buff waveform data
int core1_main(void *ctx)
{
    while (true)
    {
        asm volatile("nop \n nop \n nop");
        if (!data_lock)
        {
            uint16_t temp_cnt = 0;
            while (temp_cnt < SAMPLE_NUM)
            {
                char str_buf[1000] = "$";
                char *str_buf_ptr = str_buf + 1;
                for (int i = 0; i < NUM_OF_MONITOR_CHANNEL; i++)
                {
                    str_buf_ptr += sprintf(str_buf_ptr, "%.3f|", sample_buf[temp_cnt][i]);
                }
                *(str_buf_ptr-1) = '\n';
                puts(str_buf);
                temp_cnt++;
            }
            data_lock = true;
        }
    }
    // it shouldn't run to there
    while (true)
    {
        ;
    }
    return 0;
}
