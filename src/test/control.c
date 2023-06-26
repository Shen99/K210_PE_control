#include <stdio.h>
#include "stdbool.h"
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"

#include "pin_config.h"
#include "AD7606.h"

volatile bool control_heart_beat = false;

bool mtx; // used to sync multicore with data_lock
bool data_lock = true; // used to sync core1 and core2 to print sample data waveform
bool stop_transfer = false; // used to stop waveform transfer, due to overcurrent, ... issues
#define SAMPLE_NUM 400 // sample data buffer size
float sample_buf[SAMPLE_NUM][NUM_OF_AD7606_CHANNEL]; // sample data buffer
uint16_t sample_cnt = 0; // pointer of sample data buffer

float adc_buf[NUM_OF_AD7606_CHANNEL]; // after each sample, this buffer storage the real voltage

int control_loop(void *ctx)
{
    // static volatile bool first_time = true;
    // if (first_time)
    // {
    //     first_time = false;
    //     fpioa_set_function(CONTROL_HEART_BEAT, CONTROL_HEART_BEAT_FUNC);
    //     gpiohs_set_drive_mode(CONTROL_HEART_BEAT_GPIO_NUM, GPIO_DM_OUTPUT);
    //     gpiohs_set_pin(CONTROL_HEART_BEAT_GPIO_NUM, control_heart_beat);
    //     control_heart_beat = !control_heart_beat;
    //     return 0;
    // }

    // // control loop heart beat
    // gpiohs_set_pin(CONTROL_HEART_BEAT_GPIO_NUM, control_heart_beat);
    // control_heart_beat = !control_heart_beat;

    // AD7606 data acquisition
    // while(!AD7606_buf_ready){;}
    for (int i = 0; i < NUM_OF_AD7606_CHANNEL; i++)
    {
        adc_buf[i] = AD7606_buf[i];
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
    return 0;
}

// periodic send sample buff waveform data
void core1_main()
{
    while (true)
    {
        asm volatile("nop \n nop \n nop");
        if (!data_lock)
        {
            uint16_t temp_cnt = 0;
            while (temp_cnt < SAMPLE_NUM)
            {
                printf("$%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f\n", sample_buf[temp_cnt][0]
                                                    , sample_buf[temp_cnt][1]
                                                    , sample_buf[temp_cnt][2]
                                                    , sample_buf[temp_cnt][3]
                                                    , sample_buf[temp_cnt][4]
                                                    , sample_buf[temp_cnt][5]
                                                    , sample_buf[temp_cnt][6]
                                                    , sample_buf[temp_cnt][7]
                                                    , sample_buf[temp_cnt][8]
                                                    , sample_buf[temp_cnt][9]);
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
}
