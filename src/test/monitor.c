#include <stdint.h>
#include <bsp.h>

#include "monitor.h"
#include "control.h"

volatile bool mtx; // used to sync multicore with data_lock
volatile bool data_lock = true; // used to sync core1 and core2 to print sample data waveform
volatile bool stop_transfer = false; // used to stop waveform transfer, due to overcurrent, ... issues
double sample_buf[SAMPLE_NUM][NUM_OF_MONITOR_CHANNEL]; // monitor data buffer
uint16_t sample_cnt = 0; // pointer of sample data buffer
uint16_t screen_send_flag = 0;

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
                    str_buf_ptr += sprintf(str_buf_ptr, "%.4f|**|", sample_buf[temp_cnt][i]);
                }
                *(str_buf_ptr-1) = '\n';
                //puts(str_buf);
                printf("meas_u1 %.2f *|meas_u2 %.2f *||meas_u3 %.2f *||meas_u4 %.2f *||meas_ik %.2f *||meas_dc %.2f*||G1 %.12f*||G2 %.2f*||G3 %.2f *||G4 %.2f*|*Tc:%.2f*|*Tb:%.2f*|rc:%.12f\n",meas_u1,meas_u2,meas_u3,meas_u4,meas_ik,meas_udc,G1,G2/control_Ts,G3/control_Ts,G4/control_Ts,tc,tb/control_Ts,rc1);
                //printf("meas_u1 %.2f *|meas_u2 %.2f *||meas_u3 %.2f *||meas_u4 %.2f *||meas_ik %.2f *||meas_us %.2f*||*Tc:%.2f*|*Tb:%.2f\n",meas_u1,meas_u2,meas_u3,meas_u4,meas_ik,meas_us,(float)tc,(float)tb);
                /*
                screen_send_flag += 1;
                if (screen_send_flag > 10)
                {
                    printf("add,screenshow %f\n", compute_times);
                    printf("add 1,0,%d\n",(int)(meas_u1*0.2)); //输出C1电压
                    printf("add 1,1,%f \n",meas_u2); //输出C2电压
                    printf("Voltage1.val=%d\xff\xff\xff",(int)(meas_u2)); //输出C2电压
                    printf("Voltage2.val=%d\xff\xff\xff",(int)(meas_u3)); //输出C3电压
                    printf("Current1.val=%d\xff\xff\xff",(int)(meas_ik)); //输出AC current
                    screen_send_flag = 0;
                }
                */
                //printf("TIMES:,%f\n",compute_times);
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
