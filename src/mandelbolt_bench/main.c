/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

#define GPU_X 320
#define GPU_Y 240
#define XMIN (-2.0F)
#define XMAX (+1.0F)
#define ASPECT_RATIO ((GPU_X*1.0F)/(GPU_Y*1.0F))
#define XRANGE (XMAX-XMIN)
#define YRANGE (XRANGE/ASPECT_RATIO)
#define YMAX (YRANGE/2.0F)
#define YMIN (-YRANGE/2.0F)
#define XSTEP (XRANGE/(GPU_X*1.0F))
#define YSTEP (YRANGE/(GPU_Y*1.0F))

#define bool _Bool
#define true	1
#define false	0

void complex_mult(float* num1, float* num2, float *result)
{
    float result0 = (num1[0]*num2[0]-num1[1]*num2[1]);
    float result1 = (num1[0]*num2[1]+num1[1]*num2[0]);
    result[0] = result0;
    result[1] = result1;
}

bool mandelbrot_check(float* c)
{
    float z[2] = {0.0F, 0.0F};
    for (int i = 0; i < 4000; i++)
    {
        complex_mult(z, z, z);
        z[0] += c[0];
        z[1] += c[1];
        if ((z[0]*z[0] + z[1]*z[1]) > 4.0F)
        {
            return false;
        }
    }
    return true;
}
typedef int32_t fix15;
#define fix_float (15)
#define multfix15(a,b) ((fix15)(((int64_t)(a)*(int64_t)(b))>>fix_float))
#define divfix15(a,b) ((fix15)( ( ((int64_t)(a)) <<fix_float ) / (b) ))
#define int32_2fix15(a) ((fix15)((a)<<fix_float))
#define fix15_2int32(a) ((int)((a)>>fix_float))
#define float2fix15(a) ((fix15)((a)*(1<<fix_float)))
#define fix15_2float(a) ((float)(a)/(1<<fix_float))

void complex_mult_fix(fix15 *num1, fix15 *num2, fix15 *result)
{
    fix15 result0 = multfix15(num1[0], num2[0]) - multfix15(num1[1], num2[1]);
    fix15 result1 = multfix15(num1[0], num2[1]) + multfix15(num1[1], num2[0]);
    result[0] = result0;
    result[1] = result1;
}

bool mandelbrot_check_fix(fix15* c)
{
    fix15 z[2] = {0, 0};
    fix15 temp[2] = {0, 0};
    for (int i = 0; i < 4000; i++)
    {
        complex_mult_fix(z, z, z);
        z[0] += c[0];
        z[1] += c[1];
        if ( (multfix15(z[0], z[0]) + multfix15(z[1], z[1])) > int32_2fix15(4U) )
        {
            return false;
        }
    }
    return true;
}

float current_point[2] = {XMIN, YMAX};
fix15 current_point_fix[2] = {0, 0};

int core1_function(void *ctx)
{
    uint64_t orig = read_cycle();
    current_point[0] = XMIN;
    current_point[1] = 0;

    for (int i = 0; i < GPU_X; i++)
    {
        for (int j = GPU_Y/2; j < GPU_Y; j++)
        {
            if (mandelbrot_check(current_point))
            {
            }
            current_point[1] -= YSTEP;
        }
        current_point[1] = 0;
        current_point[0] += XSTEP;
    }
    uint64_t final = read_cycle();
    printf("cpu 1 done. %ld\n", final - orig);
    while(1);
}
uint32_t timer_get_count(timer_device_number_t timer_number, timer_channel_number_t channel);

int timer_callback(void *ctx) {
    return 0;
}

int main(void)
{
    // sysctl_cpu_set_freq(600000000);
    sysctl_pll_set_freq(SYSCTL_PLL0, 1100000000);
    uint64_t core = current_coreid();
    int data;
    printf("Core %ld Hello world\n", core);

    uint32_t core_freq = 0;
    core_freq = sysctl_clock_get_freq(SYSCTL_CLOCK_CPU);
    printf("Core freq: %d\n", core_freq);

    /* Clear stdin buffer before scanf */
    sys_stdin_flush();

    register_core1(core1_function, NULL);

    uint64_t orig = read_cycle();
    current_point_fix[0] = float2fix15(XMIN);
    current_point_fix[1] = float2fix15(YMAX);

    for (int i = 0; i < GPU_X; i++)
    {
        for (int j = 0; j < GPU_Y/2; j++)
        {
            if (mandelbrot_check_fix(current_point_fix))
            {
            }
            current_point_fix[1] -= float2fix15(YSTEP);
        }
        current_point_fix[1] = float2fix15(YMAX);
        current_point_fix[0] += float2fix15(XSTEP);
    }
    uint64_t final = read_cycle();

    printf("cpu 0 done. %ld\n", final - orig);
    while(1);
    return 0;
}
