#ifndef MPCdouble_H
#define MPCdouble_H

#include <stdio.h>
#include "sysctl.h"
#include "bsp.h"
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"
#include "spi.h"
#include "pin_config.h"

//控制周期
extern float control_Ts = 1.0f/20000;

//指定输出直流
extern float udc_ref = 60.0f;
//指定电流
ik_ref_calcu= 0.0f;
//交流输入幅值
extern float us_attitude = 331.0f;

//开关管的通断时间
extern float G1 = 0;
extern float G2 = 0;
extern float G3 = 0;
extern float G4 = 0;
//电容电压
extern float meas_u1 = 0;
extern float meas_u2 = 0;
extern float meas_u3 = 0;
extern float meas_u4 = 0;

extern float meas_u1_last = 0;
extern float meas_u2_last = 0;
extern float meas_u3_last = 0;
extern float meas_u4_last = 0;

//交流电压 
extern float meas_us = 0;
//交流电流
extern float meas_ik = 0;
//
void get_ik_ref(float us,float udc);
void MPCcontrol(float ik_ref,float u1,float u2,float u3,float u4,float us,float ik,float L,float Ts,float udc,float i1,float i2,float i3,float i4);

#endif