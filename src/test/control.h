#ifndef CONTROL_H
#define CONTROL_H

#define KHZ (1000)
#define MHZ (1000000)
#define GHZ (1000000000)

#define CONTROL_FREQ (20*KHZ)
#define CONTROL_PERIOD (1.0f/CONTROL_FREQ)

//常量
 extern float control_Ts ;  //控制周期
 extern float L_value ;  //电感值
//指定输出直流
  extern float udc_ref ;

//指定电流
  extern float ik_ref_calcu ;
//交流输入幅值
  extern float us_attitude ;


//开关管的通断时间
 extern  float G1 ;
  extern float G2 ;
  extern float G3 ;
  extern float G4 ;

//电容电压
 extern  float meas_u1 ;
  extern float meas_u2 ;
  extern float meas_u3 ;
  extern float meas_u4 ;

  extern float meas_u1_last ;

  extern float meas_u2_last ;

  extern float meas_u3_last ;

  extern float meas_u4_last ;


//交流电压 
  extern float meas_us ;

//交流电流
  extern float meas_ik ;

  extern float meas_ik_abs;

//
extern float compute_times;

int control_loop(void *ctx);

void get_ik_ref(float us,float udc);
void MPCcontrol(float ik_ref,float u1,float u2,float u3,float u4,float us,float ik,float L,float Ts,float udc);

#endif