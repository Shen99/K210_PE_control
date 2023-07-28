#ifndef CONTROL_H
#define CONTROL_H

#define KHZ (1000)
#define MHZ (1000000)
#define GHZ (1000000000)

#define CONTROL_FREQ (20*KHZ)
#define CONTROL_PERIOD (1.0f/CONTROL_FREQ)

//常量
 extern double  control_Ts ;  //控制周期
 extern double  L_value ;  //电感值
//指定输出直流
  extern double  udc_ref ;

//指定电流
  extern double  ik_ref_calcu ;
//交流输入幅值
  extern double  us_attitude ;


//开关管的通断时间
  extern  double G1 ;
  extern double G2 ;
  extern double G3 ;
  extern double G4 ;

//电容电压
 extern  double  meas_u1 ;
  extern double  meas_u2 ;
  extern double  meas_u3 ;
  extern double  meas_u4 ;
  extern double  meas_udc;

  extern double  meas_u1_last ;

  extern double  meas_u2_last ;

  extern double  meas_u3_last ;

  extern double  meas_u4_last ;


//交流电压 
  extern double  meas_us ;

//交流电流
  extern double  meas_ik ;

  extern double  meas_ik_abs;

//
    //补偿时间
    extern double tb ;
    extern double tc ;
    extern double rc1 ;
    extern double rc2 ;
    extern double rc3 ;
    extern double rc4 ;
extern double  compute_times;

int control_loop(void *ctx);

void get_ik_ref(double  us,double  udc);
void PWM_stop(void);
void MPCcontrol(double  ik_ref,double  u1,double  u2,double  u3,double  u4,double  us,double  ik,double  L,double  Ts,double  udc);

#endif