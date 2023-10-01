#include <stdio.h>
#include <bsp.h>
#include "stdbool.h"
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"

#include "pin_config.h"
#include "AD7606.h"
#include "PWM.h"
#include "monitor.h"
#include "control.h"
#include <math.h>



volatile bool control_heart_beat = false;
double adc_buf[NUM_OF_AD7606_CHANNEL]; // after each sample, this buffer storage the real voltage
double show[16]; // 串口打印输出的数据

double pwm1_duty = 0.1f;
double PIDout = 1.0f;

//常量
 double control_Ts = 1.0f/20000;  //控制周期
 double L_value = 0.005f;  //电感值
 double Co = 0.002;  //Co 电容值 2000uf
 double Cf = 0.00047; //飞跨电容值 470uf
//指定输出直流
 double udc_ref = 30.00f;  //<<<<<<<<<<<<<<<<<<<<<<<<<直流侧输出电压<<<<<<<<<<<<<<<<<<<<<<<<<<
 double Ucf_ref = 7.5f; //飞跨电容的指定电压

//指定电流
 double ik_ref_calcu = 0.0f;
//交流输入幅值peak
 double us_attitude = 18.0f * 1.414f;


//开关管的通断时间
 double G1 = 0;
 double G2 = 0;
 double G3 = 0;
 double G4 = 0;
 double G1_last = 0;
 double G2_last = 0;
 double G3_last = 0;
 double G4_last = 0;

//交流电压 
 double meas_us= 0.1f; //7606通道一》》》》交流电压

//交流电压的预测值，这里直接用当前的值，如需预测，可能需要加入锁相环移动固定相位
double us_k1=0.1;
double us_k2=0.1;

//交流电流
 double meas_ik= 0.1f;//7606通道二》》》》电流
 double  meas_ik_abs= 0.1f;

//电容电压
 double meas_u1 = 0.1f; //7606通道三》》》》C1电压 
 double meas_u2 = 0.1f;//7606通道四》》》》C2电压
 double meas_u3 = 0.1f;//7606通道五》》》》C3电压
 double meas_u4 = 0.1f;//7606通道六》》》》C4电压
 double meas_udc = 0.2f;
 double meas_io = 0.1f;//通道** 》》》输出端电流
 double meas_u1_last = 0.1f;
 double meas_u2_last = 0.1f;
 double meas_u3_last = 0.1f;
 double meas_u4_last = 0.1f;

    //补偿时间
    double t_balance_cf = 0;
    double t_balance_co = 0;
    double t_ref_cf = 0;





//
//write a function to get a absolute value of a float number
double  abs_float(double a)
{
    if(a >= 0)
    {
        return a;
    }
    else
    {
        return -a;
    }
}

struct PID
{
    double reference;
    double p;
    double i;
    double integrate;
    double upper_limit;
    double lower_limit;
    double out;
    void (*PID_compute)(struct PID *pid_instance, float in);
    void (*PID_reset)(struct PID *pid_instance);
};

void PID_compute(struct PID *pid_instance, float in);
void PID_reset(struct PID *pid_instance);

//PIPIPIPIPIPIPIPI
struct PID Iref_PID = {
    .reference = 0,
    .p = 0.000005,
    .i = 0.00005, 
    .integrate = 0,
    .upper_limit = 30,
    .lower_limit = -30,
    .out = 0,
    .PID_compute = &PID_compute,
    .PID_reset = &PID_reset
};
//PIPIPIPIPIPIPIPI


void PID_compute(struct PID *pid_instance, float in)
{
    double diff = pid_instance->reference - in;
    double proportion = pid_instance->p * diff;

    if (((pid_instance->out == pid_instance->upper_limit) && diff > 0) ||
        ((pid_instance->out == pid_instance->lower_limit) && diff < 0))
    {
        ;
    }else{
        pid_instance->integrate += pid_instance->i * diff;
    }
    pid_instance->out = pid_instance->integrate + proportion;
    pid_instance->out = (pid_instance->out >= pid_instance->upper_limit) ? pid_instance->upper_limit : pid_instance->out;
    pid_instance->out = (pid_instance->out <= pid_instance->lower_limit) ? pid_instance->lower_limit : pid_instance->out;
    PIDout = pid_instance->out;
}

void PID_reset(struct PID *pid_instance)
{
    pid_instance->out = 0;
    pid_instance->integrate = 0;    
    PIDout = pid_instance->out;
}


//get ik_ref according to the meas_us and udc_ref and then through the PID controller to get the ik_ref
//input:mean_us,udc_ref
//to change the globle variable ik_ref_calcu
void get_ik_ref(double us,double udc)
{
    //double udc_err = udc_ref - udc;
    Iref_PID.reference = udc_ref;
    (Iref_PID.PID_compute)(&Iref_PID, meas_udc);   
    ik_ref_calcu = abs_float(us / us_attitude * Iref_PID.out);
}



  //  (Iref_PID.PID_compute)(&Iref_PID, udc_err);
   // Iref_PID.out;

/**
* Function       MPCcontrol
* @author        Yucheng
* @date          2023.06.24
* @brief         mpc control five level rectifier
* @param[in]     ik_ref,u1,u2,u3,u4,us,ik,L,Ts,udc
* @param[out]    G1,G2,G3,G4
*/

void MPCcontrol(double ik_ref,double Uco1,double Uco2,double Ucf1,double Ucf2,double us,double ik,double L,double Ts,double udc,double io)
{
    G1=0; 
    G2=0; 
    G3=0; 
    G4=0; 

    double iakk[14]= {0};

    double vec_0 = 0;
    double vec_1 = 0;
    double vec_2 = 0;
    double vec_3 = 0;
    double vec_4 = 0;
    us_k1=us;
    us_k2=us;
    Ucf_ref=udc/4;

    double VL_k1=0; //根据上一时刻计算出K+1时刻的电感电压
    VL_k1=(Ucf1*(G1_last-G2_last)+Uco1*(G2_last-Ts)+Uco2*(G3_last-Ts)+Ucf2*(G4_last-G3_last))/Ts+us_k1;
    double I_k1=ik+Ts*VL_k1/L; //K+1时刻的L电流
    //compute the K+1 time every C's voltage
    double Icf1_k1=I_k1*(G2_last-G1_last)/Ts;
    double Icf2_k1=I_k1*(G3_last-G4_last)/Ts;
    double Ico1_k1=I_k1*(Ts-G2_last)/Ts;
    double Ico2_k1=I_k1*(Ts-G3_last)/Ts;
    
    double Ucf1_k1=Icf1_k1*Ts/Cf+Ucf1;
    double Ucf2_k1=Icf2_k1*Ts/Cf+Ucf2;
    double Uco1_k1=Ico1_k1*Ts/Co+Uco1;
    double Uco2_k1=Ico2_k1*Ts/Co+Uco2; //待优化
    //2023.9.25引入等效直流侧电容电压Udceq_k1，在的一定程度上优化电容电压偏离目标值时
    //用直流电压不准确，可能会选错电流矢量的问题
    double Udceq_k1=(Ucf1_k1+Ucf2_k1+Uco1_k1+Uco2_k1)*2/3;
    // 每一组矢量在电感电流上产生的斜率
    vec_0=(us_k2-Udceq_k1)/L;    //0000
    vec_4=(us_k2)/L;       //1111
    vec_3=(us_k2-((Udceq_k1)*0.25))/L;    //1110  1101  0111  1011
    vec_2=(us_k2-((Udceq_k1)*0.5))/L;    //1010  1001  0011  1100 0110  0101
    vec_1=(us_k2-((Udceq_k1)*0.75))/L;        //0100  1000  0010  00011
    
    //计算两组矢量的作用时间
    double tp1=0;
    double tp2=0;
    double tp3=0;
    double tp4=0;

    tp1=(ik_ref-I_k1-vec_0*Ts)/(vec_1-vec_0);  //A Vec_com1 V0+V1   3udc/4和udc向量1的作用时间  开关状态是几个1就是向量几
    tp2=(ik_ref-I_k1-vec_1*Ts)/(vec_2-vec_1);   //B Vec_com1 V1+V2 udc/2和 3udc/4 向量2的作用时间
    tp3=(ik_ref-I_k1-vec_2*Ts)/(vec_3-vec_2);   //C Vec_com1 V2+V3 udc/4和udc/2 向量3的作用时间
    tp4=(ik_ref-I_k1-vec_3*Ts)/(vec_4-vec_3);  //D Vec_com1 V3+V4 0和udc/4 向量4的作用时间
    
    //Limit amplitude
    if (tp1>Ts) tp1=Ts*0.999;
    if (tp1<0) tp1=Ts*0.001;
    if (tp2>Ts) tp2=Ts*0.999;
    if (tp2<0) tp2=Ts*0.001;
    if (tp3>Ts) tp3=Ts*0.999;
    if (tp3<0) tp3=Ts*0.001;
    if (tp4>Ts) tp4=Ts*0.999;
    if (tp4<0) tp4=Ts*0.001;

    //预测电流
    double A=0;double B=0;double C=0;double D=0;

    A=I_k1+vec_1*tp1+vec_0*(Ts-tp1);  //图A
    B=I_k1+vec_2*tp2+vec_1*(Ts-tp2); //图B
    C=I_k1+vec_3*tp3+vec_2*(Ts-tp3);  //图C
    D=I_k1+vec_4*tp4+vec_3*(Ts-tp4);  // 图D

    double I_pre_A=A;
    double I_pre_B=B;
    double I_pre_C=C;
    double I_pre_D=D;
    //%******计算偏差取最小值******start
    A=(A-ik_ref)*(A-ik_ref);
    B=(B-ik_ref)*(B-ik_ref);
    C=(C-ik_ref)*(C-ik_ref);
    D=(D-ik_ref)*(D-ik_ref);
    
    iakk[1]=A;
    iakk[2]=B;
    iakk[3]=C;
    iakk[4]=D;
    
    double min = iakk[1];
        uint32_t i;
    for (i=0;i<4;i++)
    {
        if(min>iakk[i])
        {
            min =  iakk[i];
        }

    }
    double I_k2=I_k1;
    //注意数组编号
    //iakk[0]= min;
if (min == A) {
       I_k2 = I_pre_A;
   }

   if (min == B) {
       I_k2 = I_pre_B;
   }

   if (min == C) {
       I_k2 = I_pre_C;
   }

   if (min == D) {
       I_k2 = I_pre_D;
   }

    //补偿时间
    double Icf_k2=I_k2; //流过飞跨电容的电流等于电感电流
    double Ico_k2=I_k2-io; //流过直流侧电容的电流等于给电容充电的电感电流-给直流侧供电的电流负载不平衡时应该继续优化
    //调节系数
    float k_u_cc=0.99;

    if(min==A) //vec_1 vec_0
    {
    t_ref_cf = (2*Ucf_ref - Ucf1_k1 - Ucf2_k1)/(Icf_k2/Cf);
    t_balance_co = (Uco1_k1 - Uco2_k1)/(Ico_k2/Co);
    //限制补偿时间的范围
    if(tp1>Ts/2) //需要根据tp的计算值来选,防止出现其他开关情况
    {
        if(t_ref_cf > ((Ts-tp1)*0.25))
            t_ref_cf = (Ts-tp1)*0.25;
        
        if(t_ref_cf < -((Ts-tp1)*0.25))
            t_ref_cf = -(Ts-tp1)*0.25;
        
        if(t_balance_co > ((Ts-tp1)*0.25))
            t_balance_co = (Ts-tp1)*0.25;
        
        if(t_balance_co < -((Ts-tp1)*0.25))
            t_balance_co = -(Ts-tp1)*0.25;
        
    }
    if(tp1<=Ts/2)
     {
        if(t_ref_cf > (tp1*0.25))
            t_ref_cf = tp1*0.25;
        
        if(t_ref_cf < -tp1*0.25)
            t_ref_cf = -tp1*0.25;
        
        if(t_balance_co > tp1*0.25)
            t_balance_co = tp1*0.25;
        
        if(t_balance_co < -tp1*0.25)
            t_balance_co = -tp1*0.25;
        
         }
    
        
    G1=tp1*0.25-0.25*t_balance_co*k_u_cc-t_ref_cf*0.5*k_u_cc;
    G2=tp1*0.25-0.25*t_balance_co*k_u_cc;
    G3=tp1*0.25+0.25*t_balance_co+t_ref_cf*0.5*k_u_cc;
    G4=tp1*0.25+0.25*t_balance_co*k_u_cc;
    }
    else if(min==B)  //vec_2 vec_1
        {
            G1=0.5*tp2+0.25*(Ts-tp2);  
            G2=0.5*tp2+0.25*(Ts-tp2);
            G3=0.5*tp2+0.25*(Ts-tp2);
            G4=0.5*tp2+0.25*(Ts-tp2);
        }
                
    
        //CCCCCCCCCCCCCCCCCCCCCCCC
    else if(min==C)  //vec_3 vec_2
        {
                G1=0.75*tp3+0.5*(Ts-tp3);
                G2=0.75*tp3+0.5*(Ts-tp3);
                G3=0.75*tp3+0.5*(Ts-tp3);
                G4=0.75*tp3+0.5*(Ts-tp3);
        }
       else if(min==D)  //vec_4 vec_3
       {
        //限制补偿时间的范围
        t_balance_cf = (Ucf1_k1-Ucf2_k1)/(Icf_k2/Cf);
        t_balance_co = (Uco1_k1-Uco2_k1)/(Ico_k2/Co);
        //限幅 :补偿时间不能超过全1矢量(V4)的四分之一,com_V4又需要两个补偿结合,
        // 所以此处t_b/2取最大范围的一半,即1/8tp,t_b限制到1/4
        if(t_balance_cf > 0.25 * tp4)
            t_balance_cf=0.25 * tp4;
        
        if(t_balance_cf < -0.25 * tp4)
            t_balance_cf = -0.25 * tp4;
        
        if(t_balance_co > 0.25 * tp4)
            t_balance_co=0.25 * tp4;
        
        if(t_balance_co < -0.25 * tp4)
            t_balance_co = -0.25 * tp4;

        G1=Ts-0.25*(Ts-tp4)+t_balance_cf*0.5*k_u_cc+t_balance_co*0.5*k_u_cc;
        G2=Ts-0.25*(Ts-tp4)+t_balance_co*0.5*k_u_cc;
        G3=Ts-0.25*(Ts-tp4)-t_balance_co*0.5*k_u_cc;
        G4=Ts-0.25*(Ts-tp4)-t_balance_cf*0.5*k_u_cc-t_balance_co*0.5*k_u_cc;
       }

    //output PWM
    G1_last=G1;
    G2_last=G2;
    G3_last=G3;
    G4_last=G4;
    phase_change(0,0);
    duty_change(0,G1/control_Ts);
    phase_change(1,0.25f);
    duty_change(1,G2/control_Ts);
    phase_change(2,0.5f);
    duty_change(2,G3/control_Ts);
    phase_change(3,0.75f);
    duty_change(3,G4/control_Ts);
    
    //update the last meas_us
   // meas_u1_last=u1;
   // meas_u2_last=u2;
   // meas_u3_last=u3;
   // meas_u4_last=u4;

    //串口打印输出数组赋值

    show[0]=meas_u1;
    show[1]=meas_u2;
    show[2]=meas_u3;
    show[3]=meas_u4;

    show[4]=meas_ik;
    show[5]=meas_udc;
    show[6]=G1/control_Ts;
    show[7]=G2/control_Ts;
    show[8]=G3/control_Ts;
    show[9]=G4/control_Ts;
    show[10]=t_ref_cf/control_Ts;
    show[11]=t_balance_co/control_Ts;
    show[12]=PIDout;
    show[13]=ik_ref_calcu;

}

void PWM_stop(void)
{
    G1=0;
    G2=0;
    G3=0;
    G4=0;
    duty_change(0,G1/control_Ts);
    duty_change(1,G2/control_Ts);
    duty_change(2,G3/control_Ts);
    duty_change(3,G4/control_Ts);
    /*把show数组全部赋值为零
    */

    show[0]=meas_u1;
    show[1]=meas_u2;
    show[2]=meas_u3;
    show[3]=meas_u4;

    show[4]=meas_ik;
    show[5]=meas_udc;
    show[6]=G1/control_Ts;
    show[7]=G2/control_Ts;
    show[8]=G3/control_Ts;
    show[9]=G4/control_Ts;
    show[10]=t_ref_cf/control_Ts;
    show[11]=t_balance_co/control_Ts;
    show[12]=PIDout;
    show[13]=ik_ref_calcu;

}
/****************************************************
 * 
 * Control Loop           ***************************
 * 
 * use MPC to control
 * control F = 20khz
 *                        ***************************
 * By Yucheng
 *                        ***************************

 *                                             *****/

int control_loop(void *ctx)
{
    pwm_update_non_blocking();

    static volatile bool first_time = true;
    if (first_time)
    {
        first_time = false;
        fpioa_set_function(CONTROL_HEART_BEAT, CONTROL_HEART_BEAT_FUNC);
        gpiohs_set_drive_mode(CONTROL_HEART_BEAT_GPIO_NUM, GPIO_DM_OUTPUT);
        gpiohs_set_pin(CONTROL_HEART_BEAT_GPIO_NUM, control_heart_beat);
        control_heart_beat = !control_heart_beat;
        return 0;
    }

    // control loop heart beat
    gpiohs_set_pin(CONTROL_HEART_BEAT_GPIO_NUM, control_heart_beat); //Change the heart beat
    control_heart_beat = !control_heart_beat;

    pwm1_duty += 0.1f;
    //compute_times += 1.0f;
    if (pwm1_duty > 1.0f)
    {
        pwm1_duty = 0.0f;
    }
    /*
    if (compute_times > 9000.0f && screen_send_flag<11000.f)
    {
        screen_send_flag = true;
    }
    
    if (compute_times > 20000.0f)
    {
        compute_times = 0.0f;
    }
    */
    //TEST PWM pin 4-5-6-7
    duty_change(7, pwm1_duty);
    //
    phase_change(7, 0.0f);
    duty_change(6,0.25f);
    phase_change(6,0.25f);
   
    // AD7606 data acquisition
    // while(!AD7606_buf_ready){;}
    for (int i = 0; i < NUM_OF_AD7606_CHANNEL; i++)
    {
        adc_buf[i] = AD7606_buf[i];
    }

    //measure data

    meas_us = adc_buf[2]*0.1139f-0.029f; //7606通道三》》》》交流电压
    // 茶花传感器系数 meas_ik = 0.0051f*adc_buf[0]-0.1995f;                              //7606通道一》》》》交流电流
    //
    meas_ik = 0.0053*adc_buf[0]-10.463;                              //7606通道一》》》》交流电流(AMC1301)
    meas_ik_abs = abs_float(meas_ik);
    meas_u1 = adc_buf[7]*0.0363f+0.0143f;                             //7606通道8》》》》1
    meas_u2 = adc_buf[6]*0.0363f+0.1177f;                              //7606通道7》》》》2
    meas_u3 = adc_buf[4]*0.0362f-0.2621f;                              //7606通道5》》》》3
    meas_u4 = adc_buf[5]*0.0361f+0.12f;                              //7606通道6》》》》4
    meas_io = 0.0053*adc_buf[1]-10.463;                              //7606通道2  待添加 需校准
    //


    meas_udc = meas_u1 + meas_u2;                                    //直流输出电压

  

    if (meas_u1 > udc_ref/4 && meas_u2 > udc_ref/4)
    {
        //MPC out put
        //Run following when measured the u1 and u2 both charged by the ac power
        //because the pi will Saturation if there are no input
        //by syc 2023.7.27
        get_ik_ref(meas_us,meas_udc);
        //output PWM 0-1-2-3
        //MPC计算与输出
        MPCcontrol(ik_ref_calcu,meas_u1,meas_u2,meas_u3,meas_u4,meas_us,meas_ik_abs,L_value,control_Ts,meas_udc,meas_io);
    }
    else 
    {
        PWM_stop();
        (Iref_PID.PID_reset)(&Iref_PID); //重置PI输出值
        ik_ref_calcu = 0;
    }




    gpiohs_set_pin(CONTROL_HEART_BEAT_GPIO_NUM, control_heart_beat); //Change the heart beat
    control_heart_beat = !control_heart_beat;
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
