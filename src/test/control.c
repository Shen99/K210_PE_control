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

double pwm1_duty = 0.1f;
double compute_times = 1.0f;

//常量
 double control_Ts = 1.0f/20000;  //控制周期
 double L_value = 0.005f;  //电感值
//指定输出直流
 double udc_ref = 25.00f;

//指定电流
 double ik_ref_calcu = 0.0f;
//交流输入幅值peak
 double us_attitude = 15.0f * 1.414f;


//开关管的通断时间
 double G1 = 0;
 double G2 = 0;
 double G3 = 0;
 double G4 = 0;

//电容电压
 double meas_u1 = 0.1f; //7606通道三》》》》C1电压 
 double meas_u2 = 0.1f;//7606通道四》》》》C2电压
 double meas_u3 = 0.1f;//7606通道五》》》》C3电压
 double meas_u4 = 0.1f;//7606通道六》》》》C4电压
 double meas_udc = 0.2f;
 double meas_u1_last = 0.1f;

 double meas_u2_last = 0.1f;

 double meas_u3_last = 0.1f;

 double meas_u4_last = 0.1f;

    //补偿时间
    double tb = 0;
    double tc = 0;
    double rc1 = 0;
    double rc2 = 0;
    double rc3 = 0;
    double rc4 = 0;

//交流电压 
 double meas_us= 0.1f; //7606通道一》》》》交流电压

//交流电流
 double meas_ik= 0.1f;//7606通道二》》》》电流

 double  meas_ik_abs= 0.1f;

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
    float reference;
    float p;
    float i;
    float integrate;
    float upper_limit;
    float lower_limit;
    float out;
    void (*PID_compute)(struct PID *pid_instance, float in);
};

void PID_compute(struct PID *pid_instance, float in);


//PIPIPIPIPIPIPIPI
struct PID Iref_PID = {
    .reference = 0,
    .p = 0.000001,
    .i = 0.001, //500*CONTROL_PERIOD,
    .integrate = 0,
    .upper_limit = 0.5,
    .lower_limit = -0.5,
    .out = 0,
    .PID_compute = &PID_compute
};
//PIPIPIPIPIPIPIPI


void PID_compute(struct PID *pid_instance, float in)
{
    float diff = pid_instance->reference - in;
    float proportion = pid_instance->p * diff;

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
}

//get ik_ref according to the meas_us and udc_ref and then through the PID controller to get the ik_ref
//input:mean_us,udc_ref
//to change the globle variable ik_ref_calcu
void get_ik_ref(double us,double udc)
{
    double udc_err = udc_ref - udc;
    (Iref_PID.PID_compute)(&Iref_PID, udc_err);
    
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

void MPCcontrol(double ik_ref,double u1,double u2,double u3,double u4,double us,double ik,double L,double Ts,double udc)
{
    G1=0; 
    G2=0; 
    G3=0; 
    G4=0; 

    double iakk[14]= {0};

    double s0 = 0;
    double s1 = 0;
    double s2 = 0;
    double s3 = 0;
    double s4 = 0;

    s0=(us-udc)/L;    //0000
    s1=(us)/L;       //1111
    s2=(us-((udc)*0.25))/L;    //1110  1101  0111  1011
    s3=(us-((udc)*0.5))/L;    //1010  1001  0011  1100 0110  0101
    s4=(us-((udc)*0.75))/L;        //0100  1000  0010  0001
    
    //计算两组矢量的作用时间
    double tp1=0;
    double tp2=0;
    double tp3=0;
    double tp4=0;


    tp1=(ik_ref-ik-s0*Ts)/(s4-s0);  //A s4+s0 3udc/4和udc
    tp2=(ik_ref-ik-s4*Ts)/(s3-s4);  //B s3+s4 udc/2和 3udc/4
    tp3=(ik_ref-ik-s3*Ts)/(s2-s3);  //C s2+s3 udc/4和udc/2
    tp4=(ik_ref-ik-s2*Ts)/(s1-s2);  //D s1+s2 0和udc/4

    //Limit amplitude
    if (tp1>Ts) tp1=Ts;
    if (tp1<0) tp1=0;
    if (tp2>Ts) tp2=Ts;
    if (tp2<0) tp2=0;
    if (tp3>Ts) tp3=Ts;
    if (tp3<0) tp3=0;
    if (tp4>Ts) tp4=Ts;
    if (tp4<0) tp4=0;

    //预测电流
    double A=0;double B=0;double C=0;double D=0;

    A=ik+s4*tp1+s0*(Ts-tp1); //图A
    B=ik+s3*tp2+s4*(Ts-tp2); //图B
    C=ik+s2*tp3+s3*(Ts-tp3); //图C
    D=ik+s1*tp4+s2*(Ts-tp4); //图D

    A=(A-ik_ref)*(A-ik_ref);
    B=(B-ik_ref)*(B-ik_ref);
    C=(C-ik_ref)*(C-ik_ref);
    D=(D-ik_ref)*(D-ik_ref);

    iakk[0]=A;
    iakk[1]=B;
    iakk[2]=C;
    iakk[3]=D;

    double min;
    min = iakk[0];
    uint32_t i;
    for (i=0;i<4;i++)
    {
        if(min>iakk[i])
        {
            min =  iakk[i];
        }

    }
    //注意数组编号
    iakk[0]= min;

    //补偿时间
    
    tb = 0;
    tc = 0;
    rc1 = 0;
    rc2 = 0;
    rc3 = 0;
    rc4 = 0;
    
    //计算补偿时间

    //C1和C2的补偿时间    
    rc1=abs_float((u1-meas_u1_last+0.000000001)/Ts);
    rc2=abs_float((u2-meas_u2_last+0.000000001)/Ts);
    tb = abs_float((u1 - u2) / (rc2 + rc1));
    //C3和C4的补偿时间
    rc3=abs_float((u3-meas_u3_last+0.000000001)/Ts);
    rc4=abs_float((u4-meas_u4_last+0.000000001)/Ts);
    tc=abs_float((u3 - u4)/(rc3 + rc4));

    /*
    AAAAAAAAAAAAAAAAAAAA
    */
    if(min==A) //s4 s0
        {
            //限制补偿时间的范围
            if(tb>0.25*tp1)
                tb=0.25*tp1;
            if(tb<0)
                tb=0;
            if(tc>0.25*tp1)
                tc=0.25*tp1;
            if(tc<0)
                tc=0;

            //11111111111
            if(u1>u2)
            {
                if(u3>u4)
                {
                    G1=tp1*0.25+0.5*tc;
                    G2=tp1*0.25+0.5*tb;
                    G3=tp1*0.25-0.5*tb;
                    G4=tp1*0.25-0.5*tc;
                }
                if(u3<u4)
                {
                    G1=tp1*0.25-0.5*tc;
                    G2=tp1*0.25+0.5*tb;
                    G3=tp1*0.25-0.5*tb;
                    G4=tp1*0.25+0.5*tc;
                }
            }
            if(u1<u2)
            {
                if(u3>u4)
                    {
                    G1=tp1*0.25+0.5*tc;
                    G2=tp1*0.25-0.5*tb;
                    G3=tp1*0.25+0.5*tb;
                    G4=tp1*0.25-0.5*tc;
                    }
                if(u3<u4)
                    {
                    G1=tp1*0.25-0.5*tc;
                    G2=tp1*0.25-0.5*tb;
                    G3=tp1*0.25+0.5*tb;
                    G4=tp1*0.25+0.5*tc;
                    }
            }
        }

    ////BBBBBBBBBBBBBBBBBBBBBBBBB
        else if(min==B)  //s3 s4
        {
            //限制补偿时间的范围
            if(tb>0.25*tp2)
                tb=0.25*tp2;
            if(tb<0)
                tb=0;
            if(tc>0.25*tp2)
                tc=0.25*tp2;
            if(tc<0)
                tc=0;
            //111111111111
            if(u1>u2)
            {66
            -=f(u90-==-=>u4)
                {
                    G1=0.5*tp2+0.25*(Ts-tp2)+0.5*tc;
                    G2=0.5*tp2+0.25*(Ts-tp2)+0.5*tb;
                    G3=0.5*tp2+0.25*(Ts-tp2)-0.5*tb;
                    G4=0.5*tp2+0.25*(Ts-tp2)-0.5*tc;
                }
                if(u3<u4)
                {
                    G1=0.5*tp2+0.25*(Ts-tp2)-0.5*tc;
                    G2=0.5*tp2+0.25*(Ts-tp2)+0.5*tb;
                    G3=0.5*tp2+0.25*(Ts-tp2)-0.5*tb;
                    G4=0.5*tp2+0.25*(Ts-tp2)+0.5*tc;
                }
            }
            //22222222222222
            if(u1<u2)
            {
                if(u3>u4)
                {
                    G1=0.5*tp2+0.25*(Ts-tp2)+0.5*tc;
                    G2=0.5*tp2+0.25*(Ts-tp2)-0.5*tb;
                    G3=0.5*tp2+0.25*(Ts-tp2)+0.5*tb;
                    G4=0.5*tp2+0.25*(Ts-tp2)-0.5*tc;
                }
                if(u3<u4)
                {
                    G1=0.5*tp2+0.25*(Ts-tp2)-0.5*tc;
                    G2=0.5*tp2+0.25*(Ts-tp2)-0.5*tb;
                    G3=0.5*tp2+0.25*(Ts-tp2)+0.5*tb;
                    G4=0.5*tp2+0.25*(Ts-tp2)+0.5*tc;
                }
            }
        }
    //CCCCCCCCCCCCCCCCCCCC
        if(min==C)  //s2 s3
    {
        //限制补偿时间的范围
        if(tb>0.25*tp3)
        {
            tb=0.25*tp3;
        }
        if(tb<0)
        {
            tb=0;
        }
        if(tc>0.25*tp3)
        {
            tc=0.25*tp3;
        }
        if(tc<0)
        {
            tc=0;
        }    
        if(u1>u2)
        {
            if(u3>u4)
                {
                G1=0.75*tp3+0.5*(Ts-tp3)+0.5*tc;
                G2=0.75*tp3+0.5*(Ts-tp3)+0.5*tb;
                G3=0.75*tp3+0.5*(Ts-tp3)-0.5*tb;
                G4=0.75*tp3+0.5*(Ts-tp3)-0.5*tc;
                }
            else if(u3<u4)
                {
                G1=0.75*tp3+0.5*(Ts-tp3)-0.5*tc;
                G2=0.75*tp3+0.5*(Ts-tp3)+0.5*tb;
                G3=0.75*tp3+0.5*(Ts-tp3)-0.5*tb;
                G4=0.75*tp3+0.5*(Ts-tp3)+0.5*tc;
                }
        }
        if(u1<u2)
        {
            if(u3>u4)
                {
                G1=0.75*tp3+0.5*(Ts-tp3)+0.5*tc;
                G2=0.75*tp3+0.5*(Ts-tp3)-0.5*tb;
                G3=0.75*tp3+0.5*(Ts-tp3)+0.5*tb;
                G4=0.75*tp3+0.5*(Ts-tp3)-0.5*tc;        
                }
            else if(u3<u4)
                {
                G1=0.75*tp3+0.5*(Ts-tp3)-0.5*tc;
                G2=0.75*tp3+0.5*(Ts-tp3)-0.5*tb;
                G3=0.75*tp3+0.5*(Ts-tp3)+0.5*tb;
                G4=0.75*tp3+0.5*(Ts-tp3)+0.5*tc;
                }
        }
    }
    /*
    DDDDDDDDDDDDDDDDDDDDDD
    */
    if (min == D) 
    {
        // 限制补偿时间的范围
        if (tb > 0.25 * tp4) 
        {
            tb = 0.25 * tp4;
        }
        if (tb < 0) 
        {
            tb = 0;
        }
        if (tc > 0.25 * tp4) 
        {
            tc = 0.25 * tp4;
        }
        if (tc < 0) 
        {
            tc = 0;
        }
        //分裂电容
        if(u1>u2)
        {
            if(u3>u4)
            {
                G1=Ts-0.25*(Ts-tp4)+0.5*tc;
                G2=Ts-0.25*(Ts-tp4)+0.5*tb;
                G3=Ts-0.25*(Ts-tp4)-0.5*tb;
                G4=Ts-0.25*(Ts-tp4)-0.5*tc;
            }
            else if(u3<u4)
            {
                G1=Ts-0.25*(Ts-tp4)-0.5*tc;
                G2=Ts-0.25*(Ts-tp4)+0.5*tb;
                G3=Ts-0.25*(Ts-tp4)-0.5*tb;
                G4=Ts-0.25*(Ts-tp4)+0.5*tc;
            }
        }
        if(u1<u2)
        {
            if(u3>u4)
            {
                G1=Ts-0.25*(Ts-tp4)+0.5*tc;
                G2=Ts-0.25*(Ts-tp4)-0.5*tb;
                G3=Ts-0.25*(Ts-tp4)+0.5*tb;
                G4=Ts-0.25*(Ts-tp4)-0.5*tc;
            }
            else if(u3<u4)
            {
                G1=Ts-0.25*(Ts-tp4)-0.5*tc;
                G2=Ts-0.25*(Ts-tp4)-0.5*tb;
                G3=Ts-0.25*(Ts-tp4)+0.5*tb;
                G4=Ts-0.25*(Ts-tp4)+0.5*tc;
            }
        }
        
        //limit the G1 G2 G3 G4
         if (G1/control_Ts>0.99)
        {
            G1=0.99*control_Ts;
        }
        if (G2/control_Ts>0.99)
        {
            G2=0.99*control_Ts;
        }
        if (G3/control_Ts>0.99)
        {
            G3=0.99*control_Ts;
        }
        if (G4/control_Ts>0.99)
        {
            G4=0.99*control_Ts;
        }
     //output PWM
        phase_change(0,0);
        duty_change(0,G1/control_Ts);
        phase_change(1,0.25f);
        duty_change(1,G2/control_Ts);
        phase_change(2,0.5f);
        duty_change(2,G3/control_Ts);
        phase_change(3,0.75f);
        duty_change(3,G4/control_Ts);
        
        
    
        //update the last meas_us
        meas_u1_last=u1;
        meas_u2_last=u2;
        meas_u3_last=u3;
        meas_u4_last=u4;

    }
}

/****************************************************
 * 
 * Control Loop           ***************************
 * 
 *                        ***************************
 *                        ***************************
 *                        ***************************
 *                        ***************************
 *                        ***************************
 *                        ***************************
 *                        ***************************
 * ******/

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
    compute_times += 1.0f;
    if (pwm1_duty > 1.0f)
    {
        pwm1_duty = 0.0f;
    }
    /*
    if (compute_times > 9000.0f && screen_send_flag<11000.f)
    {
        screen_send_flag = true;
    }
    */
    if (compute_times > 20000.0f)
    {
        compute_times = 0.0f;
    }
    //TEST PWM pin 4-5-6-7
    duty_change(7, pwm1_duty);
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
    //7606通道一》》》》交流电压
    //7606通道二》》》》电流
    //7606通道三》》》》C1电压
    //7606通道四》》》》C2电压
    //7606通道五》》》》C3电压
    //7606通道六》》》》C4电压

    meas_us = 3E-07f*adc_buf[2]*adc_buf[2]+adc_buf[2]*0.0805f-0.0205f; //(100k+10k)/940=117.0212765957
    meas_ik = 0.0051f*adc_buf[0]-0.1995f;    //10/(2.97-1.65)= 7.5758f   带绝对值的
    meas_ik_abs = abs_float(meas_ik);
    meas_u1 = adc_buf[7]*0.0362f-0.0017f;//(6000+20)/20=301 现在接到了C3
    meas_u2 = adc_buf[6]*0.0362f+0.182f;//  接到了C4
    meas_u3 = adc_buf[4]*0.036f+0.1526f;//4 C2
    meas_u4 = adc_buf[5]*0.0362f-0.286f;//3 C1

     //上面的测量值是由采样电路比例计算的，实际使用可能需要拟合校准

    
    meas_udc = meas_u1 + meas_u2;



    get_ik_ref(meas_us,meas_udc);
    //output PWM 0-1-2-3
    //MPC计算与输出
    MPCcontrol(ik_ref_calcu,meas_u1,meas_u2,meas_u3,meas_u4,meas_us,meas_ik_abs,L_value,control_Ts,meas_udc);
      
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
