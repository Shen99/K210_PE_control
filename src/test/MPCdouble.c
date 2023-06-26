

#include "MPCdouble.h"
/**
* Function       MPCcontrol
* @author        Yucheng
* @date          2023.06.24
* @brief         mpc control five level rectifier
* @param[in]     ik_ref,u1,u2,u3,u4,us,ik,L,Ts,udc,i1,i2,i3,i4
* @param[out]    G1,G2,G3,G4
*/

void MPCcontrol(float ik_ref,float u1,float u2,float u3,float u4,float us,float ik,float L,float Ts,float udc,float i1,float i2,float i3,float i4)
{
    G1=0; 
    G2=0; 
    G3=0; 
    G4=0;

    float iakk[14]= {0};

    // x1=udc*0.5;
    // x2=udc*0.5;
    // x3=udc*0.25;
    // udc*0.25;


    float s0 = 0;
    float s1 = 0;
    float s2 = 0;
    float s3 = 0;
    float s4 = 0;

    s0=(us-udc)/L;    //0000
    s1=(us)/L;       //1111
    s2=(us-((udc)*0.25))/L;    //1110  1101  0111  1011
    s3=(us-((udc)*0.5))/L;    //1010  1001  0011  1100 0110  0101
    s4=(us-((udc)*0.75))/L;        //0100  1000  0010  0001
    
    //计算两组矢量的作用时间
    float tp1=(ik_ref-ik-s0*Ts)/(s4-s0);  //A s4+s0 3udc/4和udc
    float tp2=(ik_ref-ik-s4*Ts)/(s3-s4);  //B s3+s4 udc/2和 3udc/4
    float tp3=(ik_ref-ik-s3*Ts)/(s2-s3);  //C s2+s3 udc/4和udc/2
    float tp4=(ik_ref-ik-s2*Ts)/(s1-s2);  //D s1+s2 0和udc/4

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
    float A=0;float B=0;float C=0;float D=0;

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

    float min;
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
    float tb = 0;
    float tc = 0;
    float rc1 = 0;
    float rc2 = 0;
    float rc3 = 0;
    float rc4 = 0;

    //计算补偿时间
    if (u1 > u2) 
    {
        rc1 = -(i1 / (2 * 0.001));
        //     rc1=abs((u1-x1)/Ts);
        //     rc2=abs((u2-x2)/Ts);
        rc2 = i2 / (2 * 0.001);
        tb = abs(u1 - u2) / (rc2 - rc1);
    }
    if (u1 < u2) 
    {
        rc1 = (i1 / (2 * 0.001));
        rc2 = -i2 / (2 * 0.001);
        //       rc1=(u1-x1)/Ts;
        //     rc2=(u2-x2)/Ts;
        tb = abs(u1 - u2) / (rc1 - rc2);
    }

    if (u3 > u4) 
    {
        rc3 = -(i3 / (0.5 * 0.001));
        rc4 = i4 / (0.5 * 0.001);
        tc = abs(u3 - u4) / (rc4 - rc3);
    }
    if (u3 < u4) 
    {
        rc3 = (i3 / (0.5 * 0.001));
        rc4 = -i4 / (0.5 * 0.001);
        tc = abs(u3 - u4) / (rc3 - rc4);
    }

    //   rc3=abs((u3-x3)/Ts);
    //   rc4=abs((u4-x4)/Ts);
    //   tc=abs(u3-u4)/(rc3+rc4);

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
            {
                if(u3>u4)
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

        //分裂电容
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
        //分裂电容
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
        
        meas_u1_last=u1;
        meas_u2_last=u2;
        meas_u3_last=u3;
        meas_u4_last=u4;

    }
}

//get ik_ref according to the meas_us and udc_ref and then through the PID controller to get the ik_ref
//input:mean_us,udc_ref
//to change the globle variable ik_ref_calcu
void get_ik_ref(float us,float udc)
{
    float udc_err = udc_ref - udc;
    float PID_out = PID(udc_err);
    ik_ref_calcu = fabs(us / us_attitude * PID_out);
}

