#ifndef __LQR_H
#define __LQR_H

#define FN_Threshold 20.0f//支持力阈值 

void LQR_Clc(float *Tl,float *Tpl,float *Tr,float *Tpr,float target_x,float target_dx,float target_theta);
float Get_Uspeed(float target_x,float target_dx);
float Get_Uelse_R(void);
float Get_Uelse_L(void);

typedef struct 
{
    float T_theta_gain;
    float T_dtheta_gain;
    float T_x_gain;
    float T_dx_gain;
    float T_pitch_gain;
    float T_dpitch_gain;

    float Tp_theta_gain;
    float Tp_dtheta_gain;
    float Tp_x_gain;
    float Tp_dx_gain;
    float Tp_pitch_gain;
    float Tp_dpitch_gain;
} LQR_Gains;
#endif
