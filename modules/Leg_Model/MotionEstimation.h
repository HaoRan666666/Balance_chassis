#ifndef _MOTIONESTIMATION_H
#define _MOTIONESTIMATION_H

#include "kalman_filter.h"
#include "main.h"
typedef struct
{
    float v; //滤波后的速度
    float x; //滤波后的位置
    float x_nofilter; //未滤波位置
    float v_nofilter; //未滤波速度

    float v_measured_L; //测量的左轮速度
    float v_measured_R; //测量的右轮速度
    float v_measured;   //测量的整体速度

    float a_measured; //测量的加速度

    KalmanFilter_t KalmanFilter_Instance;
} MotionEstimation_Balance;

void MotionEstimation_init(void);
void MotionEstimation_MeasureUpdate(void);
void MotionEstimation_Update(void);

#endif