#ifndef __MOTION_CONTROLLER_H
#define __MOTION_CONTROLLER_H

#include "controller.h"

extern PIDInstance Yaw_ControllerPositionPID;

void Motion_Controller_Init(void);//综合运动控制初始化
void Motion_Controller_Yaw_Control(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT, float w_Limit);//Yaw控制
void Motion_Controller_Yaw_Control_Follow(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT, float w_Limit);//Yaw控制(跟随云台)
void Motion_Controller_LegCoordination_Control(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp);//双腿协调
void Motion_Controller_Roll_Control(float Roll_Target,float *LeftLeg_DeltaL0,float *RightLeg_DeltaL0,float *LeftLeg_DeltaF,float *RightLeg_DeltaF);//Roll补偿
void Motion_Controller_Yaw_Control_Pid(float *LeftWheel_DeltaT,float *RightWheel_DeltaT,float Target_Yaw);
#endif
