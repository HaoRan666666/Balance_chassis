#ifndef __LEG_CONTROLLER_H
#define __LEG_CONTROLLER_H

#include "Observer.h"
#include "controller.h"
void Leg_Controller_VMC(Leg_data Leg,float F,float Tp,float *T1,float *T2);
void Leg_Controller_LegControlInit(void);
void Leg_Controller_Length_Control(float *LeftLeg_DeltaF,float *RightLeg_DeltaF);
void Leg_Controller_InverseKinematicsSolution(float L_0,float phi_0,float *phi_1,float *phi_4);

void Leg_Controller_AngularVelocity(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp);
void Leg_Controller_AngularPosition(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp);

extern PIDInstance Left_Leg_Pid,Right_Leg_Pid; //腿长控制pid结构体
extern PIDInstance Leg_omega_ControllerPID_L,Leg_omega_ControllerPID_R; //腿摆角角速度控制pid结构体
extern PIDInstance Leg_angle_ControllerPID_L,Leg_angle_ControllerPID_R; //腿摆角角度控制pid结构体
#endif
