#include "stm32h7xx.h"                
#include "arm_math.h"
#include "Leg_Controller.h"
#include "controller.h"

PIDInstance Left_Leg_Pid,Right_Leg_Pid; //腿长控制pid结构体
PIDInstance Leg_omega_ControllerPID_L,Leg_omega_ControllerPID_R; //腿摆角角速度控制pid结构体
PIDInstance Leg_angle_ControllerPID_L,Leg_angle_ControllerPID_R; //腿摆角角度控制pid结构体

/*
 *函数简介:腿长控制VMC
 *参数说明:腿部状态结构体
 *参数说明:摆杆推力F
 *参数说明:摆杆扭矩Tp
 *参数说明:小腿转矩T1
 *参数说明:大腿转矩T2
 *返回类型:无
 *备注:无
 */
void Leg_Controller_VMC(Leg_data Leg,float F,float Tp,float *T1,float *T2)
{	
	(*T1)=(Leg.J_11)*F+(Leg.J_21)*Tp;
	(*T2)=(Leg.J_12)*F+(Leg.J_22)*Tp;
}


/*
 *函数简介:腿长控制初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Leg_Controller_LegControlInit(void)
{
    Left_Leg_Pid.Kp=1200;
	Left_Leg_Pid.Ki=0;
	Left_Leg_Pid.Kd=0;
	Left_Leg_Pid.DeadBand=0;
	Left_Leg_Pid.MaxOut=200;
	Left_Leg_Pid.Need_Value=0.154;

	Right_Leg_Pid.Kp=1200;
	Right_Leg_Pid.Ki=0;
	Right_Leg_Pid.Kd=0;
	Right_Leg_Pid.DeadBand=0;
	Right_Leg_Pid.MaxOut=200;
	Right_Leg_Pid.Need_Value=0.154;
	
	Leg_omega_ControllerPID_L.Kp=-8;
	Leg_omega_ControllerPID_L.Ki=-10;
	Leg_omega_ControllerPID_L.Kd=0;
	Leg_omega_ControllerPID_L.DeadBand=0;
	Leg_omega_ControllerPID_L.MaxOut=15;
	Leg_omega_ControllerPID_L.Need_Value=0;
	Leg_omega_ControllerPID_L.Improve=PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement;

	Leg_omega_ControllerPID_R.Kp=-8;
	Leg_omega_ControllerPID_R.Ki=-10;
	Leg_omega_ControllerPID_R.Kd=0;
	Leg_omega_ControllerPID_R.DeadBand=0;
	Leg_omega_ControllerPID_R.IntegralLimit=8;
	Leg_omega_ControllerPID_R.MaxOut=15;
	Leg_omega_ControllerPID_R.Need_Value=0;
	Leg_omega_ControllerPID_R.Improve=PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement;

	Leg_angle_ControllerPID_L.Kp=0;
	Leg_angle_ControllerPID_L.Ki=0;
	Leg_angle_ControllerPID_L.Kd=0;
	Leg_angle_ControllerPID_L.DeadBand=0;
	Leg_angle_ControllerPID_L.MaxOut=5;
	Leg_angle_ControllerPID_L.Need_Value=0;


	Leg_angle_ControllerPID_R.Kp=0;
	Leg_angle_ControllerPID_R.Ki=0;
	Leg_angle_ControllerPID_R.Kd=0;
	Leg_angle_ControllerPID_R.DeadBand=0;
	Leg_angle_ControllerPID_R.MaxOut=5;
	Leg_angle_ControllerPID_R.Need_Value=0;
}



/*
 *函数简介:腿长控制
 *参数说明:左腿ΔF
 *参数说明:右腿ΔF
 *返回类型:无
 *备注:无
 */
void Leg_Controller_Length_Control(float *LeftLeg_DeltaF,float *RightLeg_DeltaF)
{
   Balance_data *balance_data = Get_Balance_Data();
   (*LeftLeg_DeltaF)=PIDCalculate(&Left_Leg_Pid,balance_data->Leg_L.L0,Left_Leg_Pid.Need_Value);
   (*RightLeg_DeltaF)=PIDCalculate(&Right_Leg_Pid,balance_data->Leg_R.L0,Right_Leg_Pid.Need_Value);
}

/*
 *函数简介:腿长控制逆运动学解算
 *参数说明:摆杆长度L0
 *参数说明:摆杆角度phi0
 *参数说明:五连杆phi1
 *参数说明:五连杆phi4
 *返回类型:无
 *备注:无
 */
void Leg_Controller_InverseKinematicsSolution(float L_0,float phi_0,float *phi_1,float *phi_4)
{
	float XC=l_5/2.0f+L_0*cosf(phi_0);
	float YC=L_0*sinf(phi_0);
	
	float A=l_1+XC;
	float B=l_1*l_1-XC*XC;
	float C=l_2*l_2-YC*YC;
	float D=l_2*l_2+YC*YC;
	
	float phi1=2.0f*atan2f(2.0f*l_1*YC+sqrtf(2.0f*l_1*l_1*D+2.0f*XC*XC*C-B*B-C*C),A*A-C);
	if(phi1>2.0f*PI)phi1-=2.0f*PI;
	if(phi1<0)phi1+=2.0f*PI;
	
	A=l_3+l_4;
	B=l_5-XC;
	C=l_3-l_4;
	D=XC+l_4-l_5;
	
	float E=(A*A-B*B-YC*YC);
	float F=(B*B-C*C+YC*YC);
	float phi4=2.0f*atan2f(2.0f*l_4*YC-sqrtf(E*F),D*D+YC*YC-l_3*l_3);
	if(phi4>PI)phi4-=2.0f*PI;
	if(phi4<-PI)phi4+=2.0f*PI;

	(*phi_1)=phi1;
	(*phi_4)=phi4;
}
/*
 *函数简介:腿部摆角角速度控制
 *参数说明:左腿ΔTp
 *参数说明:右腿ΔTp
 *返回类型:无
 *备注:无
 */
void Leg_Controller_AngularVelocity(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp)
{
   Balance_data *balance_data = Get_Balance_Data();
   *LeftLeg_DeltaTp=PIDCalculate(&Leg_omega_ControllerPID_L,balance_data->Leg_L.dtheta,Leg_omega_ControllerPID_L.Need_Value);
   *RightLeg_DeltaTp=PIDCalculate(&Leg_omega_ControllerPID_R,balance_data->Leg_R.dtheta,Leg_omega_ControllerPID_R.Need_Value);
}

/*
 *函数简介:腿部摆角角度控制	
 *参数说明:左腿ΔTp
 *参数说明:右腿ΔTp
 *返回类型:无
 *备注:无
 */
void Leg_Controller_AngularPosition(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp)
{
   Balance_data *balance_data = Get_Balance_Data();
   float Target_omega_L=PIDCalculate(&Leg_angle_ControllerPID_L,balance_data->Leg_L.theta,Leg_angle_ControllerPID_L.Need_Value);
   float Target_omega_R=PIDCalculate(&Leg_angle_ControllerPID_R,balance_data->Leg_R.theta,Leg_angle_ControllerPID_R.Need_Value);

   Leg_omega_ControllerPID_L.Need_Value=Target_omega_L;
   Leg_omega_ControllerPID_R.Need_Value=Target_omega_R;
   Leg_Controller_AngularVelocity(LeftLeg_DeltaTp,RightLeg_DeltaTp);
}