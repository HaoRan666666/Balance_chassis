#include "stm32h7xx.h"                  // Device header
#include "arm_math.h"
#include "Leg_Controller.h"
#include "controller.h"

PIDInstance Left_Leg_Pid,Right_Leg_Pid; //腿长控制pid结构体

/*
 *函数简介:腿长控制VMC
 *参数说明:腿部状态结构体
 *参数说明:摆杆推力F
 *参数说明:摆杆扭矩Tp
 *参数说明:髋关节转矩T1
 *参数说明:膝关节转矩T2
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
    Left_Leg_Pid.Kp=0;
	Left_Leg_Pid.Ki=0;
	Left_Leg_Pid.Kd=0;
	Left_Leg_Pid.DeadBand=0;
	Left_Leg_Pid.MaxOut=0;
	Left_Leg_Pid.Need_Value=0.14;

	Right_Leg_Pid.Kp=0;
	Right_Leg_Pid.Ki=0;
	Right_Leg_Pid.Kd=0;
	Right_Leg_Pid.DeadBand=0;
	Right_Leg_Pid.MaxOut=0;
	Right_Leg_Pid.Need_Value=0.14;
}



/*
 *函数简介:腿长控制
 *参数说明:左腿ΔF
 *参数说明:右腿ΔF
 *返回类型:无
 *备注:无
 */
void Leg_Controller_LegControl(float *LeftLeg_DeltaF,float *RightLeg_DeltaF)
{
   Balance_data *balance_data = Get_Balance_Data();
   (*LeftLeg_DeltaF)=PIDCalculate(&Left_Leg_Pid,balance_data->Leg_L.L0,Left_Leg_Pid.Need_Value);
   (*RightLeg_DeltaF)=PIDCalculate(&Right_Leg_Pid,balance_data->Leg_R.L0,Right_Leg_Pid.Need_Value);
}


/*
 *函数简介:腿长控制逆运动学解算
 *参数说明:摆杆长度L0
 *参数说明:摆杆角度phi0
 *参数说明:大腿与水平面夹角theta_1
 *参数说明:小腿连杆与水平面夹角theta_2
 *返回类型:无
 *备注:无
 */
void Leg_Controller_InverseKinematicsSolution(float L_0,float phi_0,float *theta_1,float *small_rod_angle)
{
	float XC=L_0*cosf(phi_0);
	float YC=L_0*sinf(phi_0);
	
	float A=L1+XC;
	float B=L1*L1-XC*XC;
	float C=L2*L2-YC*YC;
	float D=L2*L2+YC*YC;
	
	float phi1=2.0f*atan2f(2.0f*L1*YC+sqrtf(2.0f*L1*L1*D+2.0f*XC*XC*C-B*B-C*C),A*A-C);
	if(phi1>2.0f*PI)phi1-=2.0f*PI;
	if(phi1<0)phi1+=2.0f*PI;
	
	A=L2+L1;
	B=-XC;
	C=L2-L1;
	D=XC+L1;
	
	float E=(A*A-B*B-YC*YC);
	float F=(B*B-C*C+YC*YC);
	float phi4=2.0f*atan2f(2.0f*L1*YC-sqrtf(E*F),D*D+YC*YC-L2*L2);
	if(phi4>PI)phi4-=2.0f*PI;
	if(phi4<-PI)phi4+=2.0f*PI;

	(*theta_1)=phi4;
	(*small_rod_angle)=(PI-phi1);
}


/*
 *函数简介:腿长控制LQR控制腿长
 *参数说明:腿部状态结构体
 *参数说明:目标五连杆phi1
 *参数说明:目标五连杆phi4
 *参数说明:髋关节1转矩T1
 *参数说明:髋关节2转矩T2
 *返回类型:无
 *备注:无
 */
//用于倒地自起
void Leg_Controller_LengthLQR(Leg_data Leg,float target_theta_1,float target_small_rod_angle,float *T1,float *T2)
{
	#define Leg_Controller_LQR_FeedForward	5.4f//3.8f
	#define Leg_Controller_LQR_K1			31.6228f
	#define Leg_Controller_LQR_K2			1.2142f//31.6228    1.2142
	
	(*T1)=-Leg_Controller_LQR_FeedForward-Leg_Controller_LQR_K1*(Leg.theta_1-target_theta_1)-Leg_Controller_LQR_K2*Leg.theta_1;
	(*T2)=Leg_Controller_LQR_FeedForward-Leg_Controller_LQR_K1*(Leg.small_rod_angle-target_small_rod_angle)-Leg_Controller_LQR_K2*Leg.d_small_rod_angle;
	//不知道能不能直接用小杆角度和角速度
}
