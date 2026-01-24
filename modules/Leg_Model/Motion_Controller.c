#include "stm32h7xx.h"                  // Device header
#include "controller.h"
#include "Observer.h"
#include "user_lib.h"

#define Body_w_max 0         //机身Yaw角速度限制
#define Body_w_min 0         //机身Yaw角速度限制


#define YAW_Controll_LQR_Angel_K 0 
#define YAW_Controll_LQR_Speed_K 0  //matlab求解


PIDInstance LegCoordination_ControllerPID; //双腿协调pid   (两条虚拟杆的角度差乘以系数kp之后分别加到两条腿上)  注意极性
// PIDInstance Pitch_ControllerPID;     
// PIDInstance Roll_L0ControllerPID;          
PIDInstance Roll_FControllerPID;
// PIDInstance X_controllerPID;
PIDInstance Yaw_ControllerPID;


void Motion_Controller_Init(void)
{
    //双腿协调pid初始化  
    //TODO:需要调参
   LegCoordination_ControllerPID.Kp=0;
   LegCoordination_ControllerPID.Ki=0;
   LegCoordination_ControllerPID.Kd=0;
   LegCoordination_ControllerPID.MaxOut=0;
   LegCoordination_ControllerPID.Need_Value=0;

   Roll_FControllerPID.Kp=0;
   Roll_FControllerPID.Ki=0;
   Roll_FControllerPID.Kd=0;
   Roll_FControllerPID.MaxOut=0;
   Roll_FControllerPID.Need_Value=0;

   Yaw_ControllerPID.Kp=-2;
   Yaw_ControllerPID.Ki=0;
   Yaw_ControllerPID.Kd=0;
   Yaw_ControllerPID.MaxOut=3;
   Yaw_ControllerPID.Need_Value=0;
   
}

void Motion_Controller_Yaw_Control_Pid(float *LeftWheel_DeltaT,float *RightWheel_DeltaT)
{
    Balance_data* balance_status=Get_Balance_Data();

    PIDCalculate(&Yaw_ControllerPID,balance_status->body_data.Yaw,Yaw_ControllerPID.Need_Value);

    float T=Yaw_ControllerPID.Output;

    (*LeftWheel_DeltaT)=-T/2.0f;     //正负有待验证
    (*RightWheel_DeltaT)=T/2.0f;
}


void Motion_Controller_Yaw_Control(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT,float w_Limit)
{
      Balance_data* balance_status=Get_Balance_Data();
    //角度环
    float phi_dot = YAW_Controll_LQR_Angel_K* (balance_status->body_data.Yaw-target_Yaw);  //机身角速度

    phi_dot=float_constrain(phi_dot,-w_Limit,w_Limit);//限幅
    //速度环
    float T=YAW_Controll_LQR_Speed_K *(balance_status->body_data.Yaw+phi_dot);//这里的加号是因为d_Yaw的正方向和期望的角速度正方向相反

    (*LeftWheel_DeltaT)=-T/2.0f;     //正负有待验证
    (*RightWheel_DeltaT)=T/2.0f;
}

/*
 *函数简介:Yaw控制(跟随云台)
 *参数说明:左轮ΔT
 *参数说明:右轮ΔT
 *返回类型:无
 *备注:无
 */
void Motion_Controller_Yaw_Control_Follow(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT, float w_Limit)
{ 
     Balance_data* balance_status=Get_Balance_Data();
    //角度环
    float phi_dot = YAW_Controll_LQR_Angel_K* (balance_status->body_data.Yaw-target_Yaw);  //机身角速度
    //float w=YAW_Controll_LQR_Angel_K*(-(GM6020_MotorStatus[0].Angle-Yaw_GM6020PositionValue)/8192.0f*2.0f*PI-target_Yaw);
    phi_dot=float_constrain(phi_dot,-w_Limit,w_Limit);//限幅
    //速度环
    float T=YAW_Controll_LQR_Speed_K *(balance_status->body_data.d_Yaw+phi_dot);//这里的加号是因为d_Yaw的正方向和期望的角速度正方向相反

    (*LeftWheel_DeltaT)=-T/2.0f;     //正负有待验证
    (*RightWheel_DeltaT)=T/2.0f;
}

//双腿协调
//以两条腿的虚拟杆夹角差作为输入，经过pid计算后分别加到两条腿的Tp上
void Motion_Controller_LegCoordination_Control(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp)
{
   Balance_data* balance=Get_Balance_Data();
   float delta_node_angle=balance->Leg_R.phi_0-balance->Leg_L.phi_0;//将双腿角度差作为输入 (正负有待测试， 或者改变pid参数极性也可以)
   PIDCalculate(&LegCoordination_ControllerPID,delta_node_angle,LegCoordination_ControllerPID.Need_Value);//在control任务中改变Need_Value，通常都是0（除非有小黑子）
   
   	(*LeftLeg_DeltaTp)=LegCoordination_ControllerPID.Output;
	(*RightLeg_DeltaTp)=-LegCoordination_ControllerPID.Output;
}

//roll控制补偿
void Motion_Controller_Roll_Control(float Roll_Target,float *LeftLeg_DeltaL0,float *RightLeg_DeltaL0,float *LeftLeg_DeltaF,float *RightLeg_DeltaF)
{
   Balance_data * Balance =Get_Balance_Data();
   // 长度控制
   float delta_L0= Balance->Leg_L.L0-Balance->Leg_R.L0;   //计算两条腿当前长度差

   	float A=delta_L0*arm_cos_f32(Balance->body_data.Roll-Roll_Target)+2.0f*Body_Rl*arm_sin_f32(Balance->body_data.Roll-Roll_Target);
	float B=-delta_L0*arm_sin_f32(Balance->body_data.Roll-Roll_Target)+2.0f*Body_Rl*arm_cos_f32(Balance->body_data.Roll-Roll_Target);
    float d_l0_r,d_l0_l,tan_delta;
    if(B==0) 
    {
        d_l0_l=0;
        d_l0_r=0;
    }
    else //防止除0
    {
    tan_delta=A/B;

    d_l0_r=Body_Rl*tan_delta;
    d_l0_l=-Body_Rl*tan_delta;
    }
   
   (*LeftLeg_DeltaL0)=d_l0_l;
   (*RightLeg_DeltaL0)=d_l0_r;
//支持力补偿
  Roll_FControllerPID.Need_Value=Roll_Target;
  PIDCalculate(&Roll_FControllerPID,Balance->body_data.Roll,Roll_FControllerPID.Need_Value);
  (*LeftLeg_DeltaF)=Roll_FControllerPID.Output;
  (*RightLeg_DeltaF)=-Roll_FControllerPID.Output;  //(正负有待测试)

}