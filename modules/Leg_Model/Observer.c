#include "Observer.h"
#include "arm_math.h"
#include "ins_task.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "Leg_Controller.h"

Balance_data Balance_status; //储存轮腿所有数据的结构体 
Balance_data*  Get_Balance_Data()
{
   return &Balance_status;
}

void Observer_init(void)
{
   Balance_status.dt=0.002;  //500hz控制频率

   // 初始化左腿变量
   Balance_status.Leg_L.ddtheta = 0;
   Balance_status.Leg_L.dtheta = 0;
   Balance_status.Leg_L.theta = 0;
   Balance_status.Leg_L.last_dtheta = 0;
   Balance_status.Leg_L.L0=0.10;
   Balance_status.Leg_L.d_L0=0;
   Balance_status.Leg_L.dd_L0=0;
   // 初始化右腿变量
   Balance_status.Leg_R.ddtheta = 0;
   Balance_status.Leg_R.dtheta = 0;
   Balance_status.Leg_R.theta = 0;
   Balance_status.Leg_R.last_dtheta = 0;
   Balance_status.Leg_R.L0=0.10;
   Balance_status.Leg_R.d_L0=0;
   Balance_status.Leg_R.dd_L0=0;

   MotionEstimation_init();
}



/**************************************直接采用五连杆的运动学分析方法***************************/

/*
 *函数简介:观测器数据处理
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_DataGet(void)
{
   INS_t INS=*(Get_INS_Instance());
   DJIMotorInstance *dji_motor_instance=Get_DJI_Instance();
   DMMotorInstance *dm_motor_instance=Get_DM_Instance();
   /************************************车体数据******************************/
     
   Balance_status.body_data.a_zb = INS.MotionAccel_b[2];

   Balance_status.body_data.a_yE = INS.MotionAccel_n[0];
   Balance_status.body_data.a_zE = INS.MotionAccel_n[2];

   Balance_status.body_data.Pitch = -INS.Pitch;  //注意确认一下是不是逆时针为正方向（右手定则） 
   Balance_status.body_data.Roll  = INS.Roll;   
   Balance_status.body_data.Yaw   = INS.Yaw;

   Balance_status.body_data.d_pitch = -INS.Gyro[0];   //注意确认一下是不是逆时针为正方向（右手定则）
   Balance_status.body_data.d_Roll  = INS.Gyro[1];
   Balance_status.body_data.d_Yaw   = INS.Gyro[2];

   /************************************车轮数据******************************/
   float left_reversal_factor = -1.0f;  // 左侧反转
   //左轮
   Balance_status.Leg_L.wheel.angle=dji_motor_instance[0].measure.total_angle * left_reversal_factor * DEG_TO_RAD ;  //总角度（rad）
   Balance_status.Leg_L.wheel.speed=dji_motor_instance[0].measure.speed_aps* left_reversal_factor*DEG_TO_RAD;    
   Balance_status.Leg_L.wheel.x=Balance_status.Leg_L.wheel.angle * Wheel_R ;  //单位是米
   Balance_status.Leg_L.wheel.T=dji_motor_instance[0].measure.real_current* left_reversal_factor * 20 * 0.3 / 16384 * GEAR_RATIO;//初始数据到16384,转化后乘以转矩常数
   //右轮
   Balance_status.Leg_R.wheel.angle=dji_motor_instance[1].measure.total_angle*DEG_TO_RAD;  //总角度 //注意单位换算
   Balance_status.Leg_R.wheel.speed=dji_motor_instance[1].measure.speed_aps*DEG_TO_RAD;  
   Balance_status.Leg_R.wheel.x=Balance_status.Leg_R.wheel.angle * Wheel_R ;  //单位是米
   Balance_status.Leg_R.wheel.T=dji_motor_instance[1].measure.real_current * 20 * 0.3/ 16384 * GEAR_RATIO ;//初始数据到16384,转化后乘以转矩常数

   /************************************腿部数据******************************/   
   //左腿   
	Balance_status.Leg_L.phi_1=dm_motor_instance[0].measure.position;//左后电机单圈角度     //小连杆与水平线夹角
      if(Balance_status.Leg_L.phi_1<0)     //TODO：确认达妙反馈的是不是单圈角度
   {
      Balance_status.Leg_L.phi_1=Balance_status.Leg_L.phi_1+2.0f*PI;
   }

	Balance_status.Leg_L.phi_4=dm_motor_instance[2].measure.position;//左前电机单圈角度     //大腿与水平线夹角
	Balance_status.Leg_L.dphi_1=dm_motor_instance[0].measure.velocity;  //注意正方向问题 ，应该是以逆时针为正方向
   Balance_status.Leg_L.dphi_4=dm_motor_instance[2].measure.velocity;  //注意正方向问题 ，应该是以逆时针为正方向

   if(Balance_status.Leg_L.phi_1!=0&&Balance_status.Leg_L.phi_4!=0)//防止出现除零
   {
      Observer_LegForwardKinematicsSolution(&(Balance_status.Leg_L));
   }
	
	Balance_status.Leg_L.T1=dm_motor_instance[0].measure.torque;   //小连杆电机扭矩    逆时针为正   //需要归化到0-2pi
	Balance_status.Leg_L.T2=dm_motor_instance[2].measure.torque;   //大腿电机扭矩      逆时针为正
	
	Balance_status.Leg_L.theta=PI/2.0f-(Balance_status.Leg_L.phi_0+Balance_status.body_data.Pitch*DEG_TO_RAD);  
	Balance_status.Leg_L.last_dtheta=Balance_status.Leg_L.dtheta;
	Balance_status.Leg_L.dtheta=-(Balance_status.Leg_L.dphi_0+Balance_status.body_data.d_pitch);     //顺时针为正
	Balance_status.Leg_L.ddtheta=0.19f*(Balance_status.Leg_L.dtheta-Balance_status.Leg_L.last_dtheta)/Balance_status.dt+0.81f*Balance_status.Leg_L.ddtheta;//一阶低通滤波
	
	Observer_GetFN(&Balance_status.Leg_L);
	
	//右腿
	Balance_status.Leg_R.phi_1=dm_motor_instance[1].measure.position;//右后电机单圈角度     //小连杆与水平线夹角
   //由-pi到pi归化到0-2pi
   if(Balance_status.Leg_R.phi_1<0)
   {
      Balance_status.Leg_R.phi_1=Balance_status.Leg_R.phi_1+2.0f*PI;
   }

	Balance_status.Leg_R.phi_4=dm_motor_instance[3].measure.position;//右前电机单圈角度     //大腿与水平线夹角
	Balance_status.Leg_R.dphi_1=dm_motor_instance[1].measure.velocity;  //注意正方向问题 ，应该是以逆时针为正方向
   Balance_status.Leg_R.dphi_4=dm_motor_instance[3].measure.velocity;  //注意正方向问题 ，应该是以逆时针为正方向

    if(Balance_status.Leg_R.phi_1!=0&&Balance_status.Leg_R.phi_4!=0)//防止出现除零
   {
	Observer_LegForwardKinematicsSolution(&(Balance_status.Leg_R));
   }
	Balance_status.Leg_R.T1=dm_motor_instance[1].measure.torque;   //小连杆电机扭矩    逆时针为正
	Balance_status.Leg_R.T2=dm_motor_instance[3].measure.torque;   //大腿电机扭矩      逆时针为正
	
	Balance_status.Leg_R.theta=PI/2.0f-(Balance_status.Leg_R.phi_0+Balance_status.body_data.Pitch * DEG_TO_RAD);  
	Balance_status.Leg_R.last_dtheta=Balance_status.Leg_R.dtheta;
	Balance_status.Leg_R.dtheta=-(Balance_status.Leg_R.dphi_0+Balance_status.body_data.d_pitch);     //顺时针为正

	Balance_status.Leg_R.ddtheta=0.19f*(Balance_status.Leg_R.dtheta-Balance_status.Leg_R.last_dtheta)/Balance_status.dt+0.81f*Balance_status.Leg_R.ddtheta;//一阶低通滤波
	Balance_status.Leg_R.test=(Balance_status.Leg_R.dtheta-Balance_status.Leg_R.last_dtheta)/Balance_status.dt;//未滤波加速度 做一个对比 效果显著
	
   Observer_GetFN(&Balance_status.Leg_R);
	/************************************机体数据******************************/

	//运动估计
	MotionEstimation_Update();
	
	//功率

}


/*
 *函数简介:腿长正运动学解算
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_LegForwardKinematicsSolution(Leg_data *Leg)
{
	//获取L0和phi0
	float phi_1=Leg->phi_1;
	float phi_4=Leg->phi_4;
	float dphi_1=Leg->dphi_1;
	float dphi_4=Leg->dphi_4;
	
	float XB=l_1*arm_cos_f32(phi_1);
	float YB=l_1*arm_sin_f32(phi_1);
	float XD=l_5+l_4*arm_cos_f32(phi_4);
	float YD=l_4*arm_sin_f32(phi_4);
	
	float lBD_2=(XD-XB)*(XD-XB)+(YD-YB)*(YD-YB);
	
	float A0=2*l_2*(XD-XB);
	float B0=2*l_2*(YD-YB);
	float C0=l_2*l_2+lBD_2-l_3*l_3;
	float phi2=2*atan2f((B0+sqrt(A0*A0+B0*B0-C0*C0)),A0+C0);
	float phi3=atan2f(YB-YD+l_2*arm_sin_f32(phi2),XB-XD+l_2*arm_cos_f32(phi2));
	
	float XC=l_1*arm_cos_f32(phi_1)+l_2*arm_cos_f32(phi2);
	float YC=l_1*arm_sin_f32(phi_1)+l_2*arm_sin_f32(phi2);
	
	(Leg->L0)=sqrt((XC-l_5/2.0f)*(XC-l_5/2.0f)+YC*YC);
	(Leg->phi_0)=atan2f(YC,(XC-l_5/2.0f));  
	//获取VMC雅可比矩阵元素
	float sigma1=arm_sin_f32(phi3-phi2);
	float sigma2=arm_sin_f32(phi3-phi_4);
	float sigma3=arm_sin_f32(phi_1-phi2);
	float sigma4=arm_sin_f32(Leg->phi_0-phi3);
	float sigma5=arm_cos_f32(Leg->phi_0-phi3);
	float sigma6=arm_sin_f32(Leg->phi_0-phi2);
	float sigma7=arm_cos_f32(Leg->phi_0-phi2);
	
	(Leg->J_11)=(l_1*sigma4*sigma3)/sigma1;
	(Leg->J_12)=(l_4*sigma6*sigma2)/sigma1;
	(Leg->J_21)=(l_1*sigma5*sigma3)/((Leg->L0)*sigma1);
	(Leg->J_22)=(l_4*sigma7*sigma2)/((Leg->L0)*sigma1);
	
	//获取VMC逆解矩阵元素
	float sigma8=l_4*sigma2;
	float sigma9=l_1*sigma3;
	(Leg->T_11)=-sigma7/sigma9;
	(Leg->T_12)=sigma5/sigma8;
	(Leg->T_21)=(Leg->L0)*sigma6/sigma9;
	(Leg->T_22)=-(Leg->L0)*sigma4/sigma8;
	
	//获取dL0 ddL0 dphi0
	float sigma10=l_1*dphi_1;
	float sigma11=l_5/2.0f-XC;
	float sigma12=sigma10*arm_sin_f32(phi_1-phi3)+sigma8*dphi_4;
	float sigma13=sigma12/sigma1;
	float sigma14=sigma10*arm_cos_f32(phi_1)+sigma13*arm_cos_f32(phi2);
	float sigma15=sigma10*arm_sin_f32(phi_1)+sigma13*arm_sin_f32(phi2);
	(Leg->last_dL0)=(Leg->d_L0);
	(Leg->d_L0)=(YC*sigma14+sigma11*sigma15)/(Leg->L0);
	(Leg->dd_L0)=0.19f*(Leg->d_L0-Leg->last_dL0)/Balance_status.dt+0.81f*(Leg->dd_L0);//一阶低通滤波
	(Leg->dphi_0)=-(sigma14*sigma11-YC*sigma15)/(YC*YC+sigma11*sigma11);
}

/*
 *函数简介:支持力解算
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_GetFN(Leg_data *Leg)
{
	//z_ddot_w=z_ddot_M-L_ddot_0*cos(theta)+2*L_dot_0*theta_dot*sin(theta)+L_0*theta_ddot*sin(theta)+L_0*theta_dot^2*cos(theta)
	//F_N=P+m_w*g+m_w*z_ddot_w
	float COS=arm_cos_f32(Leg->theta);
	float SIN=arm_sin_f32(Leg->theta);
	
	(Leg->F)=(Leg->T_11)*(Leg->T1)+(Leg->T_12)*(Leg->T2);
	(Leg->Tp)=(Leg->T_21)*(Leg->T1)+(Leg->T_22)*(Leg->T2);
	float P=(Leg->F)*COS+(Leg->Tp)*SIN/(Leg->L0);
	
	float ddz_w=Balance_status.body_data.a_zE-(Leg->dd_L0)*COS+2.0f*(Leg->d_L0)*(Leg->dtheta)*SIN+(Leg->L0)*(Leg->ddtheta)*SIN+(Leg->L0)*(Leg->dtheta)*(Leg->dtheta)*COS;

	(Leg->Fn)=P+m_w*gravity+m_w*ddz_w;
}
