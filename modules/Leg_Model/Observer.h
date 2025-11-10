#ifndef OBSERVER_H
#define OBSERVER_H

#include "main.h"
#include "MotionEstimation.h"

#define DEG_TO_RAD PI/180;
//物理参数
#define Wheel_R  0.03     //轮半径
#define L1 0.105   //大腿长度
#define L2 0.125   //小腿长度
#define gravity  9.80665f

#define m_w	  0.056   //轮子重量
#define Rl	  0//0.463f/2.0f //机身宽度


typedef struct 
{ 
   float x;  //位移(m)
   float speed;  //(rad/s)
   float angle; //(rad)
   float T;    //转矩(N)

   // int Reversal_flag;  //接收信息反转标志位置
}Wheel_data;

typedef struct  
{
   Wheel_data wheel;  //轮子数据

   float theta_1;   //大腿与水平面的夹角（rad）
   float dtheta_1; //髋关节电机角速度（rad/s）
   float small_rod_angle ;  //与中心轴相连的小连杆与水平面的夹角，用于计算小腿与水平面的夹角
   float d_small_rod_angle ; 
   float theta_2;   //小腿与水平面的夹角（rad）  //这个需要计算得出
   float dtheta_2; //小腿电机角速度（rad/s）

   float L0;        //等效杆长(m)
   float last_dL0;   //上一时刻的等效杆长(m)
   float d_L0;      //杆长变化率(m/s)
   float dd_L0;     //杆长二阶变化率(m/s2)

   float phi_0;     //等效摆杆与水平面的夹角
   float last_dphi_0;	
   float dphi_0;
   float ddphi_0;    

   float theta;				//摆杆摆角theta(rad)
	float last_dtheta;			//上一次摆杆摆角角速度last_theta(rad/s) 
	float dtheta;				//摆杆摆角角速度dtheta(rad/s)
	float ddtheta;				//摆杆摆角角加速度ddtheta(rad/s^2)

   float Tp1;   //髋关节电机扭矩
   float Tp2;   //膝关节电机扭矩 （具体的转换还得看一下）

   float Fn;    //支持力
   float Tp;    //摆杆扭力（VMC解算中垂直摆杆的力）
   float F;	    //沿摆杆方向的力（推力）

   float J_11;					//VMC雅可比矩阵J元素
	float J_12;					//VMC雅可比矩阵J元素
	float J_21;					//VMC雅可比矩阵J元素
	float J_22;					//VMC雅可比矩阵J元素
	
	float T_11;					//VMC逆解矩阵T元素
	float T_12;					//VMC逆解矩阵T元素
	float T_21;					//VMC逆解矩阵T元素
	float T_22;					//VMC逆解矩阵T元素
}Leg_data;

typedef struct  
{
  float Roll;
  float Yaw;
  float Pitch;
  
  float d_Roll;
  float d_Yaw;
  float d_pitch;
  
  float a_xE;									//世界坐标系x轴加速度(m/s^2)
  float a_yE;									//世界坐标系y轴加速度(m/s^2)
  float a_zE;									//世界坐标系z轴加速度(m/s^2)

  float a_xb;									//机体坐标系x轴加速度(m/s^2)
  float a_yb;									//机体坐标系y轴加速度(m/s^2)
  float a_zb;									//机体坐标系z轴加速度(m/s^2)
	
  float x;
  float dx;

  float z_b_ddot;

  	MotionEstimation_Balance MotionEstimation;	//运动估计结构体
}Body_data;

typedef struct  
{
   float dt; //控制周期
 
   float feedforward; //根据机体重量和当前phi0进行重力前馈

   Body_data  body_data; //机体数据

   Leg_data Leg_R;   //右腿数据
   Leg_data Leg_L;   //左腿数据
}Balance_data;//轮腿各部分的数据

void Observer_init(void);

void Observer_DataGet(void);
void Observer_LegForwardKinematicsSolution(Leg_data *Leg);
float calculate_phi3_dot( float l2, float phi2, float phi3, float dxB_dt, float dyB_dt, float dxD_dt, float dyD_dt) ;
float calculate_theta2_dot_right();
void Observer_GetFN(Leg_data *Leg);
Balance_data*  Get_Balance_Data();


#endif 
