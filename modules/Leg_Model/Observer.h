#ifndef OBSERVER_H
#define OBSERVER_H

#include "main.h"
#include "MotionEstimation.h"
#define DEG_TO_RAD PI/180.0f
//物理参数
#define Wheel_R  0.03     //轮半径
#define L1 0.105   //大腿长度
#define L2 0.125   //小腿长度
#define gravity  9.80665f

#define m_w	  0.056   //轮子重量
#define m_bodyleg   2.139      //除去轮子后机体的重量(kg)  

#define Rl	  0.162//0.324f/2.0f //机身宽度

#define l_1													0.105f 
#define l_2													0.125f
#define l_3													0.125f
#define l_4													0.105f 
#define l_5													0


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
	Wheel_data Wheel;	//轮子状态
	
	float phi_1;				//五连杆phi1(rad)
	float phi_4;				//五连杆phi4(rad)
	float dphi_1;				//五连杆dphi1(rad/s)
	float dphi_4;				//五连杆dphi4(rad/s)
	
	float L_0;					//摆杆长度L0(m)
	float last_dL_0;			//上一次摆杆长度last_L0(m)
	float dL_0;					//摆杆长度变化率dL0(m/s)
	float ddL_0;				//摆杆长度二阶变化率ddL0(m/s^2)
	
	float phi_0;				//摆杆角度phi0(rad)
	float dphi_0;				//摆杆角速度dphi0(rad/s)
	
	float T1;					//髋关节1的转矩T1(N·m)
	float T2;					//髋关节2的转矩T2(N·m)
	
	float theta;				//摆杆摆角theta(rad)
	float last_dtheta;			//上一次摆杆摆角last_theta(rad)
	float dtheta;				//摆杆摆角角速度dtheta(rad/s)
	float ddtheta;				//摆杆摆角角加速度ddtheta(rad/s^2)
	
	float F;					//摆杆推力F(N)
	float Tp;					//摆杆扭矩Tp(N·m)
	float FN;					//支持力FN(N)
	
	float J_11;					//VMC雅可比矩阵J元素
	float J_12;					//VMC雅可比矩阵J元素
	float J_21;					//VMC雅可比矩阵J元素
	float J_22;					//VMC雅可比矩阵J元素
	
	float T_11;					//VMC逆解矩阵T元素
	float T_12;					//VMC逆解矩阵T元素
	float T_21;					//VMC逆解矩阵T元素
	float T_22;					//VMC逆解矩阵T元素
}Observer_LegStatus;

typedef struct  
{
   Wheel_data wheel;  //轮子数据

   float theta_1;   //大腿与水平面的夹角（rad）
   float dtheta_1; //髋关节电机角速度（rad/s）                                              逆时针为正
   float small_rod_angle ;  //与中心轴相连的小连杆与水平面的夹角，用于计算小腿与水平面的夹角       
   float d_small_rod_angle ; 
   float theta_2;   //小腿与水平面的夹角（rad）  //这个需要计算得出
   float dtheta_2; //小腿电机角速度（rad/s）                                                逆时针为正

   float L0;        //等效杆长(m)
   float last_dL0;   //上一时刻的等效杆长(m)
   float d_L0;      //杆长变化率(m/s)                                                      伸长为正
   float dd_L0;     //杆长二阶变化率(m/s2)

   float phi_0;     //等效摆杆与水平面的夹角
   float last_dphi_0;	
   float dphi_0;                                                                       //逆时针为正
   float ddphi_0;    

   //暂时没搞懂山海机甲为什么要有这个，和phi有什么区别  ————————  一个是与水平方向的夹角，一个是与竖直方向的夹角
   float theta;				//摆杆摆角theta(rad)
	float last_dtheta;			//上一次摆杆摆角角速度last_theta(rad/s) 
	float dtheta;				//摆杆摆角角速度dtheta(rad/s)                                 //顺时针为正
	double ddtheta;				//摆杆摆角角加速度ddtheta(rad/s^2)

   float Tp1;   //髋关节电机扭矩                                                         逆时针为正
   float Tp2;   //膝关节电机扭矩 （具体的转换还得看一下）                                    逆时针为正

   float Fn;    //支持力
   float Tp;    //摆杆扭力（VMC解算中垂直摆杆的力）                                         顺时针为正（不是很确定）
   float F;	    //沿摆杆方向的力（推力）                                     

   float J_11;					//VMC雅可比矩阵J元素
	float J_12;					//VMC雅可比矩阵J元素
	float J_21;					//VMC雅可比矩阵J元素
	float J_22;					//VMC雅可比矩阵J元素
	
	float T_11;					//VMC逆解矩阵T元素
	float T_12;					//VMC逆解矩阵T元素
	float T_21;					//VMC逆解矩阵T元素
	float T_22;					//VMC逆解矩阵T元素

//方案二用
   float phi_1;				//五连杆phi1(rad)
	float phi_4;				//五连杆phi4(rad)
	float dphi_1;				//五连杆dphi1(rad/s)
	float dphi_4;				//五连杆dphi4(rad/s)

   float T1;					//髋关节1的转矩T1(N·m)  小连杆电机
	float T2;					//髋关节2的转矩T2(N·m)  大腿电机

   float test;
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
void Observer_GetFN(Leg_data *Leg);
Balance_data*  Get_Balance_Data();

#endif 
