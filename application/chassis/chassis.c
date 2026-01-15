/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "Observer.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "LQR.h"
#include "user_lib.h"
#include "remote_control.h"
#include "Motion_Controller.h"
#include "Leg_Controller.h"
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t* referee_data; // 用于获取裁判系统的数据
 Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

   static Vision_Recv_s *vision_recv_data; //解决了HardFault


static SuperCapInstance *cap;                                       // 超级电容
static DMMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; //四个髋关节电机的实例
static DJIMotorInstance *motor_left ,*motor_right;  //左右足端轮电机实例

static Chassis_power_control_t power_control; // 底盘功率控制实例
/* 用于自旋变速策略的时间变量 */
// static float t;
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
// static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
// static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

//轮腿控制变量
float Targetdw; //  目标角速度
float Tl=0,Tpl=0  ,T1l=0,T2l=0,   Tr=0,Tpr=0,  T1r=0,T2r=0,   Fl=0,Fr=0;
float Yaw_WheelDelta_T=0;//转向控制所需要叠加在轮子上的力矩差  注意用算出来的总力矩差除以2分配到两个轮子上
float TargetX,TargetdX,Target_Yaw,TargetL0,TargetRoll,w_Limit=1.5f,YawTrack_Target;
uint8_t Chassis_Balance_Flag=0;
uint8_t Chassis_FirstFlag2=1;
uint8_t Chassis_Model=0;
uint8_t Chassis_YawFlag=0;
uint8_t Chassis_XTL_Flag=0; // 小陀螺模式标志位

#define SingleChassis		0//注释后Yaw跟云台同步

float test_motor_t_rf=-0.758;
float test_motor_t_lf=0.756;
float test_motor_t_rb=2.53;
float test_motor_t_lb=-2.57;

float test_speed_left=0;
float test_speed_right=0;

void ChassisInit()
{
    rc_data = RemoteControlInit(&huart5); //双板的时候删除
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口
    //髋关节电机初始化
    Motor_Init_Config_s chassis_DM_motor_config = {  
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = { 
            .angle_PID = {
                .Kp = 8,
                .Ki = 0,
                .Kd = 0,
                .IntegralLimit = 1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 3,
            },
            .speed_PID = {
                .Kp = 4, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20,
            },
            .current_PID = {
                .Kp = 0, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 0,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
        .motor_type = DM8009,
    };
    //轮电机初始化
    Motor_Init_Config_s chassis_DJI_motor_config = {  
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 2, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.

    // 1号和3号是左侧髋关节电机
    // 2号和4号是右侧髋关节电机
    chassis_DM_motor_config.can_init_config.tx_id = 0x01;
    chassis_DM_motor_config.can_init_config.rx_id = 0x11;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DMMotorInit(&chassis_DM_motor_config,DMMOTOR_MODE_MIT);

    chassis_DM_motor_config.can_init_config.tx_id = 0x02;
    chassis_DM_motor_config.can_init_config.rx_id = 0x12;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DMMotorInit(&chassis_DM_motor_config,DMMOTOR_MODE_MIT);

    chassis_DM_motor_config.can_init_config.tx_id = 0x03;
    chassis_DM_motor_config.can_init_config.rx_id = 0x13;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DMMotorInit(&chassis_DM_motor_config,DMMOTOR_MODE_MIT);

    chassis_DM_motor_config.can_init_config.tx_id = 0x04;
    chassis_DM_motor_config.can_init_config.rx_id = 0x14;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DMMotorInit(&chassis_DM_motor_config,DMMOTOR_MODE_MIT);


    chassis_DJI_motor_config.can_init_config.tx_id = 1;
    chassis_DJI_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_left = DJIMotorInit(&chassis_DJI_motor_config);

    chassis_DJI_motor_config.can_init_config.tx_id = 2;
    chassis_DJI_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_right = DJIMotorInit(&chassis_DJI_motor_config);

    referee_data = UITaskInit(&huart1,&ui_data); // 裁判系统初始化,会同时初始化UI   //删掉会导致Hardfault

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan3,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要cancomm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x52,
            .rx_id = 0x51,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}


/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    //首先把轮腿机器人的力矩分成四个部分



    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    //TODO: 改成力控
    // DJIMotorSetRef(motor_lf, vt_lf);
    // DJIMotorSetRef(motor_rf, vt_rf);
    // DJIMotorSetRef(motor_lb, vt_lb);
    // DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{ 
    RC_ctrl_t* rc_data=Get_rc_data();
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD
chassis_cmd_recv.chassis_mode = CHASSIS_NO_FOLLOW;
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DMMotorStop(motor_lf);
        DMMotorStop(motor_rf);
        DMMotorStop(motor_lb);
        DMMotorStop(motor_rb);
        DJIMotorStop(motor_left);
        DJIMotorStop(motor_right);
    }
    else
    { // 正常工作
        DMMotorEnable(motor_lf);
        DMMotorEnable(motor_rf);
        DMMotorEnable(motor_lb);
        DMMotorEnable(motor_rb);
        DJIMotorEnable(motor_left);
        DJIMotorEnable(motor_right);
    }

    // 根据控制模式设定旋转速度hcan1
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 4000;
        break;
    default:
        break;
    }

    //速度环控制  轮半径60mm  向前的速度转化为度/秒  电机转速=线速度/轮子半径
    // DJIMotorSetRef(motor_left,chassis_cmd_recv.vx/Wheel_R + chassis_cmd_recv.wz * Body_Rl);// m/s 正负号待定
    // DJIMotorSetRef(motor_right,chassis_cmd_recv.vx/Wheel_R - chassis_cmd_recv.wz * Body_Rl);

    test_speed_left=rc_data->rc.rocker_l1*30+rc_data->rc.rocker_r_*10;
    test_speed_right=rc_data->rc.rocker_l1*30-rc_data->rc.rocker_r_*10;
    DJIMotorSetRef(motor_left,-test_speed_left);// m/s 正负号待定
    DJIMotorSetRef(motor_right,-test_speed_right);// m/s 正负号待定
    // 根据控制模式进行正运动学解算,计算底盘输出
    //TODO：后续添加LQR计算
    // test_motor_t_lf=rc_data->rc.rocker_l1/660.0f*10;
    // test_motor_t_rf=rc_data->rc.rocker_r1/660.0f*10;

    // DMMotorSetRef(motor_rf,test_motor_t_rf);
    DMMotorSetRef(motor_rf,test_motor_t_rf);
    DMMotorSetRef(motor_rb,test_motor_t_rb);
    DMMotorSetRef(motor_lf,test_motor_t_lf);
    DMMotorSetRef(motor_lb,test_motor_t_lb);

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    // LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    // EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况

    //获得弹速限制和剩余热量，发送到云台
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;//弹速限制
    chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;//剩余热量
    chassis_feedback_data.cooling_rate=referee_data->GameRobotState.shooter_barrel_cooling_value;//枪口冷却速率
    chassis_feedback_data.cooling_limit=referee_data->GameRobotState.shooter_barrel_heat_limit;//枪口热量上限
    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}


void Balance_Control_Init(void)
{
	Leg_Controller_LegControlInit();  //pid参数
	Motion_Controller_Init(); 
}

// void Body_speed_control(float speed)  //m/s
// {
//    float wheel_speed_l = speed ;
//    float wheel_speed_l = speed ;

//    Chassis_Wheel_speed_Control(speed_l,speed_r);

// }

// void Chassis_Wheel_speed_Control(float speed_l ,float speed_r)
// {
//     //转矩常数*电流*减速比=转矩
//     //电流=转矩/(转矩常数*减速比)
    
//     //车体速度等于轮子线速度，转子线速度除以减速比


//     DJIMotorSetRef(motor_left ,speed_l);
//     DJIMotorSetRef(motor_right,speed_r);
// }

/*
 *函数简介:轮子力矩控制
 *参数说明:左轮力矩，右轮力矩
 *返回类型:无
 *备注:无
 */
void Chassis_Wheel_Control(float T_l ,float T_r)
{
    //转矩常数*电流*减速比=转矩
    //电流=转矩/(转矩常数*减速比)
    float Tau_l = T_l / CURRENT_TO_TORQUE / GEAR_RATIO ;  //目标力矩转化为目标电流
    float Tau_r = T_r / CURRENT_TO_TORQUE / GEAR_RATIO ;

    Tau_l= Tau_l / 20.0f * 16384; //映射到电机输入范围   
    Tau_r= Tau_r / 20.0f * 16384; 

    DJIMotorSetRef(motor_left,Tau_l);
    DJIMotorSetRef(motor_right,Tau_r);
}

void Chassis_MotorControl_Leg_init(float T1_l,float T2_l,float T1_r,float T2_r)
{
	//  float Tl,Tr;
	//  Warming_Brake(&Tl,&Tr);  //这里暂时不是很理解

	//可以把关节电机的使能放在这里

    //前腿T1 后腿T2
     DMMotorSetRef(motor_lb,T1_l);
     DMMotorSetRef(motor_lf,T2_l);
     DMMotorSetRef(motor_rb,T1_r);
     DMMotorSetRef(motor_rf,T2_r);
}

//收腿动作 
void Chassis_MotorControl(float T_l,float T1_l,float T2_l,float T_r,float T1_r,float T2_r)
{
     Chassis_Wheel_Control(T_l ,T_r);
     DMMotorSetRef(motor_lb,T1_l);
     DMMotorSetRef(motor_lf,T2_l);
     DMMotorSetRef(motor_rb,T1_r);
     DMMotorSetRef(motor_rf,T2_r);
}

//根据不同状态选择不同的控制方式
void Chassis_ModelControl(void)
{
	if(Chassis_Balance_Flag==0)  //如果还没有平衡
		Chassis_StandFromGround();
	else 
	{
		Chassis_ModelTwo();
	}
}


void Chassis_Control(void)
{
    RC_ctrl_t* rcdata = Get_rc_data();
    Balance_data* Balance_data=Get_Balance_Data();
   if(Chassis_Balance_Flag==1)    //已经平衡
	{
/*====================变腿长====================*/
		// if(switch_is_down(rcdata[TEMP].rc.switch_left) && Chassis_Model==3)   //摇杆控制
		// {
		// 	// if(rcdata.Remote_ThumbWheel>1024+50)TargetL0+=0.3f*Observer_BalanceStatus.dt;
		// 	// else if(rcdata.Remote_ThumbWheel<1024-50)TargetL0-=0.3f*Observer_BalanceStatus.dt;    
		// }
/*====================Roll控制====================*/
	TargetRoll=0;    //暂时不考虑这个功能
/*====================平移&旋转====================*/
	TargetdX=rcdata->rc.rocker_l1/660.0f/2 ;  //左竖 控制速度(m/s)		
    TargetX+=TargetdX*Balance_data->dt;

	Targetdw=rcdata->rc.rocker_r_ /660.0f/2; //右横 控制角速度 
	w_Limit=1.5f;

	Target_Yaw-=Targetdw*Balance_data->dt;
    //TODO：增加小陀螺和跟随云台功能
    
    
    }
    /*====================控制====================*/
	Chassis_ModelControl();  
}



void Chassis_StandFromGround(void)
{
    Balance_data* Balance_status=Get_Balance_Data();  
    RC_ctrl_t* rc_data = Get_rc_data(); 
    float Tl=0,T1l=0,T2l=0,Tr=0,T1r=0,T2r=0;

  	static uint16_t Shoutui_Count=0;   //收腿计数
	static uint16_t Balance_Count=0;   //平衡计数

/*====================关节力矩====================*/
    float L0_l=0.10f;
    float L0_r=0.10f;  //让腿部长度收缩到最短  
    //TODO:大轮腿最短长度待测

    float phi1l,phi4l,phi1r,phi4r; 
    //计算L0长度最短并且身体竖直时的腿部角度
    Leg_Controller_InverseKinematicsSolution(L0_l,PI/2.0f,&phi1l,&phi4l);
	Leg_Controller_InverseKinematicsSolution(L0_r,PI/2.0f,&phi1r,&phi4r);
    //计算髋关节力矩
	Leg_Controller_LengthLQR(Balance_status->Leg_L,phi1l,phi4l,&T1l,&T2l);
	Leg_Controller_LengthLQR(Balance_status->Leg_R,phi1r,phi4r,&T1r,&T2r);
    
/*====================轮向力矩====================*/

   float Motion_YAW_Control_Left_Dleta_T,Motion_YAW_Control_Right_Dleta_T;
   if(Chassis_YawFlag==0)Motion_Controller_Yaw_Control(Target_Yaw,&Motion_YAW_Control_Left_Dleta_T,&Motion_YAW_Control_Right_Dleta_T,w_Limit);
	else Motion_Controller_Yaw_Control_Follow(YawTrack_Target,&Motion_YAW_Control_Left_Dleta_T,&Motion_YAW_Control_Right_Dleta_T,w_Limit);

   //T=LQR_T+偏航角控制T
	Tl=Tl+Motion_YAW_Control_Left_Dleta_T;
	Tr=Tr+Motion_YAW_Control_Right_Dleta_T;

/*====================收腿检测====================*/  
//调整腿部到初始位置的检测
	#define Shoutui_AngleThreshold		0.18f
	#define Shoutui_SpeedThreshold		0.04f
	if(fabs(phi1l-Balance_status->Leg_L.phi_1)<Shoutui_AngleThreshold && fabs(phi4l-Balance_status->Leg_L.phi_4)<Shoutui_AngleThreshold \
	   && fabs(phi1r-Balance_status->Leg_R.phi_1)<Shoutui_AngleThreshold && fabs(phi4r-Balance_status->Leg_R.phi_4)<Shoutui_AngleThreshold \
	   && fabs(Balance_status->Leg_L.dphi_1)<Shoutui_SpeedThreshold && fabs(Balance_status->Leg_L.dphi_4)<Shoutui_SpeedThreshold \
	   && fabs(Balance_status->Leg_R.dphi_1)<Shoutui_SpeedThreshold && fabs(Balance_status->Leg_R.dphi_4)<Shoutui_SpeedThreshold)
		Shoutui_Count++;    //当前角度 与 L0等于0.1时的角度 相差小于阈值的时间超过200时 ，认为收腿完成
	else
		Shoutui_Count=0;
	
	if(Shoutui_Count>200)
	{
		Shoutui_Count=0;
		Chassis_FirstFlag2=0;
	}
	
	/*====================电机控制====================*/

#define Stand_x		-0.0f

if(Chassis_FirstFlag2==1) //腿部位置初始化阶段只有关节电机动
	{
        //先不加滤波在平地上调
		// Observer_BalanceStatus.Body.MotionEstimation.x_nofilter=Stand_x;
		// Observer_BalanceStatus.Body.MotionEstimation.x=Stand_x;
		// Observer_BalanceStatus.Body.x_nofilter=Stand_x;
		Balance_status->body_data.x=Stand_x;
		
		Chassis_MotorControl_Leg_init(T1l,T2l,T1r,T2r);
	}
    else  //收腿之后轮向电机发力
		Chassis_MotorControl(Tl,T1l,T2l,Tr,T1r,T2r);

	/*====================超时检测====================*/
	static uint16_t TimeOUT_Count=0;
	TimeOUT_Count++;
	if(TimeOUT_Count>5000)
	{
		TimeOUT_Count=0;
		// Chassis_Reset();
	}
	
/*====================平衡检测====================*/

#define Pingheng_PitchThreshold		(15.0f/180.0f*PI) 
//判断平衡的俯仰角阈值 15度

if(Balance_status->body_data.Pitch<Pingheng_PitchThreshold &&Balance_status->body_data.Pitch>-Pingheng_PitchThreshold)
		Balance_Count++;
	else
		Balance_Count=0;
	
	if(Balance_Count>=500)//计数500认为达到平衡状态
	{
	    Balance_Count=0;
		TimeOUT_Count=0;
		Chassis_Balance_Flag=1; 
        Chassis_Model=rc_data->rc.switch_right;//通过遥控器右拨开关选择平衡后的控制模式
			if(Chassis_Model==2) TargetL0=0.15f;
			else TargetL0=0.2f;

            // Observer_BalanceStatus.Body.MotionEstimation.x_nofilter=Blance_X;
			// Observer_BalanceStatus.Body.MotionEstimation.x=Blance_X;
			// Observer_BalanceStatus.Body.x_nofilter=Blance_X;
			Balance_status->body_data.x=0;
    }

}

void Chassis_ModelTwo(void)
{
    Tl=Tpl=T1l=T2l=Tr=Tpr=T1r=T2r=Fl=Fr=0;
    Balance_data* Balance_status=Get_Balance_Data();
    RC_ctrl_t* rcdata =Get_rc_data();
/*====================LQR建模====================*/
	LQR_Clc(&Tl,&Tpl,&Tr,&Tpr,TargetX,0);  

/*====================关节力矩====================*/
//翻滚角补偿->L0 F  
float Roll_Compensate_L0_Right=0,Roll_Compensate_L0_Left=0;
float Roll_Conpensate_F_Right=0,Roll_Conpensate_F_Left=0;

Motion_Controller_Roll_Control(TargetRoll,&Roll_Compensate_L0_Left,&Roll_Compensate_L0_Right,&Roll_Conpensate_F_Left,&Roll_Conpensate_F_Right);

//腿长控制->F
float Leg_Length_Control_Left_F=0,Leg_Length_Control_Right_F=0;
               //左腿部分
Left_Leg_Pid.Need_Value=TargetL0+Roll_Compensate_L0_Left;
float_constrain(Left_Leg_Pid.Need_Value,0.1,0.3); //TODO:待测  腿长限幅 
if(Balance_status->Leg_L.Fn<FN_Threshold) Left_Leg_Pid.Need_Value=0.22;//离地状态
               //右腿部分
Right_Leg_Pid.Need_Value=TargetL0+Roll_Compensate_L0_Right;
float_constrain(Right_Leg_Pid.Need_Value,0.1,0.3); //腿长限幅
if(Balance_status->Leg_R.Fn<FN_Threshold) Right_Leg_Pid.Need_Value=0.22;

Leg_Controller_LegControl(&Leg_Length_Control_Left_F,&Leg_Length_Control_Right_F);

//双腿协调->Tp
float Leg_Coordinate_Left_Tp=0,Leg_Coordinate_Right_Tp=0;
Motion_Controller_LegCoordination_Control(&Leg_Coordinate_Left_Tp,&Leg_Coordinate_Right_Tp);

//F=Mg/cos(theta)+腿长控制F+翻滚角补偿F
Fl= m_bodyleg * gravity / arm_cos_f32(PI/2-Balance_status->Leg_L.phi_0)+Roll_Conpensate_F_Left+Leg_Length_Control_Left_F;
Fr= m_bodyleg * gravity / arm_cos_f32(PI/2-Balance_status->Leg_R.phi_0)+Roll_Conpensate_F_Right+Leg_Length_Control_Right_F; 

if(Balance_status->Leg_L.Fn>=FN_Threshold) //触地状态 
Tpl=Tpl+Leg_Coordinate_Left_Tp;
else Fl=-10+Leg_Length_Control_Left_F;//减去一个常数防止悬空时腿部电机死锁 
if(Balance_status->Leg_R.Fn>=FN_Threshold)
Tpr=Tpr+Leg_Coordinate_Right_Tp;
else Fr=-10+Leg_Length_Control_Right_F;

Leg_Controller_VMC(Balance_status->Leg_L,Fl,Tpl,&T1l,&T2l);
Leg_Controller_VMC(Balance_status->Leg_R,Fr,Tpr,&T1r,&T2r);

	/*====================轮向力矩====================*/

    //偏航角控制->T
	float Locomotion_Controller_LeftWheelDeltaT1=0,Locomotion_Controller_RightWheelDeltaT1=0;
	#ifndef SingleChassis  
		if(Chassis_XTL_Flag==1) //开启小陀螺模式
		{
			if(Remote_RxData.Remote_Mouse_KeyR==1)
				Motion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
			else
			{
				Motion_Controller_Yaw_Control_Follow(YawTrack_Target,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
				Target_Yaw=Balance_status->body_data.Yaw;
			}
		}
		else
		{
			Motion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
		}
	#else
		Motion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
	#endif
    //T=LQR_T+偏航角控制T
	if(Balance_status->Leg_L.Fn>=FN_Threshold)Tl=Tl+Locomotion_Controller_LeftWheelDeltaT1;
	if(Balance_status->Leg_R.Fn>=FN_Threshold)Tr=Tr+Locomotion_Controller_RightWheelDeltaT1;

    	/*====================电机控制====================*/
	  Chassis_MotorControl(Tl,T1l,T2l,Tr,T1r,T2r);
      /*====================模式切换====================*/
	if(switch_is_down(rcdata[TEMP].rc.switch_left)) Chassis_Model=rcdata->rc.switch_right;
}

//复位代码  待完善
void Chassis_Reset(void)
{}
//跳跃代码 待完善
void Chassis_ModelJump(void)
{}









static inline uint8_t floatequal(float a, float b)
{
    return fabs(a - b) < 1e-5;
}


/**
 * @brief 底盘功率控制模块,根据裁判系统的功率数据和超级电容的剩余能量对底盘输出进行功率限制
 *        先写一个不考虑超级电容的版本
 */
static void ChassisPowerControl()
{
    Balance_data * Balance_status;
    float Chassis_target_speed; //底盘目标速度
    float Chassis_target_x; //底盘目标位移
    float Chassis_current_speed; //底盘当前速度
    float Uspeed;               //速度和位移控制的叠加控制量
    float Uyaw;                 //yaw控制所需要叠加在轮电机上的力矩
    float wLeftWheel;           //左轮角速度
    float leftUelse;            //左轮用于控制theta和phi所需的力矩
    float wRightWheel;          //右轮角速度
    float rightUelse;           //右轮用于控制theta和phi所需的力矩

    Chassis_target_speed=TargetdX; 
    Chassis_target_x=TargetX;
    Chassis_current_speed= Balance_status->body_data.dx; 
    Uspeed=Get_Uspeed(Chassis_target_x,Chassis_target_speed);  //左右轮暂时用一个变量，待验证正负号关系
    Uyaw=Yaw_WheelDelta_T;
    wLeftWheel=Balance_status->Leg_L.wheel.speed; //rad/s
    wRightWheel=Balance_status->Leg_R.wheel.speed; //rad/s
    leftUelse=Get_Uelse_L();
    rightUelse=Get_Uelse_R();

    //获取当前功率状态
    power_control.power_limit=referee_data->GameRobotState.chassis_power_limit; //最大功率限制
    //float chassis_power=// 裁判系统不返回底盘功率

    if(floatequal(Uyaw,0.0f))
    {
        if(Uyaw*Uspeed>0) //同向
        {
            power_control.K=10000.0f;
        }
        else //反向
        {
             power_control.K=-10000.0f;
        }
    }
    else  power_control.K=Uspeed/Uyaw;  //得到比例系数


    float Total_T_left=leftUelse + Uspeed + Uyaw; //左轮总力矩
    float Total_T_right=rightUelse + Uspeed - Uyaw; //右轮总力矩     Uyaw正负号关系待验证

    //计算功率估计值
    power_control.Estimated_Power= Total_T_left * wLeftWheel + power_control.K1 * fabs(wLeftWheel) + power_control.K2 * Total_T_left * Total_T_left + power_control.K3
                             + Total_T_right * wRightWheel + power_control.K1 * fabs(wRightWheel) + power_control.K2 * Total_T_right * Total_T_right + power_control.K3;

    if(power_control.Estimated_Power< power_control.power_limit) //当估计功率小于功率限制
    {
        //正常控制
        power_control.restrictedUspeed=Uspeed;
        power_control.restrictedUyaw=Uyaw;
        power_control.decayUspeed=1.0;
        power_control.decayUyaw=1.0;
    }
    else  //关键部分：关于预测功率大于限制功率的处理
    {
        float speed_error = Chassis_target_speed-Chassis_current_speed; // 计算速度误差
        if(fabs(speed_error)>0.5f&&fabs(Chassis_current_speed)>0.5f&&(((Chassis_current_speed*Chassis_target_speed)<0.0f)||(Chassis_target_speed >= 0.0f && speed_error < 0.0f) ||
             (Chassis_target_speed<= 0.0f && speed_error > 0.0f)))
             {
                 
            //当底盘处于减速/制动阶段时，机械功率变得很小或为负（发电），
            //此时总功率不会超限，因此应逐渐放松限制，使 decay 系数趋近于 1。
            //这与反电动势导致功率下降有关，但核心原因是“主动制动阶段不会超功率”。
            power_control.decayUspeed = power_control.decayUspeed * 0.97f + 1.0f * 0.03f;
            power_control.decayUyaw   = power_control.decayUyaw * 0.97f + 1.0f * 0.03f;
             }
        else
        {
            //将轮腿的力矩带入电机功率模型方程得到一个关于Uyaw的二次方程   
            float A = (power_control.K2 * (2.0f * powf(power_control.K, 2) + 2.0f));
            float B = (2.0f * power_control.K2 * (power_control.K - 1.0f) * leftUelse) + (2.0f * power_control.K2 * (power_control.K + 1.0f) * rightUelse) +
                        (wLeftWheel * (power_control.K - 1.0f)) + (wRightWheel * (power_control.K + 1.0f));
            float C = (wLeftWheel * leftUelse) + (wRightWheel * rightUelse) +
                        (power_control.K1 * (fabs(wLeftWheel) + fabs(wRightWheel))) +
                        (power_control.K2 * (powf(leftUelse, 2) + powf(rightUelse, 2))) + power_control.K3 - power_control.power_limit;

            power_control.Delta = powf(B, 2) - 4 * A * C;   //B^2-4AC

            if( power_control.Delta==0)  //重根情况
            {
                float temp_Uyaw=-B/(2*A); //求此时的二次函数极值

                if(temp_Uyaw*Uyaw>0.0f)  //不能因为功率控制而改变转向的方向
                {
                    power_control.restrictedUyaw=temp_Uyaw;
                }
                else
                {
                    power_control.restrictedUyaw=0;
                }

                power_control.restrictedUspeed=power_control.K*power_control.restrictedUyaw;  //之前规定过一定比例关系
            }
            else if (power_control.Delta>0) //不等根情况
            {
                 //计算出两个解
                float tempUyaw1 = (-B - sqrtf(power_control.Delta)) / (2.0f * A); 
                float tempUyaw2 = (-B + sqrtf(power_control.Delta)) / (2.0f * A);
                /*
                解出的两个根表示：
                满足功率极限的两个可能的 yaw 控制量
                但运动方向不允许变化→ 所以必须选择与原 Uyaw 同号的根
                如果两个根都同号 → 选择更“接近原指令”的那个（更大或更小）
                如果没有同号 → 直接降到 0（完全禁掉 yaw）*/
                if (tempUyaw1 * Uyaw > 0.0f && tempUyaw2 * Uyaw > 0.0f) //两个解都与原yaw力矩同向
                {
                    if (Uyaw > 0.0f)//原yaw力矩为正
                        power_control.restrictedUyaw = fmax(tempUyaw1, tempUyaw2);
                    else //原yaw力矩为负
                        power_control.restrictedUyaw = fmin(tempUyaw1, tempUyaw2); 
                } 
                else if (tempUyaw1 * Uyaw > 0.0f) //只有第一个解与原yaw力矩同向
                    power_control.restrictedUyaw = tempUyaw1;
                else if (tempUyaw2 * Uyaw > 0.0f) //只有第二个解与原yaw力矩同向
                    power_control.restrictedUyaw = tempUyaw2;
                else //两个解都与原yaw力矩反向
                    power_control.restrictedUyaw = 0.0f;   

                power_control.restrictedUspeed = power_control.K * power_control.restrictedUyaw;
            }
            else if (power_control.Delta<0)  //虚根情况
            {
                float tempUyaw = (-B) / (2.0f * A);//取实部

                if (tempUyaw * Uyaw > 0.0f) 
                    power_control.restrictedUyaw = tempUyaw;
                else
                    power_control.restrictedUyaw = 0.0f;

                power_control.restrictedUspeed = power_control.K * power_control.restrictedUyaw;
            }
              //计算衰减系数
            float tempDecayUspeed = power_control.restrictedUspeed / Uspeed; 
            float tempDecayUyaw   = power_control.restrictedUyaw / Uyaw; 
            //限制衰减系数在0.01到1.00之间
            tempDecayUspeed = float_constrain(tempDecayUspeed, 0.01f, 1.00f); 
            tempDecayUyaw   = float_constrain(tempDecayUyaw, 0.01f, 1.00f);

            power_control.decayUspeed = tempDecayUspeed * 0.1f + power_control.decayUspeed * 0.9f; 
            power_control.decayUyaw   = tempDecayUyaw * 0.1f + power_control.decayUyaw * 0.9f;     
        }     
    }
    float restrictedLeftTotalTorque  = power_control.restrictedUspeed - power_control.restrictedUyaw + leftUelse; //左轮受限总力矩
    float restrictedRightTotalTorque = power_control.restrictedUspeed + power_control.restrictedUyaw + rightUelse; //右轮受限总力矩

    power_control.restrictedPower = restrictedLeftTotalTorque * wLeftWheel + power_control.K1 * fabs(wLeftWheel) + power_control.K2 * restrictedLeftTotalTorque * restrictedLeftTotalTorque + power_control.K3
                                  + restrictedRightTotalTorque * wRightWheel + power_control.K1 * fabs(wRightWheel) + power_control.K2 * restrictedRightTotalTorque * restrictedRightTotalTorque + power_control.K3;
}