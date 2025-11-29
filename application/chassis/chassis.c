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
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DMMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; //四个髋关节电机的实例
static DJIMotorInstance *motor_left ,*motor_right;  //左右足端轮电机实例

static Chassis_power_control_t power_control; // 底盘功率控制实例
/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
// static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

//轮腿控制变量
float Target_dx; 
float Target_x;
float Target_yaw;
float Target_roll;
float Target_L0;
float w_Limit=2.5f;
float YawTrack_Target;
float Tl=0,Tpl=0  ,T1l=0,T2l=0,   Tr=0,Tpr=0,  T1r=0,T2r=0,   Fl=0,Fr=0;
float Yaw_WheelDelta_T=0;//转向控制所需要叠加在轮子上的力矩差  注意用算出来的总力矩差除以2分配到两个轮子上


void ChassisInit()
{
    //髋关节电机初始化
    Motor_Init_Config_s chassis_DM_motor_config = {  
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = { 
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
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
        .motor_type = DM8009,
    };
    //轮电机初始化
    Motor_Init_Config_s chassis_DJI_motor_config = {  
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
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

    //1号和3号是左侧髋关节电机
    //2号和4号是右侧髋关节电机
    chassis_DM_motor_config.can_init_config.tx_id = 1;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DMMotorInit(&chassis_DM_motor_config);

    chassis_DM_motor_config.can_init_config.tx_id = 2;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DMMotorInit(&chassis_DM_motor_config);

    chassis_DM_motor_config.can_init_config.tx_id = 3;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DMMotorInit(&chassis_DM_motor_config);

    chassis_DM_motor_config.can_init_config.tx_id = 4;
    chassis_DM_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DMMotorInit(&chassis_DM_motor_config);


    chassis_DJI_motor_config.can_init_config.tx_id = 1;
    chassis_DJI_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_left = DJIMotorInit(&chassis_DJI_motor_config);

    chassis_DJI_motor_config.can_init_config.tx_id = 2;
    chassis_DJI_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_right = DJIMotorInit(&chassis_DJI_motor_config);

    referee_data = UITaskInit(&huart1,&ui_data); // 裁判系统初始化,会同时初始化UI

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
            .can_handle = &hcan3,
            .tx_id = 0x311,
            .rx_id = 0x312,
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
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

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

    // 根据控制模式设定旋转速度
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

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    //根据控制模式进行正运动学解算,计算底盘输出
    //TODO：后续添加LQR计算

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况

    //获得弹速限制和剩余热量，发送到云台
    chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;//弹速限制
    chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;//剩余热量
    chassis_feedback_data.cooling_rate=referee_data->GameRobotState.shooter_id1_17mm_cooling_rate;//枪口冷却速率
    chassis_feedback_data.cooling_limit=referee_data->GameRobotState.shooter_id1_17mm_cooling_limit;//枪口热量上限
    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}



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

    Chassis_target_speed=Target_dx; 
    Chassis_target_x=Target_x;
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