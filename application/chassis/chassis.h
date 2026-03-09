#ifndef CHASSIS_H
#define CHASSIS_H

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();


// 底盘功率控制结构体
typedef struct 
{ 
    float power_limit;        // 当前功率限制值
    float K;      // 速度控制力矩与yaw控制力矩的比例系数
    float K1;  //电机功率模型中的系数
    float K2;  //电机功率模型中的系数
    float K3;  //电机功率模型中的系数
    float Estimated_Power;  // 估计功率    // 法一： P=tau*omega + K1*|omega| + K2*tau^2 + K3     法二：P=tau*omega+K1*omega^2+K2*tau^2+K3 

    float restrictedUspeed;  //限制后的速度控制功率
    float restrictedUyaw;    //限制后的yaw控制功率

    float decayUspeed ;  //速度控制力矩衰减系数
    float decayUyaw   ;  //yaw控制力矩衰减系数

    float Delta ;

    float restrictedPower; //限制过后的功率

}Chassis_power_control_t;

void Chassis_StandFromGround(void);
void Chassis_ModelTwo(void);
void Chassis_Control(void);
void Chassis_Wheel_Control(float T_l ,float T_r);
void Chassis_MotorControl_Leg_init(float T1_l,float T2_l,float T1_r,float T2_r);
void Chassis_Reset(void);
#endif // CHASSIS_H