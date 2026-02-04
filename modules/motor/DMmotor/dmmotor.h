#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

// TODO: 根据电机手册修改下面的最小最大值
#define DM_P_MIN  (-12.56f)
#define DM_P_MAX  12.56f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-54.0f)
#define DM_T_MAX   54.0f

typedef enum
{
    DMMOTOR_MODE_MIT = 1,
    DMMOTOR_MODE_POS_VEL,
    DMMOTOR_MODE_VEL,
    DMMOTOR_MODE_FORCE_POS          // 力位混控模式
}DMMotor_WorkMode_e;

/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
// 后续增加前馈数据指针

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
}DM_Motor_Measure_s;


typedef struct
{
    struct
    {
        uint16_t position_des;
        uint16_t velocity_des;
        uint16_t torque_des;
        uint16_t Kp;
        uint16_t Kd;
    }MIT;
    struct
    {
        float position_des;
        float velocity_des;
    }POS_VEL;
    struct
    {
        float velocity_des;
    }VEL;
    struct
    {
        float position_des;
        uint16_t velocity_des;
        uint16_t torque_des;
    }FORCE_POS;
}DMMotor_Send_s;

typedef struct 
{
    DMMotor_WorkMode_e work_mode;
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    DMMotor_Send_s dm_send;
    // PIDInstance current_PID;
    // PIDInstance speed_PID;
    // PIDInstance angle_PID;
    // float *other_angle_feedback_ptr;
    // float *other_speed_feedback_ptr;
    // float *speed_feedforward_ptr;
    // float *current_feedforward_ptr;
    // float pid_ref;
    Motor_Controller_s motor_controller;
    
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;


void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
void DMMotorTask(void const *argument);
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config ,DMMotor_WorkMode_e mode);
void DMMotorSetPosVel(DMMotorInstance *motor, float pos,float vel);
void DMMotorSetVel(DMMotorInstance *motor, float vel);

void DMMotorPosControl_MIT(DMMotorInstance *motor, float position,float kp);
void DMMotorVelControl_MIT(DMMotorInstance *motor, float velocity,float kd);
DMMotorInstance ** Get_DM_Instance();
#endif // !DMMOTOR