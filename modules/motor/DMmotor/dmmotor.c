#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id

    uint16_t id_offset=0;
    switch(motor->work_mode)
    {
    case DMMOTOR_MODE_POS_VEL:
        id_offset = 0x100;
        break;
    case DMMOTOR_MODE_VEL:
        id_offset = 0x200;
        break;
    case DMMOTOR_MODE_FORCE_POS:
        id_offset = 0x300;
        break;
    case DMMOTOR_MODE_MIT:
        id_offset= 0;
        break;
    default:
        
        break;
    }
    motor->motor_can_instace->tx_id=motor->motor_can_instace->tx_id + id_offset;
    // CANSetDLC(motor->motor_can_instace,8);
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{

    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->state= (rxbuff[0]&0xF0)>>4;
    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config,DMMotor_WorkMode_e mode)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->work_mode = mode;

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    // CAN回调设置
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);

    DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
    DWT_Delay(0.05);

    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);       //使能
    //DMMotorCaliEncoder(motor);
    DWT_Delay(0.05);
    dm_motor_instance[idx++] = motor;
    return motor;
}

//MIT模式设置目标值
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if(motor->work_mode != DMMOTOR_MODE_MIT)
    {
        DMMotorStop(motor);
        LOGERROR("DMMotorSetRef: Not implemented mode");
        return;
    }
    motor->motor_controller.pid_ref = ref;
}

//位置速度模式设置目标值
void DMMotorSetPosVel(DMMotorInstance *motor, float pos,float vel)
{
    if(motor->work_mode != DMMOTOR_MODE_POS_VEL)
    {
        DMMotorStop(motor);
        LOGERROR("DMMotorSetPosVel: Not implemented mode");
        return;
    }
    motor->motor_controller.target_position = pos;
    motor->motor_controller.target_velocity = vel;
}

void DMMotorSetVel(DMMotorInstance *motor, float vel)
{
    if(motor->work_mode != DMMOTOR_MODE_VEL)
    {
        DMMotorStop(motor);
        LOGERROR("DMMotorSetVel: Not implemented mode");
        return;
    }
    motor->motor_controller.target_velocity = vel;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void DMMotorPIDControl(DMMotorInstance *motor)
{
    float pid_measure,pid_ref,set;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DM_Motor_Measure_s *measure;           // 电机测量值

    motor_setting = &(motor->motor_settings);
    motor_controller = &motor->motor_controller;
    measure = &motor->measure;
    pid_ref = motor_controller->pid_ref;

    // pid_ref会顺次通过被启用的闭环充当数据的载体

    // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
    if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
    {
        if (motor_setting->angle_feedback_source == OTHER_FEED)
            pid_measure = *motor_controller->other_angle_feedback_ptr;
        else
        {
            pid_measure = measure->position;
            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                pid_measure *= -1;
        }
        // 更新pid_ref进入下一个环
        pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
    }

    // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
    if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            pid_ref += *motor_controller->speed_feedforward_ptr;

        if (motor_setting->speed_feedback_source == OTHER_FEED)
            pid_measure = *motor_controller->other_speed_feedback_ptr;
        else // MOTOR_FEED
        {
            pid_measure = measure->velocity;
            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                pid_measure *= -1;
        }

        // 更新pid_ref进入下一个环
        pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
    }

    set = pid_ref;
    if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        set *= -1;

    if(motor_controller->current_feedforward_ptr!=NULL)
    {
        set+=*motor_controller->current_feedforward_ptr;
    }

    LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
    motor->dm_send.MIT.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
    motor->dm_send.MIT.velocity_des =float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
    motor->dm_send.MIT.Kp = float_to_uint(0, 0, 500, 12);
    motor->dm_send.MIT.Kd = float_to_uint(0, 0, 5, 12);
    motor->dm_send.MIT.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);

    if(motor->stop_flag == MOTOR_STOP)
        motor->dm_send.MIT.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
}

void DMMotorPosControl(DMMotorInstance *motor,float pos,float velocity)
{
    LIMIT_MIN_MAX(pos, DM_P_MIN, DM_P_MAX);
    LIMIT_MIN_MAX(velocity, DM_V_MIN, DM_V_MAX);

    motor->dm_send.POS_VEL.position_des = pos;
    motor->dm_send.POS_VEL.velocity_des = velocity;

    if(motor->stop_flag == MOTOR_STOP)
        motor->dm_send.POS_VEL.velocity_des = 0;
}
void DMMotorVelControl(DMMotorInstance *motor, float velocity)
{
    LIMIT_MIN_MAX(velocity, DM_V_MIN, DM_V_MAX);

    motor->dm_send.VEL.velocity_des = velocity;

    if(motor->stop_flag == MOTOR_STOP)
        motor->dm_send.VEL.velocity_des = 0;

}
void DMMotorSend(DMMotorInstance* motor)
{
    switch(motor->work_mode)
    {
    case DMMOTOR_MODE_MIT:
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor->dm_send.MIT.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor->dm_send.MIT.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor->dm_send.MIT.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor->dm_send.MIT.velocity_des & 0xF) << 4) | (motor->dm_send.MIT.Kp >> 8) & 0xF);
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor->dm_send.MIT.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor->dm_send.MIT.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor->dm_send.MIT.Kd & 0xF) << 4) | (motor->dm_send.MIT.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor->dm_send.MIT.torque_des);

        // motor->motor_can_instace->tx_id = motor->motor_can_instace->tx_id ;
        // CANSetDLC(motor->motor_can_instace,8);
        break;
    case DMMOTOR_MODE_POS_VEL:
        memcpy(motor->motor_can_instace->tx_buff,&motor->dm_send.POS_VEL,8u);

        motor->motor_can_instace->tx_id = motor->motor_can_instace->tx_id+ 0x100;
        CANSetDLC(motor->motor_can_instace,8u);
        break;
    case DMMOTOR_MODE_VEL:
        memcpy(motor->motor_can_instace->tx_buff,&motor->dm_send.VEL,4u);
        memset(&motor->motor_can_instace->tx_buff[4],0,4u);

        motor->motor_can_instace->tx_id= motor->motor_can_instace->tx_id + 0x200;
        CANSetDLC(motor->motor_can_instace,4u);
        break;
    case DMMOTOR_MODE_FORCE_POS:
        memcpy(motor->motor_can_instace->tx_buff,&motor->dm_send.FORCE_POS.position_des,4u);
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor->dm_send.FORCE_POS.velocity_des >> 8);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor->dm_send.FORCE_POS.velocity_des);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(motor->dm_send.FORCE_POS.torque_des >> 8);
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor->dm_send.FORCE_POS.torque_des);

        motor->motor_can_instace->tx_id = motor->motor_can_instace->tx_id + 0x300;
        CANSetDLC(motor->motor_can_instace,8);
        break;
    default:
        break;
    }
    CANTransmit(motor->motor_can_instace, 1);
}

void DMMotorTask(void const *argument)
{
    
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    Motor_Control_Setting_s *motor_settings;
    DMMotor_Send_s *motor_send;
    Motor_Controller_s *motor_controller;
    DM_Motor_Measure_s *measure;
 
    for(;;)
    {
        motor_settings= &(motor->motor_settings);
        motor_controller= &motor->motor_controller;
        measure= &motor->measure;
        motor_send= &motor->dm_send;
        switch (motor->work_mode)
        {
       case DMMOTOR_MODE_MIT:
            DMMotorPIDControl(motor);
            break;
        case DMMOTOR_MODE_POS_VEL:
            DMMotorPosControl(motor,motor_controller->target_position, motor_controller->target_velocity);
            break;
        case DMMOTOR_MODE_VEL:
            DMMotorVelControl(motor,motor_controller->target_velocity);
            break;
        case DMMOTOR_MODE_FORCE_POS:

            break;
        default:
            break;
        }
        DMMotorSend(motor);
        osDelay(2);
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}