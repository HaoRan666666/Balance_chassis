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

// 计算右腿 theta2_dot
float calculate_theta2_dot_right() 
{
    DMMotorInstance * dm_motor_instance=Get_DM_Instance(); // 获取电机实例指针数组
    // 获取当前值
    float theta_1 = Balance_status.Leg_R.theta_1;
    float small_rod_angle = Balance_status.Leg_R.small_rod_angle;
    float dtheta_1 = Balance_status.Leg_R.dtheta_1;
    // 获取右后电机的角速度（小连杆角速度）
    float d_small_rod_angle = dm_motor_instance[3].measure.velocity;
    // 计算 B 点和 D 点的坐标
    float xd = L1 * arm_cos_f32(theta_1);
    float yd = L1 * arm_sin_f32(theta_1);
    float xb = L1 * arm_cos_f32(PI - small_rod_angle);
    float yb = L1 * arm_sin_f32(PI - small_rod_angle);
    // 计算 B 点和 D 点的速度
    float dxd_dt = -L1 * arm_sin_f32(theta_1) * dtheta_1;
    float dyd_dt = L1 * arm_cos_f32(theta_1) * dtheta_1;
    float dxb_dt = L1 * arm_sin_f32(PI - small_rod_angle) * d_small_rod_angle;
    float dyb_dt = -L1 * arm_cos_f32(PI - small_rod_angle) * d_small_rod_angle;
    // 计算中间变量 phi2
    float A0 = 2 * L2 * (xd - xb);
    float B0 = 2 * L2 * (yd - yb);
    float L_bd2 = (xd - xb) * (xd - xb) + (yd - yb) * (yd - yb);
    float C0 = L_bd2;
    float phi2 = 2 * atanf((B0 + sqrtf(A0*A0 + B0*B0 - C0*C0)) / (A0 + C0));
    // 计算 theta2
    float theta2 = atan2f(yb - yd + L2 * arm_sin_f32(phi2), xb - xd + L2 * arm_cos_f32(phi2));
    // 计算 theta2_dot 使用解析方法
    float denominator = L2 * arm_sin_f32(phi2 - theta2);
    // 避免除以零
    if (fabsf(denominator) < 1e-6f) 
    {
        return 0.0f;
    }
    float numerator = arm_cos_f32(phi2) * (dxd_dt - dxb_dt) + arm_sin_f32(phi2) * (dyd_dt - dyb_dt);
    float theta2_dot = numerator / denominator;
    return theta2_dot;
}


void Observer_init(void)
{
   Balance_status.dt=0.002;  //2ms观测一次 
}

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
   //车体数据
   Balance_status.body_data.a_xb = INS.MotionAccel_b[1];
   Balance_status.body_data.a_yb = INS.MotionAccel_b[0];
   Balance_status.body_data.a_zb = INS.MotionAccel_b[2];

   Balance_status.body_data.a_xE = INS.MotionAccel_n[1];
   Balance_status.body_data.a_yE = INS.MotionAccel_n[0];
   Balance_status.body_data.a_zE = INS.MotionAccel_n[2];

   Balance_status.body_data.Pitch = INS.Pitch;
   Balance_status.body_data.Roll  = INS.Roll;
   Balance_status.body_data.Yaw   = INS.Yaw;

   Balance_status.body_data.d_pitch = INS.Gyro[0];
   Balance_status.body_data.d_Roll  = INS.Gyro[1];
   Balance_status.body_data.d_Yaw   = INS.Gyro[2];

   //车轮数据
   //Balance_status.Leg_L.L0=
   float left_reversal_factor = -1.0f;  // 左侧反转
  
   Balance_status.Leg_L.wheel.angle=dji_motor_instance[0].measure.total_angle * left_reversal_factor * DEG_TO_RAD ;  //总角度（rad）
   Balance_status.Leg_L.wheel.speed=dji_motor_instance[0].measure.speed_aps* left_reversal_factor*DEG_TO_RAD;    
   Balance_status.Leg_L.wheel.x=Balance_status.Leg_L.wheel.angle * Wheel_R ;  //单位是米
   Balance_status.Leg_L.wheel.T=dji_motor_instance[0].measure.real_current* left_reversal_factor* 20 * 0.3 / 16384 ;//初始数据到16384,转化后乘以转矩常数

   Balance_status.Leg_R.wheel.angle=dji_motor_instance[1].measure.total_angle*DEG_TO_RAD;  //总角度 //注意单位换算
   Balance_status.Leg_R.wheel.speed=dji_motor_instance[1].measure.speed_aps*DEG_TO_RAD;  
   Balance_status.Leg_R.wheel.x=Balance_status.Leg_R.wheel.angle * Wheel_R ;  //单位是米
   Balance_status.Leg_R.wheel.T=dji_motor_instance[1].measure.real_current * 20 * 0.3/ 16384 ;//初始数据到16384,转化后乘以转矩常数

   /************************************腿部数据******************************/

   Balance_status.Leg_L.small_rod_angle= dm_motor_instance[2].measure.position;//左后电机单圈角度
   Balance_status.Leg_L.d_small_rod_angle= dm_motor_instance[2].measure.velocity;//左后电机单圈角度
   Balance_status.Leg_L.theta_1= dm_motor_instance[0].measure.position ;//左前电机单圈角度
   Balance_status.Leg_L.dtheta_1= dm_motor_instance[0].measure.velocity;
// //theta2求解测试代码（直接用五连杆的方法）
   float xd=L1*arm_cos_f32(Balance_status.Leg_L.theta_1);
   float yd=L1*arm_sin_f32(Balance_status.Leg_L.theta_1);
   float xb=L1*arm_cos_f32(PI -Balance_status.Leg_L.small_rod_angle);
   float yb=L1*arm_sin_f32(PI -Balance_status.Leg_L.small_rod_angle);

   float A0=2*L2*(xd-xb);
   float B0=2*L2*(yd-yb);
   float L_bd2=(xd-xb)*(xd-xb)+(yd-yb)*(yd-yb);
   float C0=L_bd2;
   
   float phi2=2*atanf((B0+sqrtf(A0*A0+B0*B0-C0*C0))/(A0+C0));

   Balance_status.Leg_L.theta_2=atan2f(yb - yd + L2 * arm_sin_f32(phi2), xb - xd + L2 * arm_cos_f32(phi2));
   
   Balance_status.Leg_L.dtheta_2=calculate_theta2_dot_right();

   //d_theta2
   //计算 B 点和 D 点的速度  (五连杆方式)
   Balance_status.Leg_L.last_dphi_0=Balance_status.Leg_L.dphi_0;//在更新d_phi0之前保存本次d_phi0
   Observer_LegForwardKinematicsSolution(&Balance_status.Leg_L);
   Balance_status.Leg_L.ddphi_0=0.19f*(Balance_status.Leg_L.dphi_0-Balance_status.Leg_L.last_dphi_0)/Balance_status.dt+0.81f*Balance_status.Leg_L.ddphi_0;//一阶低通滤波
  
   Balance_status.Leg_L.Tp1= dm_motor_instance[0].measure.torque; 
   Balance_status.Leg_L.Tp2= dm_motor_instance[2].measure.torque;

   Balance_status.Leg_L.theta=PI/2 - Balance_status.Leg_L.phi_0; //摆杆摆角theta(rad)
   Balance_status.Leg_L.last_dtheta=Balance_status.Leg_L.dtheta;	//上一次摆杆摆角角速度last_theta(rad/s)
   Balance_status.Leg_L.dtheta=-Balance_status.Leg_L.dphi_0;			//摆杆摆角角速度dtheta(rad/s)
   Balance_status.Leg_L.ddtheta=0.19f*(Balance_status.Leg_L.dtheta-Balance_status.Leg_L.last_dtheta)/Balance_status.dt+0.81f*Balance_status.Leg_L.ddtheta;//一阶低通滤波

   Observer_GetFN(&Balance_status.Leg_L);
                                     //右腿
   Balance_status.Leg_R.small_rod_angle=dm_motor_instance[3].measure.position;//右后电机单圈角度
   Balance_status.Leg_R.d_small_rod_angle= dm_motor_instance[3].measure.velocity;
   Balance_status.Leg_R.theta_1= dm_motor_instance[1].measure.position;//左前电机单圈角度
   Balance_status.Leg_R.dtheta_1= dm_motor_instance[1].measure.velocity;

  //theta2求解测试代码（直接用五连杆的方法）
    xd=L1*arm_cos_f32(Balance_status.Leg_R.theta_1);
    yd=L1*arm_sin_f32(Balance_status.Leg_R.theta_1);
    xb=L1*arm_cos_f32(PI -Balance_status.Leg_R.small_rod_angle);
    yb=L1*arm_sin_f32(PI -Balance_status.Leg_R.small_rod_angle);

    A0=2*L2*(xd-xb);
    B0=2*L2*(yd-yb);
    L_bd2=(xd-xb)*(xd-xb)+(yd-yb)*(yd-yb);
    C0=L_bd2;
   
    phi2=2*atanf((B0+sqrtf(A0*A0+B0*B0-C0*C0))/(A0+C0));
    //atan函数无法区分第三象限和第一象限  第二象限和第四象限 
    //atan2函数可以通过x，y的正负判断

   Balance_status.Leg_R.theta_2=atan2f(yb - yd + L2 * arm_sin_f32(phi2), xb - xd + L2 * arm_cos_f32(phi2));
  
   Balance_status.Leg_R.dtheta_2=calculate_theta2_dot_right();

   Balance_status.Leg_R.last_dphi_0=Balance_status.Leg_R.dphi_0;//在更新d_phi0之前保存本次d_phi0
   Observer_LegForwardKinematicsSolution(&Balance_status.Leg_R);
   Balance_status.Leg_R.ddphi_0=0.19f*(Balance_status.Leg_R.dphi_0-Balance_status.Leg_R.last_dphi_0)/Balance_status.dt+0.81f*Balance_status.Leg_R.ddphi_0;//一阶低通滤波
   
   Balance_status.Leg_R.Tp1=dm_motor_instance[1].measure.torque ;
   Balance_status.Leg_R.Tp2=dm_motor_instance[3].measure.torque ;

   Balance_status.Leg_R.theta=PI/2 - Balance_status.Leg_R.phi_0; //摆杆摆角theta(rad)
   Balance_status.Leg_R.last_dtheta=Balance_status.Leg_R.dtheta;	//上一次摆杆摆角角速度last_theta(rad/s)
   Balance_status.Leg_R.dtheta=-Balance_status.Leg_R.dphi_0;			//摆杆摆角角速度dtheta(rad/s)
   Balance_status.Leg_R.ddtheta=0.19f*(Balance_status.Leg_R.dtheta-Balance_status.Leg_R.last_dtheta)/Balance_status.dt+0.81f*Balance_status.Leg_R.ddtheta;//一阶低通滤波

   Observer_GetFN(&Balance_status.Leg_R);

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
   float theta_1=Leg->theta_1;
   float theta_2=Leg->theta_2;
   float d_theta_1=-Leg->dtheta_1;//这里加了一个负号，注意角速度正方向问题
   float d_theta_2=Leg->dtheta_2;

   float xc= L1*arm_cos_f32(theta_1)+L2*arm_cos_f32(theta_2);

   float yc= L1*arm_sin_f32(theta_1)+L2*arm_sin_f32(theta_2);
   Leg->L0=sqrtf(xc*xc+yc*yc);
   Leg->phi_0=atan2f(yc,xc);

  //获取VMC雅可比矩阵元素
   Leg->J_11=-(L1*L2*arm_sin_f32(theta_1-theta_2))/ (sqrtf(L1*L1+2*cos(theta_1-theta_2)*L1*L2+L2*L2));
   Leg->J_12=(L1*L2*arm_sin_f32(theta_1-theta_2))/ (sqrtf(L1*L1+2*cos(theta_1-theta_2)*L1*L2+L2*L2));
   Leg->J_21=(L1*(L1+L2*arm_cos_f32(theta_1-theta_2)))/(L1*L1+2*arm_cos_f32(theta_1-theta_2)*L1*L2+L2*L2);
   Leg->J_22=(L2*(L2+L1*arm_cos_f32(theta_1-theta_2)))/(L1*L1+2*arm_cos_f32(theta_1-theta_2)*L1*L2+L2*L2);

   //获取VMC逆解矩阵元素
   // 计算 DeltaTheta = theta1 - theta2
   float delta_theta = theta_1 - theta_2;
   float cos_delta = arm_cos_f32(delta_theta);
   float sin_delta = arm_sin_f32(delta_theta);

   // 计算 A 和 D
   float A = L1 * L1 + 2 * L1 * L2 * cos_delta + L2 * L2;
   float D = sqrtf(A);

   // 检查 sin_delta 是否接近零，避免除以零
   float J_inv_11 ;
   float J_inv_12 ;
   float J_inv_21 ;
   float J_inv_22 ;

   if (fabsf(sin_delta) > 1e-6f) 
   {

              // 计算逆矩阵元素
    J_inv_11 = -D * (L2 + L1 * cos_delta) / (A * L1 * sin_delta);
    J_inv_12 = 1.0f;
    J_inv_21 = D * (L1 + L2 * cos_delta) / (A * L2 * sin_delta);
    J_inv_22 = 1.0f;
    
   }
   else 
   {
        // 处理奇异位置
      J_inv_11 = 1.0f; 
      J_inv_12 = 0.0f; 
      J_inv_21 = 0.0f;
      J_inv_22 = 1.0f;
      return;
   }

   Leg->T_11=J_inv_11 ;
   Leg->T_12=J_inv_12 ;
   Leg->T_21=J_inv_21 ;
   Leg->T_22=J_inv_22 ;

   //获取dL0 ddL0 dphi0
   (Leg->last_dL0)=(Leg->d_L0);
   float xc_dot = -L1 * arm_sin_f32(theta_1) * d_theta_1 - L2 * arm_sin_f32(theta_2) * d_theta_2;
   float yc_dot = L1 * arm_cos_f32(theta_1) * d_theta_1 + L2 * arm_cos_f32(theta_2) * d_theta_2;
   if(Leg->L0>1e-6)
   {
       Leg->d_L0 = (xc * xc_dot + yc * yc_dot) / Leg->L0; // 确保L0不为零
       Leg->dphi_0 = (xc * yc_dot - yc * xc_dot) / (Leg->L0 * Leg->L0);
   }
   (Leg->dd_L0)=0.19f*(Leg->d_L0-Leg->last_dL0)/Balance_status.dt+0.81f*(Leg->dd_L0);//一阶低通滤波



   //获取L0和phi0
	// float phi_1=PI-Leg->small_rod_angle;
	// float phi_4=Leg->theta_1;
	// float dphi_1=-mt_motor_instance[3]->measure.velocity;//这里加了一个负号，注意角速度正方向问题
	// float dphi_4=Leg->dtheta_1;
	
	// float XB=L1*arm_cos_f32(phi_1);
	// float YB =L1*arm_sin_f32(phi_1);
	// float XD=L1*arm_cos_f32(phi_4);
	// float YD=L1*arm_sin_f32(phi_4);
	
	// float lBD_2=(XD-XB)*(XD-XB)+(YD-YB)*(YD-YB);
	
	// float A0=2*L2*(XD-XB);
	// float B0=2*L2*(YD-YB);
	// float C0=L2*L2+lBD_2-L2*L2;
	// float phi2=2*atan2f((B0+sqrt(A0*A0+B0*B0-C0*C0)),A0+C0);
	// float phi3=atan2f(YB-YD+L2*arm_sin_f32(phi2),XB-XD+L2*arm_cos_f32(phi2));
	
	// float XC=L1*arm_cos_f32(phi_1)+L2*arm_cos_f32(phi2);
	// float YC=L1*arm_sin_f32(phi_1)+L2*arm_sin_f32(phi2);
	
	// (Leg->L0)=sqrt(XC*XC+YC*YC);
	// (Leg->phi_0)=atan2f(YC,XC);   //phi0通过五连杆中的c坐标计算
	
	// //获取VMC雅可比矩阵元素
	// float sigma1=arm_sin_f32(phi3-phi2);
	// float sigma2=arm_sin_f32(phi3-phi_4);
	// float sigma3=arm_sin_f32(phi_1-phi2);
	// float sigma4=arm_sin_f32(Leg->phi_0-phi3);
	// float sigma5=arm_cos_f32(Leg->phi_0-phi3);
	// float sigma6=arm_sin_f32(Leg->phi_0-phi2);
	// float sigma7=arm_cos_f32(Leg->phi_0-phi2);
	
	// (Leg->J_11)=(L1*sigma4*sigma3)/sigma1;
	// (Leg->J_12)=(L1*sigma6*sigma2)/sigma1;
	// (Leg->J_21)=(L1*sigma5*sigma3)/((Leg->L0)*sigma1);
	// (Leg->J_22)=(L1*sigma7*sigma2)/((Leg->L0)*sigma1);
	
	// //获取VMC逆解矩阵元素
	// float sigma8=L1*sigma2;
	// float sigma9=L1*sigma3;
	// (Leg->T_11)=-sigma7/sigma9;
	// (Leg->T_12)=sigma5/sigma8;
	// (Leg->T_21)=(Leg->L0)*sigma6/sigma9;
	// (Leg->T_22)=-(Leg->L0)*sigma4/sigma8;
	
	// //获取dL0 ddL0 dphi0
	// float sigma10=L1*dphi_1;
	// float sigma11=-XC;
	// float sigma12=sigma10*arm_sin_f32(phi_1-phi3)+sigma8*dphi_4;
	// float sigma13=sigma12/sigma1;
	// float sigma14=sigma10*arm_cos_f32(phi_1)+sigma13*arm_cos_f32(phi2);
	// float sigma15=sigma10*arm_sin_f32(phi_1)+sigma13*arm_sin_f32(phi2);
	// (Leg->last_dL0)=(Leg->d_L0);
	// (Leg->d_L0)=(YC*sigma14+sigma11*sigma15)/(Leg->L0);
	// (Leg->dd_L0)=0.19f*(Leg->d_L0-Leg->last_dL0)/Balance_status.dt+0.81f*(Leg->dd_L0);//一阶低通滤波
	// (Leg->dphi_0)=-(sigma14*sigma11-YC*sigma15)/(YC*YC+sigma11*sigma11);


}

void Observer_GetFN(Leg_data *Leg)
{
   float COS=arm_cos_f32(PI-Leg->phi_0);//转化为与竖直方向的夹角
	float SIN=arm_sin_f32(PI-Leg->phi_0);

   (Leg->F)=(Leg->T_11) * (Leg->Tp1)+(Leg->T_12) * (Leg->Tp2);
	(Leg->Tp)=(Leg->T_21) * (Leg->Tp1)+(Leg->T_22) * (Leg->Tp2);
	float P=(Leg->F)*COS+(Leg->Tp)*SIN/(Leg->L0);
	
	float ddz_w=Balance_status.body_data.a_zE-(Leg->dd_L0)*COS+2.0f*(Leg->d_L0)*(Leg->dphi_0)*SIN+(Leg->L0)*(Leg->ddphi_0)*SIN+(Leg->L0)*(Leg->dphi_0)*(Leg->dphi_0)*COS;

	(Leg->Fn)= P+ m_w * gravity + m_w * ddz_w; 
}