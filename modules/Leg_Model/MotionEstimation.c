#include "MotionEstimation.h"
#include "Observer.h"
#include "kalman_filter.h"

//状态向量：x=[速度v，加速度a]
//测量向量：z=[速度v，加速度a]

//     Z = H * X + V   观测方程
//     X = F * X + B * U + W  状态方程

//     R= E{V*V^T}                观测噪声协方差矩阵
//     P= E{(X-Xhat)*(X-Xhat)^T}  估计误差协方差矩阵
//     Q= E{W*W^T}                过程噪声协方差矩阵
//     F=                         状态转移矩阵
//     B=                         控制矩阵       
//     H=                         观测矩阵


//     卡尔曼滤波五个步骤：
//     1.状态预测：      Xhat_minus(k)=F*Xhat(k-1)+B*U(k-1)
//     2.协方差预测：     P_minus(k)=F*P(k-1)*F^T+Q
//     以上两步为预测步骤


//     以下三步为校正步骤
//     3.计算卡尔曼增益： K(k)=P_minus(k)*H^T*(H*P_minus(k)*H^T+R)^-1
//     4.状态更新：      Xhat(k)=Xhat_minus(k)+K(k)*(Z(k)-H*Xhat_minus(k))
//     5.协方差更新：    P(k)=(I-K(k)*H)*P_minus(k)

float MotionEstimation_H[4]={1.0f,      0,
                             0,         1.0f}; //观测矩阵H，用来将状态向量映射到测量空间，这里假设测量直接对应状态的每个分量，即位置和速度都可以直接测量
                             

float MotionEstimation_F[4]={1.0f,      0.001f,           //F12是dt，控制周期为1ms
                              0,        1.0f}; //状态转移矩阵F
                              //根据表达式 v = v0 + at，状态转移矩阵F的形式为：
                              //F = [1, dt; 0, 1]
float MotionEstimation_P[4]={1.0f,      0,
                             0,        1.0f}; //估计误差协方差矩阵P，这个会在滤波过程中不断更新
//超参数。主要调节参数，过程噪声协方差矩阵Q和观测噪声协方差矩阵R的选择会直接影响滤波器的性能。较小的Q值表示对模型的信任较高，较大的R值表示对测量的信任较低。根据实际情况调整这些参数以获得更好的估计结果。
float MotionEstimation_Q[4]={0.1f,      0,
                              0,        0.1f}; //过程噪声协方差矩阵Q，
                              
float MotionEstimation_R[4]={100.0f,    0,
                              0,        1000000.0f}; //观测噪声协方差矩阵R，分别是速度和加速度的测量噪声方差，速度测量较为可靠，因此设置较小的方差；加速度测量可能较为不稳定，因此设置较大的方差



//卡尔曼滤波初始化
void MotionEstimation_init(void)
{
    Balance_data* Balance_data=Get_Balance_Data();
    Kalman_Filter_Init(&Balance_data->body_data.MotionEstimation.KalmanFilter_Instance,2,0,2);
    memcpy(Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.P_data,MotionEstimation_P,sizeof(MotionEstimation_P));
	memcpy(Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.F_data,MotionEstimation_F,sizeof(MotionEstimation_F));
	memcpy(Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.H_data,MotionEstimation_H,sizeof(MotionEstimation_H));
	memcpy(Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.Q_data,MotionEstimation_Q,sizeof(MotionEstimation_Q));
	memcpy(Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.R_data,MotionEstimation_R,sizeof(MotionEstimation_R));
}


/*
 *函数简介:运动估计量测更新
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void MotionEstimation_MeasureUpdate(void)
{
  Balance_data* Balance_data=Get_Balance_Data();
  //w=web+phi_bc_dot+wecd;
  //驱动轮转子相对大地的角速度=机体相对大地的角速度+轮向电机定子相对机体的角速度+轮向电机转子相对定子的角速度
  //v=r*w+L0_dot*sin(theta)+L0*theta_dot*cos(theta)
   float w_l=Balance_data->Leg_L.wheel.speed-Balance_data->body_data.d_pitch-Balance_data->Leg_L.dphi_0; //左轮相对大地的角速度
   Balance_data->body_data.MotionEstimation.v_measured_L=Wheel_R*w_l+Balance_data->Leg_L.d_L0*arm_sin_f32(Balance_data->Leg_L.theta)+Balance_data->Leg_L.L0*Balance_data->Leg_L.dtheta*arm_cos_f32(Balance_data->Leg_L.theta); //左轮测量速度
   
    float w_r=Balance_data->Leg_R.wheel.speed-Balance_data->body_data.d_pitch-Balance_data->Leg_R.dphi_0; //右轮相对大地的角速度
    Balance_data->body_data.MotionEstimation.v_measured_R=Wheel_R*w_r+Balance_data->Leg_R.d_L0*arm_sin_f32(Balance_data->Leg_R.theta)+Balance_data->Leg_R.L0*Balance_data->Leg_R.dtheta*arm_cos_f32(Balance_data->Leg_R.theta); //右轮测量速度
    
    Balance_data->body_data.MotionEstimation.v_measured=0.5f*(Balance_data->body_data.MotionEstimation.v_measured_L+Balance_data->body_data.MotionEstimation.v_measured_R); //整体测量速度
    Balance_data->body_data.MotionEstimation.a_measured=Balance_data->body_data.a_yE; //测量加速度

}

/*
 *函数简介:运动估计更新
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void MotionEstimation_Update(void)
{
    Balance_data* Balance_data=Get_Balance_Data();
    MotionEstimation_MeasureUpdate();
    //未滤波位置和速度保存
    Balance_data->body_data.MotionEstimation.v_nofilter=0.5f*(Balance_data->Leg_L.wheel.speed*Wheel_R + Balance_data->Leg_R.wheel.speed*Wheel_R);
    Balance_data->body_data.MotionEstimation.x_nofilter+=Balance_data->body_data.MotionEstimation.v_nofilter*Balance_data->dt;

    Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.MeasuredVector[0]=Balance_data->body_data.MotionEstimation.v_measured;
    Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.MeasuredVector[1]=Balance_data->body_data.MotionEstimation.a_measured;

    //更新卡尔曼滤波器，五个公式
    Kalman_Filter_Update(&Balance_data->body_data.MotionEstimation.KalmanFilter_Instance);

    Balance_data->body_data.MotionEstimation.v=Balance_data->body_data.MotionEstimation.KalmanFilter_Instance.FilteredValue[0];
    Balance_data->body_data.MotionEstimation.x+=Balance_data->body_data.MotionEstimation.v*Balance_data->dt;

}