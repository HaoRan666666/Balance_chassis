#include "stm32h7xx.h"            
#include "Observer.h"



#define FN_Threshold 5.0f//支持力阈值 

float K2_l[12]={0},K2_r[12]={0};

	float K2_p11[4]={20.4392,94.2238,-155.5329,1.7024};//根据经验公式展开之后的系数
	float K2_p12[4]={55.9817,-50.4603,-8.3689,0.2381};
	float K2_p13[4]={-181.8475,219.3275,-96.9871,3.4664};
	float K2_p14[4]={-103.5657,138.6344,-72.4948,1.9338};
	float K2_p15[4]={275.7576,-157.1632,-21.7816,31.6595};
	float K2_p16[4]={27.8745,-17.9068,-1.3970,4.1551};
	float K2_p21[4]={1497.2384,-1401.4599,420.1510,13.5622};
	float K2_p22[4]={111.2058,-131.8536,59.7382,0.6340};
	float K2_p23[4]={431.5558,-236.0704,-41.0164,48.6301};
	float K2_p24[4]={316.9914,-189.6058,-18.4850,36.9037};
	float K2_p25[4]={1343.4311,-1608.5975,710.8411,-33.7040};
	float K2_p26[4]={153.3062,-189.6469,88.1234,-8.8669};

/*
 *函数简介:根据目标位移和速度计算 由x和dx控制所需的速度控制量
 *参数说明:目标位移x
 *参数说明:目标速度dx
 *返回类型:速度控制量
 *备注:无
 */
float Get_Uspeed(float target_x,float target_dx)
{
	Balance_data*  Balance_status  = Get_Balance_Data();
	return K2_r[8]*(Balance_status->body_data.x-target_x)+K2_r[9]*(Balance_status->body_data.dx-target_dx);//TODO验证一下左右是不是一样的
}

/*
 *函数简介:根据目标位移和速度计算 由theta和phi控制所需的轮子控制量
 *参数说明:目标位移x
 *参数说明:目标速度dx
 *返回类型:轮子控制量
 *备注:无
 */
float Get_Uelse_R(void)
{
	Balance_data*  Balance_status  = Get_Balance_Data();
	return K2_r[6]*Balance_status->Leg_R.theta+K2_r[7]*Balance_status->Leg_R.dtheta+K2_r[10]*Balance_status->body_data.Pitch+K2_r[11]*Balance_status->body_data.d_pitch;
}

float Get_Uelse_L(void)
{
	Balance_data*  Balance_status  = Get_Balance_Data();	
	return K2_l[6]*Balance_status->Leg_L.theta+K2_l[7]*Balance_status->Leg_L.dtheta+K2_l[10]*Balance_status->body_data.Pitch+K2_l[11]*Balance_status->body_data.d_pitch;
}


    /*
 *函数简介:LQR计算
 *参数说明:左轮T
 *参数说明:左腿Tp
 *参数说明:右轮T
 *参数说明:右腿Tp
 *参数说明:目标位移x
 *参数说明:目标速度dx
 *返回类型:无
 *备注:无
 */


void LQR_Clc(float *Tl,float *Tpl,float *Tr,float *Tpr,float target_x,float target_dx)
{
	{
		    Balance_data*  Balance_status  = Get_Balance_Data();
			float L0_l=Balance_status->Leg_L.L0;
			float L0_r=Balance_status->Leg_R.L0;
			
			K2_l[0]=K2_p11[0]*L0_l*L0_l*L0_l+K2_p11[1]*L0_l*L0_l+K2_p11[2]*L0_l+K2_p11[3];
			K2_l[1]=K2_p12[0]*L0_l*L0_l*L0_l+K2_p12[1]*L0_l*L0_l+K2_p12[2]*L0_l+K2_p12[3];
			K2_l[2]=K2_p13[0]*L0_l*L0_l*L0_l+K2_p13[1]*L0_l*L0_l+K2_p13[2]*L0_l+K2_p13[3];
			K2_l[3]=K2_p14[0]*L0_l*L0_l*L0_l+K2_p14[1]*L0_l*L0_l+K2_p14[2]*L0_l+K2_p14[3];
			K2_l[4]=K2_p15[0]*L0_l*L0_l*L0_l+K2_p15[1]*L0_l*L0_l+K2_p15[2]*L0_l+K2_p15[3];
			K2_l[5]=K2_p16[0]*L0_l*L0_l*L0_l+K2_p16[1]*L0_l*L0_l+K2_p16[2]*L0_l+K2_p16[3];

			K2_l[6]=K2_p21[0]*L0_l*L0_l*L0_l+K2_p21[1]*L0_l*L0_l+K2_p21[2]*L0_l+K2_p21[3];
			K2_l[7]=K2_p22[0]*L0_l*L0_l*L0_l+K2_p22[1]*L0_l*L0_l+K2_p22[2]*L0_l+K2_p22[3];
			K2_l[8]=K2_p23[0]*L0_l*L0_l*L0_l+K2_p23[1]*L0_l*L0_l+K2_p23[2]*L0_l+K2_p23[3];
			K2_l[9]=K2_p24[0]*L0_l*L0_l*L0_l+K2_p24[1]*L0_l*L0_l+K2_p24[2]*L0_l+K2_p24[3];
			K2_l[10]=K2_p25[0]*L0_l*L0_l*L0_l+K2_p25[1]*L0_l*L0_l+K2_p25[2]*L0_l+K2_p25[3];
			K2_l[11]=K2_p26[0]*L0_l*L0_l*L0_l+K2_p26[1]*L0_l*L0_l+K2_p26[2]*L0_l+K2_p26[3];//根据腿长调节K矩阵    经验公式K（L0）=P0+P1*L0+P2*L0*L0+P2*L0*L0*L0
			
			K2_r[0]=K2_p11[0]*L0_r*L0_r*L0_r+K2_p11[1]*L0_r*L0_r+K2_p11[2]*L0_r+K2_p11[3];
			K2_r[1]=K2_p12[0]*L0_r*L0_r*L0_r+K2_p12[1]*L0_r*L0_r+K2_p12[2]*L0_r+K2_p12[3];
			K2_r[2]=K2_p13[0]*L0_r*L0_r*L0_r+K2_p13[1]*L0_r*L0_r+K2_p13[2]*L0_r+K2_p13[3];
			K2_r[3]=K2_p14[0]*L0_r*L0_r*L0_r+K2_p14[1]*L0_r*L0_r+K2_p14[2]*L0_r+K2_p14[3];
			K2_r[4]=K2_p15[0]*L0_r*L0_r*L0_r+K2_p15[1]*L0_r*L0_r+K2_p15[2]*L0_r+K2_p15[3];
			K2_r[5]=K2_p16[0]*L0_r*L0_r*L0_r+K2_p16[1]*L0_r*L0_r+K2_p16[2]*L0_r+K2_p16[3];

			K2_r[6]=K2_p21[0]*L0_r*L0_r*L0_r+K2_p21[1]*L0_r*L0_r+K2_p21[2]*L0_r+K2_p21[3];
			K2_r[7]=K2_p22[0]*L0_r*L0_r*L0_r+K2_p22[1]*L0_r*L0_r+K2_p22[2]*L0_r+K2_p22[3];
			K2_r[8]=K2_p23[0]*L0_r*L0_r*L0_r+K2_p23[1]*L0_r*L0_r+K2_p23[2]*L0_r+K2_p23[3];
			K2_r[9]=K2_p24[0]*L0_r*L0_r*L0_r+K2_p24[1]*L0_r*L0_r+K2_p24[2]*L0_r+K2_p24[3];
			K2_r[10]=K2_p25[0]*L0_r*L0_r*L0_r+K2_p25[1]*L0_r*L0_r+K2_p25[2]*L0_r+K2_p25[3];
			K2_r[11]=K2_p26[0]*L0_r*L0_r*L0_r+K2_p26[1]*L0_r*L0_r+K2_p26[2]*L0_r+K2_p26[3];
			
			
			//先当板凳调
			if(Balance_status->Leg_L.Fn<FN_Threshold)
			{
				// (*Tl)=0;
				(*Tpl)=K2_l[6]*Balance_status->Leg_L.theta+K2_l[7]*Balance_status->Leg_L.dtheta;
			}
			else
			{
				// (*Tl)=K2_l[0]*Balance_status->Leg_L.theta+K2_l[1]*Balance_status->Leg_L.dtheta+K2_l[2]*(Balance_status->body_data.x-target_x)+K2_l[3]*(Balance_status->body_data.dx-target_dx)+K2_l[4]*Balance_status->body_data.Pitch+K2_l[5]*Balance_status->body_data.d_pitch;
				(*Tpl)=K2_l[6]*Balance_status->Leg_L.theta+K2_l[7]*Balance_status->Leg_L.dtheta+K2_l[8]*(Balance_status->body_data.x-target_x)+K2_l[9]*(Balance_status->body_data.dx-target_dx)+K2_l[10]*Balance_status->body_data.Pitch+K2_l[11]*Balance_status->body_data.d_pitch;
			}
			
			if(Balance_status->Leg_R.Fn<FN_Threshold)
			{
				// (*Tr)=0;
				(*Tpr)=K2_r[6]*Balance_status->Leg_R.theta+K2_r[7]*Balance_status->Leg_R.dtheta;
			}
			else
			{

				// (*Tr)=K2_r[0]*Balance_status->Leg_R.theta+K2_r[1]*Balance_status->Leg_R.dtheta+K2_r[2]*(Balance_status->body_data.x-target_x)+K2_r[3]*(Balance_status->body_data.dx-target_dx)+K2_r[4]*Balance_status->body_data.Pitch+K2_r[5]*Balance_status->body_data.d_pitch;
				(*Tpr)=K2_r[6]*Balance_status->Leg_R.theta+K2_r[7]*Balance_status->Leg_R.dtheta+K2_r[8]*(Balance_status->body_data.x-target_x)+K2_r[9]*(Balance_status->body_data.dx-target_dx)+K2_r[10]*Balance_status->body_data.Pitch+K2_r[11]*Balance_status->body_data.d_pitch;
			}
			
	}
}



