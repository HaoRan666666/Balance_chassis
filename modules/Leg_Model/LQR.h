#ifndef __LQR_H
#define __LQR_H

void LQR_Clc(float *Tl,float *Tpl,float *Tr,float *Tpr,float target_x,float target_dx);
float Get_Uspeed(float target_x,float target_dx);
float Get_Uelse_R(void);
float Get_Uelse_L(void);
#endif
