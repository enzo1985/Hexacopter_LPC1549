/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������ƴ�
 * �ļ���  ��ANO_Filter.cpp
 * ����    ���˲�����غ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "filter.h"


/*----------------------һ�׵�ͨ�˲���ϵ������-------------------------*/
float LowPassFilter_1st_Factor_Cal(float deltaT, float Fcut)
{
	return deltaT / (deltaT + 1 / (2 * M_PI * Fcut));
}

/*----------------------һ�׵�ͨ�˲���------------------------*/
Vector3f LowPassFilter_1st(Vector3f oldData, Vector3f newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}

/*----------------------���׵�ͨ�˲���ϵ������-------------------------*/
void LowPassFilter_2nd_Factor_Cal(LPF2ndData_t* lpf_data)
{
	//��ֹƵ��:30Hz ����Ƶ��:500Hz
	lpf_data->b0 = 0.1883633f;
	lpf_data->a1 = 1.023694f;
	lpf_data->a2 = 0.2120577f;
}

/*----------------------���׵�ͨ�˲���------------------------*/
Vector3f LowPassFilter_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData)
{
	Vector3f lpf_2nd_data;
	
	lpf_2nd_data = newData * lpf_2nd->b0 + lpf_2nd->lastout * lpf_2nd->a1 - lpf_2nd->preout * lpf_2nd->a2;
	lpf_2nd->preout = lpf_2nd->lastout;
	lpf_2nd->lastout = lpf_2nd_data;
	
	return lpf_2nd_data;
}

/*----------------------�����˲���ϵ������-------------------------*/
float ComplementaryFilter_Factor_Cal(float deltaT, float tau)
{
	return tau / (deltaT + tau);
}

/*----------------------һ�׻����˲���-----------------------------*/
Vector3f ComplementaryFilter_1st(Vector3f gyroData, Vector3f accData, float cf_factor)
{ 
	return (gyroData * cf_factor + accData *(1 - cf_factor));	
}





/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
