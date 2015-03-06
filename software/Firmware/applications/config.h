#ifndef __CONFIG_H
#define __CONFIG_H

#include "board.h"
#include "pid.h"
#include "filter.h"
#include "imu.h"
#include "rc.h"
#include "pidctrl.h"

/*----------------------IMU--------------------*/
#define ANO_IMU_USE_DCM_CF
//#define ANO_IMU_USE_Quaternions_CF

//#define ANO_IMU_USE_LPF_1st
#define ANO_IMU_USE_LPF_2nd

#define IMU_LOOP_TIME					2000	//��λΪuS
#define PID_INNER_LOOP_TIME		2000	//��λΪus
#define PID_OUTER_LOOP_TIME		5000	//��λΪus

#define ACC_1G 			4096		//�ɼ��ٶȼƵ�����ȷ��
#define ACC_LPF_CUT 30.0f		//���ٶȵ�ͨ�˲�����ֹƵ��30Hz

#define GYRO_CF_TAU 1.2f
/*---------------------------------------------*/

/*-------------------�������ݷ��ͷ�ʽѡ��-----------------*/
#define ANO_DT_USE_UART
//#define ANO_DT_USE_WIFI
/*--------------------------------------------------------*/


class Config
{
	
public:
	
	Config();

	class Factor{
		public:		
			float acc_lpf;		
			float gyro_cf;		
	}factor;

	class Flag{
		public:
			uint8_t ARMED;
			uint8_t failsafe;
	}f;
	
	//ָʾ��
	void Pilot_Light(void);
	
};

extern Config config;

#endif
