/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_DT.cpp
 * 描述    ：无线数据收发和处理
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include <rtthread.h>
#include "commander.h"
#include "config.h"
#include "params.h"
#include "sensor.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

Commander cmd;

void Commander::Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	
	config.f.failsafe = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			sensor.Acc_CALIBRATED = 1;
		if(*(data_buf+4)==0X02)
			sensor.Gyro_CALIBRATED = 1;
		if(*(data_buf+4)==0X03)
		{
			sensor.Acc_CALIBRATED = 1;		
			sensor.Gyro_CALIBRATED = 1;			
		}
		if(*(data_buf+4)==0XA0)
		{
			config.f.ARMED = 0;	//上锁
		}
		if(*(data_buf+4)==0XA1)
		{
			config.f.ARMED = 1;	//解锁
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.Send_PID1 = 1;
			f.Send_PID2 = 1;
			f.Send_PID3 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
	}

	if(*(data_buf+2)==0X03)
	{
		rc.rawData[THROTTLE] = (uint16_t)(*(data_buf+4)<<8)|*(data_buf+5);
		rc.rawData[YAW] = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		rc.rawData[ROLL] = (uint16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		rc.rawData[PITCH] = (uint16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		rc.rawData[AUX1] = (uint16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		rc.rawData[AUX2] = (uint16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		rc.rawData[AUX3] = (uint16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		rc.rawData[AUX4] = (uint16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		rc.rawData[AUX5] = (uint16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		rc.rawData[AUX6] = (uint16_t)(*(data_buf+22)<<8)|*(data_buf+23);
	}

	if(*(data_buf+2)==0X10)								//PID1
	{
		pid_t pidval;
		pidval.kp = (uint16_t)(*(data_buf+4)<<8)|*(data_buf+5);
		pidval.ki = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		pidval.kd = (uint16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		Params_setRollPid(pidval);
		pidval.kp = (uint16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		pidval.ki = (uint16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		pidval.kd = (uint16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		Params_setPitchPid(pidval);
		pidval.kp = (uint16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		pidval.ki = (uint16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		pidval.kd = (uint16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		Params_setYawPid(pidval);
		Params_Save();
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		pid_t pidval;
		pidval.kp = (uint16_t)(*(data_buf+4)<<8)|*(data_buf+5);
		pidval.ki = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		pidval.kd = (uint16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		Params_setAltPid(pidval);
		pidval.kp = (uint16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		pidval.ki = (uint16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		pidval.kd = (uint16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		Params_setLevelPid(pidval);
		pidval.kp = (uint16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		pidval.ki = (uint16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		pidval.kd = (uint16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		Params_setMagPid(pidval);
		Params_Save();
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X16)								//OFFSET
	{

	}

/////////////////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0x18)					
	{

	}
}

void Commander::Data_Exchange(void)
{
	static uint8_t cnt = 0;
	
	switch(cnt)
	{
		case 1: 
			f.Send_RCData = 1;
			break;
		case 2:
			f.Send_MotoPwm = 1;
			break;
		case 30:
			cnt = 0;
			break;
		default:
			if(cnt%3)
				f.Send_Senser = 1;	
			else
				f.Send_Status = 1;
						
	}
	cnt++;
	
	if(f.Send_Status){
		f.Send_Status = 0;
		Send_Status();
	}	
	if(f.Send_Senser){
		f.Send_Senser = 0;
		Send_Senser();
	}	
	if(f.Send_RCData){
		f.Send_RCData = 0;
		Send_RCData();
	}		
	if(f.Send_MotoPwm){
		f.Send_MotoPwm = 0;
		Send_MotoPWM();
	}	
	if(f.Send_PID1){
		f.Send_PID1 = 0;
		Send_PID1();
	}	
	if(f.Send_PID2){
		f.Send_PID2 = 0;
		Send_PID2();
	}	
}

void Commander::Send_Status(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	uint16_t _temp;
	_temp = (int)(imu.angle.x*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(imu.angle.y*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(imu.angle.z*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)(1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	uint32_t _temp2 = 100;//UltraAlt * 100;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}


void Commander::Send_Senser(void)
{
	uint8_t _cnt=0;
	uint16_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	_temp = imu.Acc.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu.Acc.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu.Acc.z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Sensor_GetGyro().x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Sensor_GetGyro().y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Sensor_GetGyro().z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data(data_to_send, _cnt);
}

void Commander::Send_RCData(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(rc.rawData[THROTTLE]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[THROTTLE]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[YAW]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[YAW]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[ROLL]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[ROLL]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[PITCH]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[PITCH]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX1]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX1]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX2]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX2]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX3]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX3]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX4]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX4]);
	data_to_send[_cnt++]=0;//BYTE1(rc.rawData[AUX5]);
	data_to_send[_cnt++]=0;//BYTE0(rc.rawData[AUX5]);
	data_to_send[_cnt++]=0;//BYTE1(rc.rawData[AUX6]);
	data_to_send[_cnt++]=0;//BYTE0(rc.rawData[AUX6]);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Commander::Send_MotoPWM(void)
{
	uint8_t _cnt=0;
	uint16_t Moto_PWM[MOTORS_NUM_MAX];
	fc.getMotorsPWM(Moto_PWM);
	
	for(uint8_t i=0;i<MOTORS_NUM_MAX;i++)
		Moto_PWM[i] -= 1000;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Moto_PWM[0]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[0]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[1]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[1]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[2]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[2]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[3]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[3]);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;

	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Commander::Send_PID1(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	uint16_t _temp;
	_temp = fc.pid_group[PIDROLL].kP ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDROLL].kI ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDROLL].kD ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDPITCH].kP ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDPITCH].kI ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDPITCH].kD ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDYAW].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDYAW].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDYAW].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
}

void Commander::Send_PID2(void)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;
	
	uint16_t _temp;
	_temp = fc.pid_group[PIDALT].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDALT].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDALT].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDLEVEL].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDLEVEL].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDLEVEL].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDMAG].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDMAG].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid_group[PIDMAG].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Commander::Send_Check(uint16_t check)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=BYTE1(check);
	data_to_send[6]=BYTE0(check);
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<7;i++)
		sum += data_to_send[i];
	
	data_to_send[7]=sum;

	Send_Data(data_to_send, 8);
}
static rt_err_t data_rx_callback(rt_device_t dev, rt_size_t size)
{
  rt_event_send(cmd.recv_event,0x01);
	return RT_EOK;
}
 static void recv_thread_entry(void* parameter)
{
	rt_uint32_t ev =0;
	rt_err_t ret = RT_EOK;
	rt_uint16_t recv_len = 0;
  while(1)
  {
 ret =  rt_event_recv(cmd.recv_event,0x01,RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,&ev);
	if(ret == RT_EOK)
	{
	 recv_len = rt_device_read(cmd.dev,0,cmd.recv_buf,1024);
	
	if (recv_len > 0)
	{
		int index =0;
		while(index <(recv_len-4))
		{
			if((cmd.recv_buf[index]==0xaa)&&(cmd.recv_buf[index+1]==0xaf)&&(cmd.recv_buf[index+2]>0)&&(cmd.recv_buf[index+2]<0xf1))
			{
					if(cmd.recv_buf[index+3]<50)
					{
						uint8_t cmd_len = cmd.recv_buf[index+3];
						if(cmd_len+index<recv_len)
						{
							cmd.Data_Receive_Anl( &cmd.recv_buf[index],cmd_len+5);
							index+=cmd_len+5;
						}
						else
						{
							index++;
						}
					}
					else
					{
						index+=4;
					}
			}
			else
			{
			 index++;
			}
		}
	}
	}
	}
}

void Commander::Init(const char *name)
{
	rt_thread_t recv_thread;
	dev  = rt_device_find(name);
	if(dev == RT_NULL)
  {
	 rt_kprintf("can not find %s!\n", name);
		return ;
	}
	recv_event = rt_event_create("cmdrecv",RT_IPC_FLAG_FIFO);
	if(recv_event==RT_NULL)
  {
	 rt_kprintf("commander recv event create error!\n");
		return ;
	}
	rt_device_set_rx_indicate(dev,data_rx_callback);
	rt_device_open(dev,RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_RDWR);
	recv_thread = rt_thread_create("cmd_recv",recv_thread_entry,RT_NULL,1024,12,5);
	if(recv_thread != RT_NULL)
		rt_thread_startup(recv_thread);
}

void Commander::Send_Data(uint8_t *dataToSend , uint8_t length)
{
	rt_device_write(dev,0,dataToSend,length);
}


void Commander::Failsafe_Check(void)
{
		static uint8_t failsafeCnt = 0;
		if(failsafeCnt > 30)
		{
			failsafeCnt = 0;
			if(!config.f.failsafe)
				config.f.failsafe = 1;
			else
			{	
				config.f.ARMED = 0;
			}
		}
		failsafeCnt++;	
}




/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
