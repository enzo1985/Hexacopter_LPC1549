#ifndef __ANO_DT_H
#define __ANO_DT_H
#include "lpc15xx.h"

#include <rtthread.h>

class Commander
{
	
public:
	void Init(const char *name);

	void Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
	//检查是否有接收到无线数据
	void Check_Event(void);
	//数据发送
	void Data_Exchange(void);
	//失控保护检查
	void Failsafe_Check(void);

	class flag{
		public:
		uint8_t Send_Status;
		uint8_t Send_Senser;
		uint8_t Send_PID1;
		uint8_t Send_PID2;
		uint8_t Send_PID3;
		uint8_t Send_RCData;
		uint8_t Send_Offset;
		uint8_t Send_MotoPwm;
	}f;
	rt_device_t dev;
  rt_event_t recv_event;
	rt_uint8_t data_to_send[64];
  rt_uint8_t recv_buf[512];
	
private:

	void Send_Status(void);
	void Send_Senser(void);
	void Send_RCData(void);
	void Send_MotoPWM(void);
	void Send_PID1(void);
	void Send_PID2(void);
	void Send_PID3(void);
	void Send_Check(uint16_t check);

	void Send_Data(uint8_t *dataToSend , uint8_t length);

};


extern Commander cmd;

#endif









