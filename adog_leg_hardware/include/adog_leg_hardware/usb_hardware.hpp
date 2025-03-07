#ifndef USB_HARDWARE_HPP
#define USB_HARDWARE_HPP

#include "stdint.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "serialib.h"
#include <unistd.h>

#pragma pack(1)
typedef struct
{
	float pos;
	float speed;
	float tor;
	float kp;
	float kd;
} Send_DataTypeDef;

typedef struct usb
{
	uint8_t head[2];
	Send_DataTypeDef motor_ut0;
	Send_DataTypeDef motor_dm;
	Send_DataTypeDef motor_ut1;
	uint16_t crc;
} USB_SendPackageTypedef;

#pragma pack()

#pragma pack(1)
typedef struct
{
	float pos;
	float speed;
	float tor;
} Receive_DataTypeDef;

typedef struct
{
	const uint8_t head[2]={0xff,0xfe};
	Receive_DataTypeDef motor_ut0;
	Receive_DataTypeDef motor_dm;
	Receive_DataTypeDef motor_ut1;
	uint16_t crc;
} USB_ReceivePackageTypedef;
#pragma pack()
typedef class USB
{
private:
	serialib serial;
	uint8_t send_buff[sizeof(USB_SendPackageTypedef)] = {0};
	uint8_t receive_buff[sizeof(USB_ReceivePackageTypedef)] = {0};
	USB_SendPackageTypedef send_package;
	USB_ReceivePackageTypedef receive_package;

public:
	USB()
	{
		// send_package.head[0] = 0xff;
		// send_package.head[1] = 0xfd;

		send_package.motor_ut0.pos = 0;
		send_package.motor_ut0.speed = 0;
		send_package.motor_ut0.tor = 0;
		send_package.motor_ut0.kp = 0;
		send_package.motor_ut0.kd = 0;

		send_package.motor_dm.pos = 0;
		send_package.motor_dm.speed = 0;
		send_package.motor_dm.tor = 0;
		send_package.motor_dm.kp = 0;
		send_package.motor_dm.kd = 0;

		send_package.motor_ut1.pos = 0;
		send_package.motor_ut1.speed = 0;
		send_package.motor_ut1.tor = 0;
		send_package.motor_ut1.kp = 0;
		send_package.motor_ut1.kd = 0;

		send_package.crc = 0;

		// receive_package.head[0] = 0xff;
		// receive_package.head[1] = 0xfe;

		receive_package.motor_ut0.tor = 0;
		receive_package.motor_ut0.speed = 0;
		receive_package.motor_ut0.pos = 0;

		receive_package.motor_dm.tor = 0;
		receive_package.motor_dm.speed = 0;
		receive_package.motor_dm.pos = 0;

		receive_package.motor_ut1.tor = 0;
		receive_package.motor_ut1.speed = 0;
		receive_package.motor_ut1.pos = 0;

		receive_package.crc = 0;

	}
	USB(std::string serialDevice)
	{
		USB();
		char errorOpening = serial.openDevice(serialDevice.c_str(), 115200);
		if (errorOpening != 1)
		{	
			std::cout<<std::endl;
			while (1)
			{
				std::cout <<serialDevice.c_str()<< "usb open error" << std::endl;
				sleep(1);
			}
		}
	}

	void USB_Set_Date(USB_SendPackageTypedef& send_package)
	{
		memcpy(&this->send_package, &send_package, sizeof(USB_SendPackageTypedef));
	}

	const USB_ReceivePackageTypedef& USB_Get_Date(void)
	{
		return receive_package;
	}

	void USB_Send(void);
	int USB_Receive(void);
	void USB_Debug(void);
} USB;

#endif
