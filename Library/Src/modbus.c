/*
 * modbus.c
 *
 *  Created on: Dec 15, 2022
 *      Author: DELL
 */

#include "modbus.h"
#include "main.h"

extern UART_HandleTypeDef huart2;


uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_lo ^ *buffer++; /* calculate the CRC  */
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

// buffer: data nhan duoc de kiem tra checksum
// index: size cua data
// Vd truyen vao data có 7 byte: index = 7
uint8_t checkcrc16(uint8_t *buffer, uint8_t index)
{
	uint8_t transform[index], a;
	for (int i = 0; i < index - 2; i++)
	{
		transform[i] = buffer[i];
	}
	uint16_t crc = crc16(transform, index - 2);
	transform[index - 2] = crc&0xFF;   // CRC LOW
	transform[index - 1] = (crc>>8)&0xFF;  // CRC HIGH
	if ((transform[index - 2] == buffer[index - 2]) && (transform[index - 2] == buffer[index - 2]))
	{
		a = 1;
	}
	else
	{
		a = 0;
	}
	return a;
}

uint32_t convertToInt(uint32_t* arr, int low, int high)
{
	uint32_t f = 0;
	int i;
    for (i = high; i >= low; i--) {
        f = f + arr[i] * pow(2, high - i);
    }
    return f;
}

// Chuyen float 32 bit sang so thuc
float unpack754_32( uint32_t floatingToIntValue )
{
	 myfloat ieee754;
	 unsigned int mantissa = 0;
	 unsigned int exponent = 0 ;
	 unsigned int sign = 0;

	 sign = NTH_BIT(floatingToIntValue, 31);
	 for( int ix=0; ix<8; ix++)
	   exponent = (exponent | (NTH_BIT(floatingToIntValue, (30-ix))))<<1;
	 exponent = exponent>>1;
	 for( int ix=0; ix<23; ix++)
	   mantissa = (mantissa | (NTH_BIT(floatingToIntValue, (22-ix))))<<1;
	 mantissa = mantissa >> 1;

	 ieee754.raw.sign = sign;
	 ieee754.raw.exponent = exponent;
	 ieee754.raw.mantissa = mantissa;
	 return ieee754.f;
}


void Read_HodingRes(uint8_t SlaveAddr,uint16_t RegAddr, uint8_t Quantity)
{
	uint8_t TxData[8];
	TxData[0] = SlaveAddr;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = RegAddr >> 8;
	TxData[3] = RegAddr & 0xFF;
	// 0x0200: Error Status Address

	TxData[4] = 0;
	TxData[5] = Quantity;
	// no of registers to read will be 00000000 00000001 = 1 Registers = 2 Bytes

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}

/********************************************************************************************
 * ******************************************************************************************
 */
void Send_Error_Status(void)
{
	uint8_t TxData[8];
	TxData[0] = 0xF0;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = 0x02;
	TxData[3] = 0;
	// 0x0200: Error Status Address

	TxData[4] = 0;
	TxData[5] = 0x01;
	// no of registers to read will be 00000000 00000001 = 1 Registers = 2 Bytes

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}
uint32_t Receive_Error_Status(uint16_t data)
{
	if (data == 0x01)
		return No_Error;
	else
		return Have_Errors;
}

/********************************************************************************************
 * ******************************************************************************************
 */
void Send_Error_Code(void)
{
	uint8_t TxData[8];
	TxData[0] = 0xF0;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = 0x02;
	TxData[3] = 0x03;
	// 0x0203: Error Code

	TxData[4] = 0;
	TxData[5] = 0x02;

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}
uint32_t Receive_Error_Code(uint16_t data)
{
	if (data == 0)
		return Status_OK;
	else if (data == 0x01)
		return Temp_Measur_Err;
	else if (data == 0x02)
		return Hum_Measur_Err;
	else if (data == 0x04)
		return Humi_Sen_Fail;
	else if (data == 0x08)
		return Capa_Re_Err;
	else if (data == 0x10)
		return Temp_Out_Range;
	else
		return Sensor_Heat_Err;

}

/********************************************************************************************
 * ******************************************************************************************
 */
void Send_Error_Temp(void)
{
	uint8_t TxData[8];
	TxData[0] = 0xF0;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = 0x02;
	TxData[3] = 0x08;
	// T measurement status

	TxData[4] = 0;
	TxData[5] = 0x01;

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}
uint8_t Receive_Error_Temp(uint16_t data)
{
	if (data == 0)
		return Status_OK;
	else if (data == 0x01)
		return Measur_Not_Avai;
	else if (data == 0x02)
		return Measur_Not_Reliable;
	else if (data == 0x04)
		return UnderRange;
	else if (data == 0x08)
		return Overrange;
	else if (data == 0x20)
		return Value_Locked;
	else if (data == 0x80)
		return Sensor_Failure;
	else
		return Measur_Not_Ready;
}

/********************************************************************************************
 * ******************************************************************************************
 */
void Send_Error_RH(void)
{
	uint8_t TxData[8];
	TxData[0] = 0xF0;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = 0x02;
	TxData[3] = 0x07;
	// RH measurement status

	TxData[4] = 0;
	TxData[5] = 0x01;
	// no of registers to read will be 00000000 00000001 = 1 Registers = 2 Bytes

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}
uint8_t Receive_Error_RH(uint16_t data)
{
	if (data == 0)
		return Status_OK;
	else if (data == 0x01)
		return Measur_Not_Avai;
	else if (data == 0x02)
		return Measur_Not_Reliable;
	else if (data == 0x04)
		return UnderRange;
	else if (data == 0x08)
		return Overrange;
	else if (data == 0x20)
		return Value_Locked;
	else if (data == 0x80)
		return Sensor_Failure;
	else
		return Measur_Not_Ready;
}
/********************************************************************************************
 * ******************************************************************************************
 */
void Send_Error_Device(void)
{
	uint8_t TxData[8];
	TxData[0] = 0xF0;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = 0x02;
	TxData[3] = 0x0A;
	//Error Device

	TxData[4] = 0;
	TxData[5] = 0x01;
	// no of registers to read will be 00000000 00000001 = 1 Registers = 2 Bytes

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}
uint8_t Receive_Error_Device(uint16_t data)
{
	if (data == 0)
		return Status_OK;
	else if (data == 0x01)
		return Need_Maintance;
	else if (data == 0x02)
		return Recover_Auto;
	else if (data == 0x04)
		return Warning;
	else if (data == 0x08)
		return Notification;
	else
		return Calib_Mode_Active;
}

/********************************************************************************************
 * ******************************************************************************************
 */
void Read_Temp(void)
{
	uint8_t TxData[8];
	TxData[0] = 0xF0;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = 0;
	TxData[3] = 0x02;
	//  Temp Address

	TxData[4] = 0;
	TxData[5] = 0x02;

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}
void Receive_Temp(uint32_t data)
{
	unpack754_32(data);
}

/********************************************************************************************
 * ******************************************************************************************
 */
void Read_Water(void)
{
	uint8_t TxData[8];
	TxData[0] = 0xF0;  // slave address
	TxData[1] = 0x03;  // Function code for Read Input Registers

	TxData[2] = 0;
	TxData[3] = 0x22;
	// Ham luong nuoc trong dau

	TxData[4] = 0;
	TxData[5] = 0x02;

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc&0xFF;   // CRC LOW
	TxData[7] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, TxData, 8, 1000);
}
void Receive_Water(uint32_t data)
{
	unpack754_32(data);
}
