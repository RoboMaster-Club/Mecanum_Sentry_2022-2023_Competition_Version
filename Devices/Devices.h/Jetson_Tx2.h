/**
 * @file Jetson_Tx2.h
 * @author Leo Liu
 * @brief communicate and obtain data from Jetson
 * @version 1.0
 * @date 2022-10-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __JETSON_TX2_H
#define __JETSON_TX2_H

#include "dma.h"
#include "usart.h"
#include "Board_A_IMU.h"
#include "User_Defined_Math.h"
#include "Odometry.h"
#include "Referee_System.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define Tx2_Func_GroundInit								\
	{																				\
					&Jetson_Tx2_Get_Data,						\
					&Jetson_Tx2_Send_Data,					\
					&Jetson_Tx2_Handler,						\
					&Jetson_Tx2_USART_Receive_DMA,	\
	}

typedef struct
{
	uint8_t Rx_Buffer[20];
	uint8_t Tx_Buffer[34];
	
	struct
	{
		float Pitch_Angle; //rad
		float Pitch_Angular_Rate; //rad/s
		float Yaw_Angular_Rate;	//rad/s
		float Position_X;	//m
    float Position_Y; //m
    float Orientation; //rad
		float Velocity_X; //m/s
		float Velocity_Y; //m/s
		uint8_t Game_Start_Flag; //1 for start and 0 for not start
		uint8_t Enemy_Color_Flag; //1 for red and 0 for blue
		
		union
		{
			float data[8];
			uint8_t Data[32];
		}Raw_Data;
	}Sending;
	
	struct
	{
		uint8_t Frame_ID;
		uint8_t Frame_Type; //0 for autoaiming, 1 for navigation, 2 for heart beat
		
		struct
		{
			float X_Vel;	//m/s
			float Y_Vel; //m/s
			float Yaw_Angular_Rate;  //rad/s
			uint8_t State; // 0 for stationary, 1 for moving, 2 for spinning
		}Navigation;
			
		struct
		{
			float Yaw;
			float Pitch;
		}Auto_Aiming;
			
		struct
		{
			uint8_t a;
			uint8_t b;
			uint8_t c;
			uint8_t d;
		}Heart_Beat;
		
		union
		{
			float data[3];
			uint8_t Data[12];
		}Raw_Data;
	}Receiving;
	
}Tx2_Data_t;

typedef struct
{
	void (*Jetson_Tx2_Get_Data)(void);
	void (*Jetson_Tx2_Send_Data)(UART_HandleTypeDef *huart);
	void (*Jetson_Tx2_Handler)(UART_HandleTypeDef *huart);
	void (*Jetson_Tx2_USART_Receive_DMA)(UART_HandleTypeDef *huartx);
}Tx2_Func_t;


extern Tx2_Data_t Tx2_Data;
extern Tx2_Func_t Tx2_Func;

#endif
