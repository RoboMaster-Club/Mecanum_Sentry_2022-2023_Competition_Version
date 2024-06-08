/**
 * @file Jetson_Orin.h
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

#define SUPPLIER_ZONE_SHIFT (13)
#define CENTRAL_BUFF_ZONE_SHIFT (19)

#define Orin_Func_GroundInit								\
	{																				\
					&Jetson_Orin_Get_Data,						\
					&Jetson_Orin_Send_Data,					\
					&Jetson_Orin_Handler,						\
					&Jetson_Orin_USART_Receive_DMA,	\
	}

typedef struct 
	{
		uint8_t Frame_Header; //0xAA
		float Pitch_Angle; //rad
		float Pitch_Angular_Rate; //rad/s
		float Yaw_Angular_Rate;	//rad/s
		float Position_X;	//m
    	float Position_Y; //m
    	float Orientation; //rad
		float Velocity_X; //m/s
		float Velocity_Y; //m/s
		uint16_t Current_HP;
		uint8_t Game_Start_Flag:1; //1 for start and 0 for not start
		uint8_t Enemy_Color_Flag:1; //1 for red and 0 for blue
		uint8_t Supplier_Zone_Flag:1; //1 for RFID detected
		uint8_t Central_Buff_Zone_Flag:1; //1 for RFID detected
	}__attribute__((packed)) Send_To_Orin_t;

typedef struct 
	{
		uint8_t Frame_Header; //0xAA 
		uint8_t Frame_Type; //0 for autoaiming, 1 for navigation, 2 for heart beat

		struct __attribute__ ((__packed__))
		{
			float X_Vel;	//m/s
			float Y_Vel; //m/s
			float Yaw_Angular_Rate;  //rad/s
			uint8_t State; // 0 for stationary, 1 for moving, 2 for spinning
		}Navigation;

		struct __attribute__ ((__packed__))
		{
			float Yaw;
			float Pitch;
			uint8_t Fire_Flag; //1 for fire, 0 for not fire
		}Auto_Aiming;

		struct __attribute__ ((__packed__))
		{
			uint8_t a;
			uint8_t b;
			uint8_t c;
			uint8_t d;
		}Heart_Beat;
	}__attribute__((packed)) Receive_From_Orin_t;

typedef struct
{
	void (*Jetson_Orin_Get_Data)(void);
	void (*Jetson_Orin_Send_Data)(UART_HandleTypeDef *huart);
	void (*Jetson_Orin_Handler)(UART_HandleTypeDef *huart);
	void (*Jetson_Orin_USART_Receive_DMA)(UART_HandleTypeDef *huartx);
}Orin_Func_t;

extern Orin_Func_t Orin_Func;
extern Send_To_Orin_t Send_To_Orin;
extern Receive_From_Orin_t Receive_From_Orin;

#endif
