/**
 * @file Jetson_Orin.c
 * @author Leo Liu
 * @brief communicate and obtain data from Jetson
 * @version 1.0
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Jetson_Orin.h"
#include "State_Machine.h"

void Jetson_Orin_Handler(UART_HandleTypeDef *huart);
void Jetson_Orin_USART_Receive_DMA(UART_HandleTypeDef *huartx);
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);
void Jetson_Orin_Get_Data(void);
void Jetson_Orin_Send_Data(UART_HandleTypeDef *huart);

Orin_Func_t Orin_Func = Orin_Func_GroundInit;

Send_To_Orin_t Send_To_Orin;
Receive_From_Orin_t Receive_From_Orin;
uint8_t Orin_Rx_Buffer[20];
uint16_t Counter;
uint8_t spintop_flag;
#undef Orin_Func_GroundInit

void Jetson_Orin_Get_Data(void)
{
	Receive_From_Orin.Frame_Header = Orin_Rx_Buffer[0];
	if(Receive_From_Orin.Frame_Header == 0xAA)
	{
		Receive_From_Orin.Frame_Type = Orin_Rx_Buffer[1];
		switch(Receive_From_Orin.Frame_Type)
		{
			case 0:
				// Sentry will send either an auto-aim or nav package, we will set the mode depending on the package
				if (Chassis.Current_Mode != Auto_Aiming || Gimbal.Current_Mode != Auto_Aiming)
				{
					Chassis.Current_Mode = Spin_Top;
					Gimbal.Current_Mode = Auto_Aiming;
				}
			
				memcpy(&Receive_From_Orin.Auto_Aiming,&Orin_Rx_Buffer[4],sizeof(Receive_From_Orin.Auto_Aiming));
				break;
			
			case 1:
				// Sentry will send either an auto-aim or nav package, we will set the mode depending on the package
				if (Chassis.Current_Mode != Auto_Navigation || Gimbal.Current_Mode != Auto_Navigation)
				{
					Chassis.Current_Mode = Auto_Navigation;
					Gimbal.Current_Mode = Auto_Navigation;
				}
			
				memcpy(&Receive_From_Orin.Navigation,&Orin_Rx_Buffer[4],sizeof(Receive_From_Orin.Navigation));
				break;
			
			case 2:
				memcpy(&Receive_From_Orin.Heart_Beat,&Orin_Rx_Buffer[2],sizeof(Receive_From_Orin.Heart_Beat));
				break;
			
			default:
				break;
		}
	}
}

void Jetson_Orin_Send_Data(UART_HandleTypeDef *huart)
{
	Send_To_Orin.Frame_Header = 0xAA;
	Send_To_Orin.Pitch_Angle = Board_A_IMU.Export_Data.Pitch / 180.0f * PI;
	Send_To_Orin.Pitch_Angular_Rate = Board_A_IMU.Export_Data.Gyro_Pitch / 180.0f * PI;
	Send_To_Orin.Yaw_Angular_Rate = Board_A_IMU.Export_Data.Gyro_Yaw / 180.0f * PI;
	Send_To_Orin.Position_X = RBG_Pose.Position_X / 1000.0f;
	Send_To_Orin.Position_Y = RBG_Pose.Position_Y / 1000.0f;
	Send_To_Orin.Orientation = Board_A_IMU.Export_Data.Total_Yaw / 180.0f * PI;
	Send_To_Orin.Velocity_X = RBG_Pose.Velocity_X / 1000.0f;
	Send_To_Orin.Velocity_Y = RBG_Pose.Velocity_Y / 1000.0f;
	Send_To_Orin.Current_HP = Referee_Robot_State.Current_HP;

//	if(Receive_From_Orin.Frame_Header == 0xAA)
//		Send_To_Orin.Game_Start_Flag = 1;
//	counter++;
//	if(counter > 5000)
//		Send_To_Orin.Game_Start_Flag = 1;
	Send_To_Orin.Game_Start_Flag = (Referee_System.Game_Status.Progress == 4) ? 1 : 0; //4 for match begin
	Send_To_Orin.Enemy_Color_Flag = (Referee_System.Robot_State.ID > 11) ? 1 : 0; //ID > 11 means myself is blue, which means enemy is red
	Send_To_Orin.Supplier_Zone_Flag = (Referee_System.RFID.State & (1 << SUPPLIER_ZONE_SHIFT)) >> SUPPLIER_ZONE_SHIFT; //1 for Supplier Zone RFID detected
	Send_To_Orin.Central_Buff_Zone_Flag = (Referee_System.RFID.State & (1 << CENTRAL_BUFF_ZONE_SHIFT)) >> CENTRAL_BUFF_ZONE_SHIFT; //1 for Central Buff Zone Zone RFID detected

	if(Send_To_Orin.Game_Start_Flag && !spintop_flag)
	{
		Chassis.Current_Mode = Spin_Top;
		spintop_flag = 1;
	}
	HAL_UART_Transmit(&huart7,(uint8_t *)&Send_To_Orin,sizeof(Send_To_Orin_t),10);
}

static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
	if (huart->RxState == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
			return HAL_ERROR;
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode = HAL_UART_ERROR_NONE;

		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	}
	else
		return HAL_BUSY;
	return HAL_OK;
}

// Receive data if pass verification
void Jetson_Orin_Handler(UART_HandleTypeDef *huart)
{
	__HAL_DMA_DISABLE(huart->hdmarx);
	Jetson_Orin_Get_Data();
	__HAL_DMA_ENABLE(huart->hdmarx);
}

void Jetson_Orin_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
	__HAL_UART_CLEAR_IDLEFLAG(huartx);
	__HAL_UART_ENABLE(huartx);
	__HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
	USART_Receive_DMA_NO_IT(huartx, Orin_Rx_Buffer, sizeof(Orin_Rx_Buffer));
}
