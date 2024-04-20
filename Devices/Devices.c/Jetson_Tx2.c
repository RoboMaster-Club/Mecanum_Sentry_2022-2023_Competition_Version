/**
 * @file Jetson_Tx2.c
 * @author Leo Liu
 * @brief communicate and obtain data from Jetson
 * @version 1.0
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Jetson_Tx2.h"

void Jetson_Tx2_Handler(UART_HandleTypeDef *huart);
void Jetson_Tx2_USART_Receive_DMA(UART_HandleTypeDef *huartx);
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);
void Jetson_Tx2_Get_Data(void);
void Jetson_Tx2_Send_Data(UART_HandleTypeDef *huart);

Tx2_Func_t Tx2_Func = Tx2_Func_GroundInit;
Tx2_Data_t Tx2_Data = {0};

#undef Tx2_Func_GroundInit

void Jetson_Tx2_Get_Data(void)
{
	if(Tx2_Data.Rx_Buffer[0] == 0xAA)
	{
		Tx2_Data.Receiving.Frame_ID = Tx2_Data.Rx_Buffer[0];
		Tx2_Data.Receiving.Frame_Type = Tx2_Data.Rx_Buffer[1];
		switch(Tx2_Data.Receiving.Frame_Type)
		{
			case 0:
				memcpy(&Tx2_Data.Receiving.Raw_Data.Data[0],&Tx2_Data.Rx_Buffer[4],8*sizeof(uint8_t));
				Tx2_Data.Receiving.Auto_Aiming.Yaw = Tx2_Data.Receiving.Raw_Data.data[0];
				Tx2_Data.Receiving.Auto_Aiming.Pitch = Tx2_Data.Receiving.Raw_Data.data[1];
				break;
			
			case 1:
				memcpy(&Tx2_Data.Receiving.Raw_Data.Data[0],&Tx2_Data.Rx_Buffer[4],12*sizeof(uint8_t));
				Tx2_Data.Receiving.Navigation.X_Vel = Tx2_Data.Receiving.Raw_Data.data[0];
				Tx2_Data.Receiving.Navigation.Y_Vel = Tx2_Data.Receiving.Raw_Data.data[1];
				Tx2_Data.Receiving.Navigation.Yaw_Angular_Rate = Tx2_Data.Receiving.Raw_Data.data[2];
				Tx2_Data.Receiving.Navigation.State = Tx2_Data.Rx_Buffer[16];
				break;
			
			case 2:
				Tx2_Data.Receiving.Heart_Beat.a = Tx2_Data.Rx_Buffer[2];
				Tx2_Data.Receiving.Heart_Beat.b = Tx2_Data.Rx_Buffer[3];
				Tx2_Data.Receiving.Heart_Beat.c = Tx2_Data.Rx_Buffer[4];
				Tx2_Data.Receiving.Heart_Beat.d = Tx2_Data.Rx_Buffer[5];
				break;
			
			default:
				break;
		}
	}
}

void Jetson_Tx2_Send_Data(UART_HandleTypeDef *huart)
{
	Tx2_Data.Sending.Pitch_Angle = Board_A_IMU.Export_Data.Pitch / 180.0f * PI;
	Tx2_Data.Sending.Pitch_Angular_Rate = Board_A_IMU.Export_Data.Gyro_Pitch / 180.0f * PI;
	Tx2_Data.Sending.Yaw_Angular_Rate = Board_A_IMU.Export_Data.Gyro_Yaw / 180.0f * PI;
	Tx2_Data.Sending.Position_X = RBG_Pose.Position_X / 1000.0f;
	Tx2_Data.Sending.Position_Y = RBG_Pose.Position_Y / 1000.0f;
	Tx2_Data.Sending.Orientation = RBG_Pose.Orientation;
	Tx2_Data.Sending.Velocity_X = RBG_Pose.Velocity_X / 1000.0f;
	Tx2_Data.Sending.Velocity_Y = RBG_Pose.Velocity_Y / 1000.0f;
	Tx2_Data.Sending.Game_Start_Flag = (Referee_System.Game_Status.Progress == 4) ? 1 : 0; //4 for match begin
	Tx2_Data.Sending.Enemy_Color_Flag = (Referee_System.Robot_State.ID > 11) ? 1 : 0; //ID > 11 means myself is blue, which means enemy is red
	
	Tx2_Data.Sending.Raw_Data.data[0] = Tx2_Data.Sending.Pitch_Angle;
	Tx2_Data.Sending.Raw_Data.data[1] = Tx2_Data.Sending.Pitch_Angular_Rate;
	Tx2_Data.Sending.Raw_Data.data[2] = Tx2_Data.Sending.Yaw_Angular_Rate;
	Tx2_Data.Sending.Raw_Data.data[3] = Tx2_Data.Sending.Position_X;
	Tx2_Data.Sending.Raw_Data.data[4] = Tx2_Data.Sending.Position_Y;
	Tx2_Data.Sending.Raw_Data.data[5] = Tx2_Data.Sending.Orientation;
	Tx2_Data.Sending.Raw_Data.data[6] = Tx2_Data.Sending.Velocity_X;
	Tx2_Data.Sending.Raw_Data.data[7] = Tx2_Data.Sending.Velocity_Y;
	
	Tx2_Data.Tx_Buffer[0] = 0xAA;
	Tx2_Data.Tx_Buffer[1] = Tx2_Data.Sending.Enemy_Color_Flag << 1 | Tx2_Data.Sending.Game_Start_Flag;
	memcpy(&Tx2_Data.Tx_Buffer[2],&Tx2_Data.Sending.Raw_Data.Data[0],32*sizeof(uint8_t));
	
	HAL_UART_Transmit(&huart7, Tx2_Data.Tx_Buffer, sizeof(Tx2_Data.Tx_Buffer),10);
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
void Jetson_Tx2_Handler(UART_HandleTypeDef *huart)
{
	__HAL_DMA_DISABLE(huart->hdmarx);
	Jetson_Tx2_Get_Data();
	__HAL_DMA_ENABLE(huart->hdmarx);
}

void Jetson_Tx2_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
	__HAL_UART_CLEAR_IDLEFLAG(huartx);
	__HAL_UART_ENABLE(huartx);
	__HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
	USART_Receive_DMA_NO_IT(huartx, Tx2_Data.Rx_Buffer, sizeof(Tx2_Data.Rx_Buffer));
}
