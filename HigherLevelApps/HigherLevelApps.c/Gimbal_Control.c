/**
 * @file Gimbal_Control.c
 * @author Leo Liu
 * @brief control gimbal
 * @version 1.0
 * @date 2022-07-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Gimbal_Control.h"
#include "Odometry.h"

void Gimbal_Init(void);
void Gimbal_Control_Get_Data(Gimbal_t *Gimbal);
void Gimbal_Processing(Gimbal_t *Gimbal);
void SpeedCalc(void const *argument);

Gimbal_Func_t Gimbal_Func = Gimbal_Func_GroundInit;
#undef Gimbal_Func_GroundInit

Gimbal_t Gimbal;
void Gimbal_Init(void)
{
	// Set the origin for yaw and pitch
	Gimbal.Target_Yaw = Board_A_IMU.Export_Data.Total_Yaw;
	Gimbal.Target_Pitch = PITCH_MID_MECH_ANGLE;
}

void Gimbal_Control_Get_Data(Gimbal_t *Gimbal)
{
	// The multiplying/dividing constant are tested value and can be changed
	if (State_Machine.Control_Source == Remote_Control)
	{
		Gimbal->Target_Yaw = Gimbal->Target_Yaw * 0.7f + 0.3f * (Gimbal->Target_Yaw - DR16_Export_Data.Remote_Control.Joystick_Right_Vx / 650.0f);

		// Set Current Yaw as Target when stop turn command (prevent yaw overshooting)
		if (DR16_Export_Data.Remote_Control.Joystick_Right_Vx)
		{
			Gimbal->On_Command = true;
		}
		if (Gimbal->On_Command && (DR16_Export_Data.Remote_Control.Joystick_Right_Vx == 0))
		{
			Gimbal->On_Command = false;
			Gimbal->Target_Yaw = Gimbal->Current_Yaw;
		}
		Gimbal->Target_Pitch += DR16_Export_Data.Remote_Control.Joystick_Right_Vy / 2200.0f;
		Gimbal->Target_Pitch = VAL_LIMIT(Gimbal->Target_Pitch, PITCH_UPPER_LIMIT, PITCH_LOWER_LIMIT);
	}

	else if (State_Machine.Control_Source == Computer)
	{
		Gimbal->Target_Yaw -= (float)DR16_Export_Data.Mouse.x / 60.0f;
		Gimbal->Target_Pitch -= (float)DR16_Export_Data.Mouse.y / 5.0f;
		Gimbal->Target_Pitch = VAL_LIMIT(Gimbal->Target_Pitch, PITCH_UPPER_LIMIT, PITCH_LOWER_LIMIT);
	}
}

void Gimbal_Processing(Gimbal_t *Gimbal)
{
	if (State_Machine.Robot_Ready_Flag)
	{
		switch (Gimbal->Current_Mode)
		{
		case (Follow_Gimbal):
		{
			// Reset the target angle so gimbal doesn't spin like crazy
			if (Gimbal->Prev_Mode != Follow_Gimbal)
			{
				Gimbal->Target_Yaw = Gimbal->Current_Yaw;
			}
			// soft update
			Gimbal->Target_Yaw_Speed = Gimbal->Target_Yaw_Speed * 0.85f + 0.15f * PID_Func.Positional_PID_Min_Error(&Yaw_Angle_Follow_PID, Gimbal->Target_Yaw, Gimbal->Current_Yaw, 0.0);
			GM6020_Yaw.Output_Current = GM6020_Yaw.Output_Current * 0.85f + 0.15f * PID_Func.Positional_PID_Min_Error(&Yaw_Speed_Follow_PID, Gimbal->Target_Yaw_Speed, Gimbal->Current_Yaw_Speed, 0.0);
			
			Gimbal->Current_Pitch = PITCH_DIRECTION * Board_A_IMU.Export_Data.Pitch;
			Gimbal->Pitch_Angle_Output = PID_Func.Positional_PID(&Pitch_Angle_PID, Gimbal->Target_Pitch, -Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&AutoAim_Pitch_Speed_PID, Gimbal->Pitch_Angle_Output, -Board_A_IMU.Export_Data.Gyro_Pitch);

			Gimbal->Prev_Mode = Follow_Gimbal;

			break;
		}
		
		case (Auto_Aiming):
		{
			// Reset the target angle so gimbal doesn't spin like crazy
			if (Gimbal->Prev_Mode != Auto_Aiming)
			{
				Gimbal->Target_Yaw = Gimbal->Current_Yaw;
			}
			// soft update
			Gimbal->Target_Yaw = -Tx2_Data.Receiving.Auto_Aiming.Yaw;
			Gimbal->Target_Yaw_Speed = PID_Func.Positional_PID(&AutoAim_Yaw_Angle_PID,Gimbal->Target_Yaw,0);
			Gimbal->Current_Yaw_Speed = Gimbal->Current_Yaw_Speed*0.9f + 0.1f*GM6020_Yaw.Actual_Speed;
			GM6020_Yaw.Output_Current = PID_Func.Positional_PID(&AutoAim_Yaw_Speed_PID,Gimbal->Target_Yaw_Speed,Gimbal->Current_Yaw_Speed);
			
			Gimbal->Target_Pitch = -Tx2_Data.Receiving.Auto_Aiming.Pitch;
			Gimbal->Target_Pitch_Speed = PID_Func.Positional_PID(&AutoAim_Pitch_Angle_PID,Gimbal->Target_Pitch,0);
			Gimbal->Current_Pitch_Speed = -(Gimbal->Current_Pitch_Speed*0.9f + 0.1f*GM6020_Pitch.Actual_Speed);
			GM6020_Pitch.Output_Current = -PID_Func.Positional_PID(&AutoAim_Pitch_Speed_PID,Gimbal->Target_Pitch_Speed,Gimbal->Current_Pitch_Speed);

			Gimbal->Prev_Mode = Auto_Aiming;

			break;
		}
		
		case (Auto_Navigation):
		{
			// Reset the target angle so gimbal doesn't spin like crazy
			if (Gimbal->Prev_Mode != Auto_Navigation)
			{
				Gimbal->Target_Yaw = Gimbal->Current_Yaw;
			}
			Gimbal->Target_Yaw += Tx2_Data.Receiving.Navigation.Yaw_Angular_Rate /PI * 180.0f / 500.0f; 
			// soft update
			Gimbal->Target_Yaw_Speed = Gimbal->Target_Yaw_Speed * 0.85f + 0.15f * PID_Func.Positional_PID_Min_Error(&Yaw_Angle_Follow_PID, Gimbal->Target_Yaw, Gimbal->Current_Yaw, 0.0);
			GM6020_Yaw.Output_Current = GM6020_Yaw.Output_Current * 0.85f + 0.15f * PID_Func.Positional_PID_Min_Error(&Yaw_Speed_Follow_PID, Gimbal->Target_Yaw_Speed, Gimbal->Current_Yaw_Speed, 0.0);
			Gimbal->Current_Pitch = PITCH_DIRECTION * Board_A_IMU.Export_Data.Pitch;
			Gimbal->Pitch_Angle_Output = PID_Func.Positional_PID(&Pitch_Angle_PID, Gimbal->Target_Pitch, -Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&AutoAim_Pitch_Speed_PID, Gimbal->Pitch_Angle_Output, -Board_A_IMU.Export_Data.Gyro_Pitch);

			Gimbal->Prev_Mode = Auto_Navigation;

			break;
		}
		
		 

		case (Spin_Top):
		{
			// Reset the target angle so gimbal doesn't spin like crazy
			if (Gimbal->Prev_Mode != Spin_Top)
				Gimbal->Target_Yaw = Gimbal->Current_Yaw;

			Gimbal->Target_Yaw = Gimbal->Target_Yaw * 0.5f + 0.5f * (Gimbal->Target_Yaw - DR16_Export_Data.Remote_Control.Joystick_Right_Vx / 3500.0f);
			// Gimbal is held in position through yaw encoder
			Gimbal->Target_Yaw_Speed = Gimbal->Target_Yaw_Speed * 0.8f + 0.2f * PID_Func.Positional_PID_Min_Error(&Yaw_Angle_Spin_PID, Gimbal->Target_Yaw, Gimbal->Current_Yaw, 0);
			GM6020_Yaw.Output_Current = GM6020_Yaw.Output_Current * 0.8f + 0.2f * PID_Func.Positional_PID_Min_Error(&Yaw_Speed_Spin_PID, Gimbal->Target_Yaw_Speed, Gimbal->Current_Yaw_Speed, 0);

			Gimbal->Current_Pitch = PITCH_DIRECTION * Board_A_IMU.Export_Data.Pitch;
			Gimbal->Pitch_Angle_Output = PID_Func.Positional_PID(&Pitch_Angle_PID, Gimbal->Target_Pitch, -Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&AutoAim_Pitch_Speed_PID, Gimbal->Pitch_Angle_Output, -Board_A_IMU.Export_Data.Gyro_Pitch);

			Gimbal->Prev_Mode = Spin_Top;

			break;
		}

		case (Disabled):
		{
			Gimbal->Target_Yaw = Gimbal->Current_Yaw;
			GM6020_Yaw.Output_Current = 0;
			GM6020_Pitch.Output_Current = 0;
			PID_Func.Clear_PID_Data(&Yaw_Angle_Follow_PID);
			PID_Func.Clear_PID_Data(&Pitch_Angle_PID);

			Gimbal->Prev_Mode = Disabled;

			break;
		}
		}
	}
	else
	{
		// slowly move yaw into mid, to prevent potential damage

		Gimbal->Target_Yaw = (fmod(fmod(RBG_Pose.Orientation_Degree - Gimbal->Current_Yaw, 360.0f) + 360.f, 360.0f) > 180.0f) ? (fmod(fmod(RBG_Pose.Orientation_Degree - Gimbal->Current_Yaw, 360.0f) + 360.f, 360.0f) - 360.0f) : (fmod(fmod(RBG_Pose.Orientation_Degree - Gimbal->Current_Yaw, 360.0f) + 360.f, 360.0f));
		Gimbal->Target_Yaw_Speed = Gimbal->Target_Yaw_Speed * 0.85f + 0.15f * PID_Func.Positional_PID_Min_Error(&Yaw_Angle_Start_PID, Gimbal->Target_Yaw, 0.0, 0.0);
		GM6020_Yaw.Output_Current = GM6020_Yaw.Output_Current * 0.85f + 0.15f * PID_Func.Positional_PID_Min_Error(&Yaw_Speed_Start_PID, Gimbal->Target_Yaw_Speed, Gimbal->Current_Yaw_Speed, 0.0);

		// move pitch to mid
		Gimbal->Target_Pitch = 0;
		Gimbal->Target_Pitch_Speed = Gimbal->Target_Pitch_Speed * 0.85f + 0.15f * PID_Func.Positional_PID_Min_Error(&Pitch_Angle_PID, Gimbal->Target_Pitch, Gimbal->Current_Pitch, 0.0);
		GM6020_Pitch.Output_Current = GM6020_Pitch.Output_Current * 0.6f + 0.4f * PID_Func.Positional_PID_Min_Error(&Pitch_Speed_PID, Gimbal->Target_Pitch_Speed, Gimbal->Current_Pitch_Speed, 0.0);

		// if yaw turn to mid, set state machine to ready
		if (Gimbal->Target_Yaw < 2.0f)
		{
			State_Machine.Robot_Ready_Flag = true;
		}
	}
}

void SpeedCalc(void const *argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const uint8_t time_step = 2;
	const TickType_t TimeIncrement = pdMS_TO_TICKS(time_step);

	float yawBuffer[11];  // Yaw Buffer
	double yawBufferMean; // Buffer For Linear Regression
	double yawSum1;		  // Sum For Linear Regression
	double sum2 = 33.0f;  // Linear Regression Param

	float pitchBuffer[11];	// pitch Buffer
	double pitchBufferMean; // Buffer For Linear Regression
	double pitchSum1;		// Sum For Linear Regression

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

		pitchBufferMean = 0;
		pitchSum1 = 0;
		for (int i = 1; i < 11; i++)
		{
			pitchBuffer[i - 1] = pitchBuffer[i];
			pitchBufferMean += pitchBuffer[i];
		}
		Gimbal.Current_Pitch = ((GM6020_Pitch.Total_Angle - PITCH_MID_MECH_ANGLE) / GM6020_MECH_ANGLE_MAX * 360.0f);
		pitchBuffer[9] = Gimbal.Current_Pitch;
		pitchBufferMean += pitchBuffer[9];
		pitchBufferMean /= 10;
		for (int i = 0; i < 10; i++)
		{
			pitchSum1 += (pitchBuffer[i] - pitchBufferMean) * (i * 0.002 - 0.009);
		}

		Gimbal.Current_Pitch_Speed = pitchSum1 / sum2 * 10000;

		yawBufferMean = 0;
		yawSum1 = 0;
		for (int i = 1; i < 11; i++)
		{
			yawBuffer[i - 1] = yawBuffer[i];
			yawBufferMean += yawBuffer[i];
		}
		Gimbal.Current_Yaw = Gimbal.Current_Yaw * 0.7f + 0.3f * (YAW_DIRECTION * RBG_Pose.Orientation_Degree + ((GM6020_Yaw.Total_Angle - YAW_MID_MECH_ANGLE) / GM6020_MECH_ANGLE_MAX) * 360.0f);
		yawBuffer[9] = Gimbal.Current_Yaw;
		yawBufferMean += yawBuffer[9];
		yawBufferMean /= 10;
		for (int i = 0; i < 10; i++)
		{
			yawSum1 += (yawBuffer[i] - yawBufferMean) * (i * 0.002 - 0.009);
		}

		Gimbal.Current_Yaw_Speed = yawSum1 / sum2 * 10000;
	}
}
