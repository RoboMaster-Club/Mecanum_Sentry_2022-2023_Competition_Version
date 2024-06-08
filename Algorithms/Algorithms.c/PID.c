/**
 * @file PID.c
 * @author Leo Liu
 * @brief PID Algorithms
 * @version 1.0
 * @date 2022-07-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "PID.h"

float Positional_PID(PID_t *PID, float Target_Value, float Measured_Value);
float Positional_PID_Min_Error(PID_t *PID, float Target_Value, float Measured_Value, float Min_Error);
float Incremental_PID(PID_t *PID, float Target_Value, float Measured_Value);
void Clear_PID_Data(PID_t *PID);

// Initializing all the used PID parameters in the header file
PID_t Chassis_Angle_PID = Chassis_Angle_PIDInit;
#undef Chassis_Angle_PIDInit

PID_t Chassis_Small_Angle_PID = Chassis_Small_Angle_PIDInit;
#undef Chassis_Small_Angle_PIDInit

PID_t Chassis_Speed_PID = Chassis_Speed_PIDInit;
#undef Chassis_Angle_PIDInit

PID_t Fric_Wheel_PID = Fric_Wheel_PIDInit;
#undef Fric_Wheel_PIDInit

PID_t Yaw_Angle_Follow_PID = Yaw_Angle_Follow_PIDInit;
#undef Yaw_Angle_Follow_PIDInit

PID_t Yaw_Speed_Follow_PID = Yaw_Speed_Follow_PIDInit;
#undef Yaw_Speed_Follow_PIDInit

PID_t Yaw_Angle_Spin_PID = Yaw_Angle_Spin_PIDInit;
#undef Yaw_Angle_Spin_PIDInit

PID_t Yaw_Speed_Spin_PID = Yaw_Speed_Spin_PIDInit;
#undef Yaw_Speed_Spin_PIDInit

PID_t Yaw_Angle_Start_PID = Yaw_Angle_Reset_PIDInit;
#undef Yaw_Angle_Reset_PIDInit

PID_t Yaw_Speed_Start_PID = Yaw_Speed_Reset_PIDInit;
#undef Yaw_Speed_Reset_PIDInit

PID_t Pitch_Angle_PID = Pitch_Angle_PIDInit;
#undef Pitch_Angle_PIDInit

PID_t Pitch_Speed_PID = Pitch_Speed_PIDInit;
#undef Pitch_Speed_PIDInit

PID_t Trigger_Angle_Left_PID = Trigger_Angle_PIDInit;
PID_t Trigger_Angle_Right_PID = Trigger_Angle_PIDInit;
#undef Trigger_Angle_PIDInit

PID_t Trigger_Speed_PID = Trigger_Speed_PIDInit;
#undef Trigger_Speed_PIDInit

PID_t AutoAim_Pitch_Angle_PID = AutoAim_Pitch_Angle_PIDInit;
#undef AutoAim_Pitch_Angle_PIDInit

PID_t AutoAim_Pitch_Speed_PID = AutoAim_Pitch_Speed_PIDInit;
#undef AutoAim_Pitch_Speed_PIDInit

PID_t AutoAim_Yaw_Angle_PID = AutoAim_Yaw_Angle_PIDInit;
#undef AutoAim_Yaw_Angle_PIDInit

PID_t AutoAim_Yaw_Speed_PID = AutoAim_Yaw_Speed_PIDInit;
#undef AutoAim_Yaw_Speed_PIDInit

PID_Func_t PID_Func = PID_Func_GroundInit;
#undef PID_Func_GroundInit

float Positional_PID(PID_t *PID, float Target_Value, float Measured_Value)
{
	PID->Target_Value = Target_Value;
	PID->Measured_Value = Measured_Value;
	PID->Prev_Error = PID->Error;
	PID->Error = PID->Target_Value - PID->Measured_Value;

	PID->P_Out = PID->Kp * PID->Error;
	PID->I_Out += PID->Ki * PID->Error;
	PID->D_Out = PID->Kd * (PID->Error - PID->Prev_Error);

	PID->I_Out = VAL_LIMIT(PID->I_Out, PID->I_Out_Max, -PID->I_Out_Max);
	PID->Output = (PID->P_Out + PID->I_Out + PID->D_Out);
	PID->Output = VAL_LIMIT(PID->Output, PID->Output_Max, -PID->Output_Max);

	return PID->Output;
}

float Positional_PID_Min_Error(PID_t *PID, float Target_Value, float Measured_Value, float Min_Error)
{
	PID->Target_Value = Target_Value;
	PID->Measured_Value = Measured_Value;
	PID->Prev_Error = PID->Error;
	PID->Error = PID->Target_Value - PID->Measured_Value;

	if (abs(PID->Error) < Min_Error)
	{
		PID->Error = 0;
	}

	PID->P_Out = PID->Kp * PID->Error;
	PID->I_Out += PID->Ki * PID->Error;
	PID->D_Out = PID->Kd * (PID->Error - PID->Prev_Error);

	PID->I_Out = VAL_LIMIT(PID->I_Out, PID->I_Out_Max, -PID->I_Out_Max);
	PID->Output = (PID->P_Out + PID->I_Out + PID->D_Out);
	PID->Output = VAL_LIMIT(PID->Output, PID->Output_Max, -PID->Output_Max);

	return PID->Output;
}

float Incremental_PID(PID_t *PID, float Target_Value, float Measured_Value)
{
	static float errorBuffer[20];
	float bufferMean = 0;
	float errorSum1 = 0;
	for (int i = 1; i < 21; i++)
	{
		errorBuffer[i - 1] = errorBuffer[i];
		bufferMean += errorBuffer[i];
	}

	errorBuffer[9] = Target_Value - Measured_Value;
	bufferMean += errorBuffer[19];
	bufferMean /= 20;
	for (int i = 0; i < 20; i++)
	{
		errorSum1 += (errorBuffer[i] - bufferMean) * (i - 9.5);
	}

	PID->Target_Value = Target_Value;
	PID->Measured_Value = Measured_Value;
	PID->Prev_Prev_Error = PID->Prev_Error;
	PID->Prev_Error = PID->Error;
	PID->Error = PID->Target_Value - PID->Measured_Value;

	PID->P_Out = PID->Kp * (PID->Error - PID->Prev_Error);
	PID->I_Out = PID->Ki * PID->Error;
	// PID->D_Out = PID->Kd * (PID->Error - 2.0f * PID->Prev_Error + PID->Prev_Prev_Error);
	PID->D_Out = PID->Kd * (errorSum1 / 4.5f);

	PID->I_Out = VAL_LIMIT(PID->I_Out, PID->I_Out_Max, -PID->I_Out_Max);
	PID->Output = (PID->P_Out + PID->I_Out + PID->D_Out);
	PID->Output = VAL_LIMIT(PID->Output, PID->Output_Max, -PID->I_Out_Max);

	return PID->Output;
}

void Clear_PID_Data(PID_t *PID)
{
	PID->Target_Value = 0;
	PID->Measured_Value = 0;
	PID->Prev_Prev_Error = 0;
	PID->Prev_Error = 0;
	PID->Error = 0;
	PID->P_Out = 0;
	PID->I_Out = 0;
	PID->D_Out = 0;
	PID->Output = 0;
}
