/**
 * @file Shooting_Control.h
 * @author Leo Liu
 * @brief header file for shooting control
 * @version 1.0
 * @date 2022-07-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __SHOOTING_CONTROL_H
#define __SHOOTING_CONTROL_H

#include "M3508_Motor.h"
#include "M2006_Motor.h"
#include "State_Machine.h"
#include "DR16_Remote.h"
#include "Ramp_Calc.h"
#include "Referee_System.h"

#define FRIC_SPEED_SLOW 8200//7000
#define FRIC_SPEED_16 8200 //5300		  // Tested value for 16m/s
#define FRIC_SPEED_28 8200 //6000		  // Tested value for 28m/s
#define LEFT_TRIGGER_DIRECTION 1  // Trigger motor direction
#define RIGHT_TRIGGER_DIRECTION 1 // Trigger motor direction
#define FRIC_LEFT_DIRECTION -1	  // Left friction wheel motor direction
#define FRIC_RIGHT_DIRECTION 1	  // Right friction wheel motor direction

#define Shooting_Func_GroundInit  \
	{                             \
		&Shooting_Init,           \
			&Trigger_Get_Data,    \
			&Shooting_Processing, \
			&Shooting_Disabled,   \
	}

typedef struct
{
	struct
	{
		uint8_t Left_Single_Fire_Flag;	 // Left Signal that need to fire once
		uint8_t Left_Single_Fired_Flag;	 // Left Signal that it has fired once already
		uint8_t Right_Single_Fire_Flag;	 // Right Signal that need to fire once
		uint8_t Right_Single_Fired_Flag; // Right Signal that it has fired once already
		uint8_t Left_Burst_Flag;		 // Left Signal continuous shooting
		uint8_t Right_Burst_Flag;		 // Right Signal continuous shooting
		uint8_t Auto_Aiming;			 // Auto Aiming Signal
	} Type;

	struct
	{
		float Left_Target_Angle;
		float Left_Target_Speed;
		float Right_Target_Angle;
		float Right_Target_Speed;
	} Trigger;

	struct
	{
		uint8_t Turned_On;
	} Fric_Wheel;
	int16_t Left_Click_Counter;
	int16_t Right_Click_Counter;
	uint8_t Fric_Wheel_Ready_Flag;
} Shooting_t;

typedef struct
{
	void (*Shooting_Init)(void);
	void (*Trigger_Get_Data)(Shooting_t *Shooting);
	void (*Shooting_Processing)(Shooting_t *Shooting);
	void (*Shooting_Disabled)(void);
} Shooting_Func_t;

extern Shooting_t Shooting;
extern Shooting_Func_t Shooting_Func;

#endif
