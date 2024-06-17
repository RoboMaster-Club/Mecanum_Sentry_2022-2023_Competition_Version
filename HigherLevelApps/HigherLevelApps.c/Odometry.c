#include "Odometry.h"
#include "M3508_Motor.h"
#include "main.h"
#include "GM6020_Motor.h"
#include "Board_A_IMU.h"
#include <math.h>

Pose RBG_Pose;
Pose Pose_Incr;
Motor_Data Last;
Motor_Data Current;
Motor_Data Increment;
uint16_t counter = 0;

void Init_Robot_Pose(void);
void Update_Robot_Pose(void);
void Odometry(void const *argument);

Odometry_Func_t Odometry_Func = Odometry_Func_GroundInit;
#undef Odometry_Func_GroundInit

void Init_Robot_Pose()
{
  RBG_Pose.Position_X = RBG_Init_X;
  RBG_Pose.Position_Y = RBG_Init_Y;
  RBG_Pose.Orientation = RBG_Init_ORIENTATION;

  Last.Front_Right = -M3508_Chassis[0].Total_Angle * Angle_To_Disp;
  Last.Front_Left = M3508_Chassis[1].Total_Angle * Angle_To_Disp;
  Last.Rear_Left = M3508_Chassis[2].Total_Angle * Angle_To_Disp;
  Last.Rear_Right = -M3508_Chassis[3].Total_Angle * Angle_To_Disp;
}

void Update_Robot_Pose()
{
  if (Last.Front_Left != 0 || counter > 5000)
  {
    Current.Front_Right = -M3508_Chassis[0].Total_Angle * Angle_To_Disp;
    Current.Front_Left = M3508_Chassis[1].Total_Angle * Angle_To_Disp;
    Current.Rear_Left = M3508_Chassis[2].Total_Angle * Angle_To_Disp;
    Current.Rear_Right = -M3508_Chassis[3].Total_Angle * Angle_To_Disp;

    Increment.Front_Right = Current.Front_Right - Last.Front_Right;
    Increment.Front_Left = Current.Front_Left - Last.Front_Left;
    Increment.Rear_Left = Current.Rear_Left - Last.Rear_Left;
    Increment.Rear_Right = Current.Rear_Right - Last.Rear_Right;

    Pose_Incr.Position_X = (Increment.Front_Left + Increment.Front_Right + Increment.Rear_Right + Increment.Rear_Left) / 4;
    Pose_Incr.Position_Y = (-Increment.Front_Left + Increment.Front_Right - Increment.Rear_Right + Increment.Rear_Left) / 4;
		// offset 2660
		// positive:ccw
		// imu yaw positive:ccw
    RBG_Pose.Orientation = RBG_Init_ORIENTATION +
                           ((-Current.Front_Left + Current.Front_Right + Current.Rear_Right - Current.Rear_Left) / (4 * (CHASSIS_HALF_WIDTH + CHASSIS_HALF_LENGTH))) * 0.3f + 0.7f * RBG_Pose.Orientation;
		float chassis_orientation = Board_A_IMU.Export_Data.Yaw / 180.0f * PI - (GM6020_Yaw.Actual_Angle - 2660.0f) / 8192.0f * PI * 2;
		
		RBG_Pose.Prev_Time = RBG_Pose.Current_Time;
		RBG_Pose.Current_Time = HAL_GetTick();
		RBG_Pose.Period = (RBG_Pose.Current_Time - RBG_Pose.Prev_Time)/1000.0f;
		
		RBG_Pose.Prev_Position_X	= RBG_Pose.Position_X;
		RBG_Pose.Prev_Position_Y	= RBG_Pose.Position_Y;
		RBG_Pose.Prev_Orientation_Degree = RBG_Pose.Orientation_Degree;
    RBG_Pose.Orientation_Degree = RBG_Pose.Orientation / PI * 180;
    RBG_Pose.Position_X += Pose_Incr.Position_X * cos(chassis_orientation) - Pose_Incr.Position_Y * sin(chassis_orientation);
    RBG_Pose.Position_Y += Pose_Incr.Position_X * sin(chassis_orientation) + Pose_Incr.Position_Y * cos(chassis_orientation);
		
		RBG_Pose.Velocity_X = (RBG_Pose.Position_X - RBG_Pose.Prev_Position_X)/RBG_Pose.Period;
		RBG_Pose.Velocity_Y = (RBG_Pose.Position_Y - RBG_Pose.Prev_Position_Y)/RBG_Pose.Period;
		RBG_Pose.Velocity_Orientation = (RBG_Pose.Orientation_Degree - RBG_Pose.Prev_Orientation_Degree)/RBG_Pose.Period;

    Last.Front_Right = Current.Front_Right;
    Last.Front_Left = Current.Front_Left;
    Last.Rear_Left = Current.Rear_Left;
    Last.Rear_Right = Current.Rear_Right;
  }
  else
  {
    Init_Robot_Pose();
    counter += 1;
  }
}

void Odometry(void const *argument)
{
  /* USER CODE BEGIN Robot_Control */
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(5);
  /* Infinite loop */
  for (;;)
  {
    Update_Robot_Pose();
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END Robot_Control */
}
