#ifndef ODOMETRY_H
#define ODOMETRY_H

#define RBG_Init_X 0.0f
#define RBG_Init_Y 0.0f
#define RBG_Init_ORIENTATION 0.0f
#define WHEEL_DIAMETER 152.5f // [mm]
#define WHEEL_RADIUS WHEEL_DIAMETER / 2.0f
#define CHASSIS_HALF_WIDTH 223.8f  //[mm]
#define CHASSIS_HALF_LENGTH 179.6f //[mm]

#define PI 3.14159f
#define REDUCTION_RATIO 3591 / 187
#define Angle_To_Disp (float)(((2.0f * PI) / (M3508_MECH_ANGLE_MAX * REDUCTION_RATIO)) * WHEEL_RADIUS)

#define Odometry_Func_GroundInit \
    {                            \
        &Update_Robot_Pose,      \
            &Init_Robot_Pose     \
    }

typedef struct
{
    float Front_Right;
    float Front_Left;
    float Rear_Left;
    float Rear_Right;
} Motor_Data;

typedef struct
{
    float Position_X;
    float Position_Y;
    float Orientation;
    float Orientation_Degree;
		float Prev_Position_X;
		float Prev_Position_Y;
		float Prev_Orientation_Degree;
		float Velocity_X;
		float Velocity_Y;
		float Velocity_Orientation;
		float Prev_Time;
    float Current_Time;
		float Period;
}	Pose;

typedef struct
{
    void (*Update_Robot_Pose)(void);
    void (*Init_Robot_Pose)(void);
} Odometry_Func_t;

void Odometry(void const *argument);

extern Odometry_Func_t Odometry_Func;
extern Pose RBG_Pose;
extern Motor_Data Last;
extern Motor_Data Current;
extern Motor_Data Increment;
extern unsigned short counter;
#endif
