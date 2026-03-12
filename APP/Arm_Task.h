#ifndef __ARM_TASK_H__
#define __ARM_TASK_H__

#include "dm4310_drv.h"

#define g 9.8066f  // 哈尔滨重力加速度
#define GEAR_RATIO  4.0f  // 电机齿轮比
// --- 关节限位定义 ---
#define J1_MIN_RAD -1.570796F
#define J1_MAX_RAD 1.570796f
#define J2_MIN_RAD 0.0F
#define J2_MAX_RAD 3.1415926f
#define J3_MIN_RAD -1.570796F
#define J3_MAX_RAD 0.1f
#define J4_MIN_RAD -3.1415926F
#define J4_MAX_RAD 3.1415926f
#define J5_MIN_RAD -1.570796F
#define J5_MAX_RAD 1.570796f

// --- 机械臂尺寸参数 ---
#define l1 0.06f    //!连杆1长度
#define l2 0.3f     //!连杆2长度
#define l3 0.07f    //!连杆3长度
#define l4 0.148f   //!连杆4长度
#define l5 0.038f   //!连杆5长度
#define l6 0.00f    //!连杆6长度
#define l01 0.15f   //!连杆2质心到J2输出轴的直线距离
#define l03 0.160f  //!小臂质心到J3输出轴的直线距离

// --- 质量参数 ---
#define m1 0.362f   //!J1质量
#define m2 0.362f   //!J2质量
#define m3 0.362f   //!J3质量
#define m4 0.325f   //!J4质量
#define m5 0.325f   //!J5质量
#define m6 0.325f   //!J6质量
#define m01 0.1f    //!连杆2质量
#define m02 0.85f   //!小臂补偿总质量  

// --- 归零参数 ---
#define HOME_KP 5.0f
#define HOME_KD 0.5f

typedef enum
{
    ARM_STATE_DISABLED = 0,     // 失能/放松
    ARM_STATE_HOMING,           // 上电阶梯归零 (缓启动)
    ARM_STATE_GRAVITY_COMP,     // 重力补偿 (示教模式)
    ARM_STATE_REPLAY,           // 从臂跟随 (自定义控制器模式)
    ARM_STATE_KEYMOUSE,         // 键鼠控制模式
    ARM_STATE_ERROR             // 错误保护
} Arm_State_e;

//==============================================补充定义================================================================================//
#define MAX_SERVO_NUM 6
#define HOMING_STEP 6
typedef struct {
    uint8_t id;               // 关节电机ID（如CAN ID或索引）
    int32_t pos;      // 当前位置（单位：0.1度 或 脉冲数）
    uint8_t tor;
    // 可根据实际需求添加更多字段，如加速度、力矩限制、温度等
} Para;
typedef struct {
    uint8_t id;               // 关节电机ID（如CAN ID或索引）
    int32_t current_pos;      // 当前位置（单位：0.1度 或 脉冲数）
    int32_t target_pos;       // 目标位置
    int16_t current_speed;    // 当前速度
    int16_t max_speed;        // 最大允许速度
    uint8_t enabled;          // 使能标志：1-使能，0-失能
    uint8_t homing_offset;    // 归零偏移量（用于校准）
    uint8_t error_code;       // 错误代码（0表示无错误）
	uint8_t mode;
	Para para;
    // 可根据实际需求添加更多字段，如加速度、力矩限制、温度等
} Joint_t;
//==============================================补充定义================================================================================//

typedef struct
{
    Joint_t joints[MAX_SERVO_NUM]; // 关节电机对象 (需在电机驱动中定义)
    Arm_State_e current_state;     // 当前状态
    uint8_t start_flag;            // 启动标志
    uint8_t homing_done;           // 归零完成标志
} Arm_t;

// 声明外部变量
extern Arm_t my_arm;

// --- 函数声明 ---
void Arm_Task(void const * argument);
void Arm_Init(void);

// void Arm_GoHome_Slowly(void); // 【已删除】旧的阻塞式归零函数已废弃

#endif
