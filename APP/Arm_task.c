#include "Arm_Task.h"
#include "fdcan.h"
#include "cmsis_os.h"      
#include "dm4310_drv.h"
#include "math.h" 
#include "stdlib.h"        // 【修复】引入 stdlib.h，解决 abs() 隐式声明警告
#include "dynamic.h"       
#include "referee.h"

// =========================================================
// 全局变量区
// =========================================================
Arm_t my_arm;

// 【修复】定义一个“傀儡”变量，专门用来应付 usb_protocol.c 的编译报错。
// 我们在下方的实际控制中完全不用它，从而彻底隔离 USB 的干扰！
float usb_target_pos[6] = {0}; 

// 这是我们图传链路真正使用的、纯净的控制目标变量
float arm_target_pos[6] = {0}; 
static float current_filtered_target[6] = {0};

// 自定义控制器增量缩放系数（灵敏度）
#define CUSTOM_ANGLE_SCALE 0.002f    

// 关节限位保护（测试时暂设为全0，代表不限位）
int MAX_LIMIT[6] = {0};
int MIN_LIMIT[6] = {0};

// ---------------------------------------------------------
// 机械臂初始化
// ---------------------------------------------------------
void Arm_Init(void)
{
    // 初始化6个 DM4310 电机并使能进入 MIT 控制模式
    for(int i=0; i<6; i++) {
        // 【修复】加上 (Joint_Motor_t *) 强制类型转换，解决类型不匹配警告
        joint_motor_init((Joint_Motor_t *)&my_arm.joints[i], i+1, MIT_MODE); 
        enable_motor_mode(&hfdcan1, my_arm.joints[i].para.id, my_arm.joints[i].mode);
        osDelay(10);
    }
}

// =========================================================
// 【零 USB 干扰版】仅 006 号电机接收图传响应
// =========================================================
void Arm_Task(void const * argument)
{
    osDelay(1000); // 上电等待底盘及驱动就绪
    Arm_Init();    // 上电使能所有电机
    
    // 专为 006 号电机（数组下标为 5）设置的状态变量
    static uint16_t last_custom_angle_5 = 0; 
    static uint8_t is_first_custom_data = 1;

    // 初始化时获取当前位置，防飞车
    for(int i=0; i<6; i++) {
        arm_target_pos[i] = my_arm.joints[i].para.pos / GEAR_RATIO;
        current_filtered_target[i] = arm_target_pos[i];
    }

    while(1)
    {
        // -------------------------------------------------------------
        // 解析 006 号电机的图传数据 (数组下标 5，位于 custom_robot_data.data 第10和11字节)
        // -------------------------------------------------------------
        uint16_t current_custom_angle_5 = REF.custom_robot_data.data[5*2] | 
                                         (REF.custom_robot_data.data[5*2 + 1] << 8);
        
        // 获取有效初值（防上电跳变）
        if (is_first_custom_data && current_custom_angle_5 != 0) {
            last_custom_angle_5 = current_custom_angle_5;
            is_first_custom_data = 0;
        }

        // -------------------------------------------------------------
        // 计算 006 号增量逻辑
        // -------------------------------------------------------------
        if (!is_first_custom_data) 
        {
            // 校验总线舵机数据是否在合法范围内 0500~2500
            if (current_custom_angle_5 >= 500 && current_custom_angle_5 <= 2500 &&
                last_custom_angle_5 >= 500 && last_custom_angle_5 <= 2500) 
            {
                int16_t delta = (int16_t)current_custom_angle_5 - (int16_t)last_custom_angle_5;
                if (abs(delta) > 1) { // 死区过滤
                    arm_target_pos[5] += (float)delta * CUSTOM_ANGLE_SCALE;
                }
            }
            last_custom_angle_5 = current_custom_angle_5;
        }

        // 保护与滤波（仅针对 006 号）
        if(MAX_LIMIT[5] != MIN_LIMIT[5]) { 
            if(arm_target_pos[5] > MAX_LIMIT[5]) arm_target_pos[5] = MAX_LIMIT[5];
            if(arm_target_pos[5] < MIN_LIMIT[5]) arm_target_pos[5] = MIN_LIMIT[5];
        }
        
        current_filtered_target[5] = current_filtered_target[5] * 0.85f + arm_target_pos[5] * 0.15f;

        // -------------------------------------------------------------
        // 发送 CAN 控制指令
        // -------------------------------------------------------------
        for(int i=0; i<6; i++) 
        {
            if (i == 5) // 如果是第 6 号电机 (CAN ID = 6)
            {
                float motor_pos_ref = current_filtered_target[5] * GEAR_RATIO;
                float kp = 6.0f; 
                float kd = 1.2f; 
                mit_ctrl_4310(&hfdcan1, 6, motor_pos_ref, 0, kp, kd, 0.0f);
            }
            else // 其余 0~4 号电机
            {
                // 发送 0 力矩瘫软，完全不干预
                mit_ctrl_4310(&hfdcan1, i+1, 0, 0, 0, 0, 0); 
            }
        }
                
        osDelay(2);
    }
}
