/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             检测错误任务， 通过接收数据时间来判断.提供 检测钩子函数,错误存在函数.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    add a sensor 
    1. in detect_task.h, add the sensor name at the end of errorList,like
    enum errorList
    {
        ...
        XXX_TOE,    //new sensor
        ERROR_LIST_LENGHT,
    };
    2.in detect_init function, add the offlineTime, onlinetime, priority params,like
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3. if XXX_TOE has data_is_error_fun ,solve_lost_fun,solve_data_error_fun function, 
        please assign to function pointer.
    4. when XXX_TOE sensor data come, add the function detect_hook(XXX_TOE) function.
    如果要添加一个新设备
    1.第一步在detect_task.h，添加设备名字在errorList的最后，像
    enum errorList
    {
        ...
        XXX_TOE,    //新设备
        ERROR_LIST_LENGHT,
    };
    2.在detect_init函数,添加offlineTime, onlinetime, priority参数
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.如果有data_is_error_fun ,solve_lost_fun,solve_data_error_fun函数，赋值到函数指针
    4.在XXX_TOE设备数据来的时候, 添加函数detect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"


#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//错误码以及对应设备顺序
enum errorList
{
		 DBUS_TOE 						= 0,
		 CHASSIS_MOTOR1_TOE	 	= 1,
		 CHASSIS_MOTOR2_TOE 	= 2,
		 CHASSIS_MOTOR3_TOE 	= 3,
		 CHASSIS_MOTOR4_TOE 	= 4,
		 GIMBAL_MOTOR_YAW_TOE = 5,
		 BOARD_GYRO_TOE 			= 6,
		 BOARD_ACCEL_TOE 			= 7,
		 BOARD_MAG_TOE 				= 8,
		 REFEREE_TOE 					= 9,
		 RM_IMU_TOE 					= 10,
		 PC_TOE	 = 11,
		 ERROR_LIST_LENGHT 		= 12,							// 记录设备总数，必须放在枚举最后
};

typedef __packed struct
{
    uint32_t new_time;							// 当前检测时间
    uint32_t last_time;							// 上次检测时间
    uint32_t lost_time;							// 设备离线时刻
    uint32_t work_time;							// 设备上线持续时间
    uint16_t set_offline_time : 12; // 判定设备离线时间
    uint16_t set_online_time : 12;	// 上线到稳定工作时间，该时间>=offline_time
    uint8_t enable : 1;							// 使能检测该设备
    uint8_t priority : 4;						// 设备检测优先级
    uint8_t error_exist : 1;				// 设备异常，0：正常；1：异常。未知是由于数据紊乱还是设备离线引起
    uint8_t is_lost : 1;						// 设备离线，0：正常；1：离线
    uint8_t data_is_error : 1;			// 判断数据错误

    fp32 frequency;									// 设备检测频率 ？
    bool_t (*data_is_error_fun)(void);  // 判断数据是否出错 参考init中遥控器数据出错检测写法
    void (*solve_lost_fun)(void);
    void (*solve_data_error_fun)(void);
} error_t;



extern void detect_task(void const *pvParameters);


extern bool_t toe_is_error(uint8_t err);


extern void detect_hook(uint8_t toe);


extern const error_t *get_error_list_point(void);


extern const error_t *get_REF_error_list_point(void);
#endif
