#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__

#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200
//DM4310
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -3.5f
#define T_MAX 3.5f
//DM4340
#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -11.0f
#define T_MAX2 11.0f
typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;

//关节电机结构体，保留
typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;


//通用的电机反馈函数
extern void dm_motor_fdback(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
/*
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);//!弃用
extern void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);//!弃用
*/
//电机使能和失能函数
extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
//大臂关节电机全部使用MIT控制协议
extern void mit_ctrl_4310(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);//!保留
extern void mit_ctrl_4340(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);//!保留

extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);

	
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);


extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

#endif /* __DM4310_DRV_H__ */



