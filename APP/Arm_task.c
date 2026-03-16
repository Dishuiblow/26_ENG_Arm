#include "Arm_task.h"
#include "fdcan.h"
#include "cmsis_os.h"      
#include "dm4310_drv.h"
#include "math.h" 
#include "stdlib.h"        
#include "dynamic.h"       
#include "referee.h"
#include "dt7_remote_read.h"

//修正1：宏定义移到文件开头（规范语序）
#define CUSTOM_ANGLE_SCALE 0.002f    
#define J1_RAD_APP 1.5f
#define J2_RAD_APP 1.5f
#define J3_RAD_APP 1.5f
#define J4_RAD_APP 1.5f
#define J5_RAD_APP 1.5f
#define J6_RAD_APP 1.5f
//// 仅调整语法结构，不改变变量初始化方式
const RC_ctrl_t *rc_data = NULL;
//const RC_ctrl_t *rc_data = get_dt7_remote_control_point();

static uint8_t last_shift_state=0;
Arm_t my_arm;

float usb_target_pos[6] = {0}; 
float arm_target_pos[6] = {0}; 
static float current_filtered_target[6] = {0};
//// ? 修正2：补全结尾分号
//uint8_t current_shift_state = (rc_data->key.v & KEY_PRESSED_OFFSET_SHIFT);
uint8_t current_shift_state = 0;

int MAX_LIMIT[6] = {0};
int MIN_LIMIT[6] = {0};

fp32 arm_target_angle = 0.0f;

uint8_t arm_toggle_state = 0;


void Arm_Init(void){
    for(int i=0; i<6; i++) {
        joint_motor_init((Joint_Motor_t *)&my_arm.joints[i], i+1, MIT_MODE); 
        enable_motor_mode(&hfdcan1, my_arm.joints[i].para.id, my_arm.joints[i].mode);
	for(int i=0;i<6;i++)		
			{my_arm.joints[i].target_pos=0;}
			
			
			 rc_data=get_dt7_remote_control_point();
        osDelay(10);
    }
}

void ARM_Set_Control()
{
	if(rc_data->key.v&KEY_PRESSED_OFFSET_Q)
	{my_arm.joints[0].target_pos+=J1_RAD_APP;}
else if(rc_data->key.v&KEY_PRESSED_OFFSET_A)	
{
	my_arm.joints[0].target_pos-=J1_RAD_APP;
}

if(rc_data->key.v&KEY_PRESSED_OFFSET_W)
	{my_arm.joints[1].target_pos+=J2_RAD_APP;}
else if(rc_data->key.v&KEY_PRESSED_OFFSET_S)	
{
	my_arm.joints[1].target_pos-=J2_RAD_APP;
}

if(rc_data->key.v&KEY_PRESSED_OFFSET_E)
	{my_arm.joints[2].target_pos+=J3_RAD_APP;}
else if(rc_data->key.v&KEY_PRESSED_OFFSET_D)	
{
	my_arm.joints[2].target_pos-=J3_RAD_APP;
}

if(rc_data->key.v&KEY_PRESSED_OFFSET_R)
	{my_arm.joints[3].target_pos+=J4_RAD_APP;}
else if(rc_data->key.v&KEY_PRESSED_OFFSET_F)	
{
	my_arm.joints[3].target_pos-=J4_RAD_APP;
}

if(rc_data->key.v&KEY_PRESSED_OFFSET_SHIFT)
	{my_arm.joints[4].target_pos+=J5_RAD_APP;}
else if(rc_data->key.v&KEY_PRESSED_OFFSET_CTRL)	
{
	my_arm.joints[4].target_pos-=J5_RAD_APP;
}
if(rc_data->key.v&KEY_PRESSED_OFFSET_Z)
	{my_arm.joints[5].target_pos+=J6_RAD_APP;}
else if(rc_data->key.v&KEY_PRESSED_OFFSET_X)	
{
	my_arm.joints[5].target_pos-=J6_RAD_APP;
}
//加一个限幅函数
	
}


int i_test = 0;

extern  int i_test1;
void ARM_Task(void const * argument){
    osDelay(1000);
    Arm_Init();
	  ARM_Set_Control();
//在这里添加发送函数
	//mit_ctrl_4310（0，0，0，0，my_arm.joints[0].target_pos）
	//mit_ctrl_4310（）
	
	
	
//    //rc_data = get_dt7_remote_control_point();  //增加
//    
//    static uint16_t last_custom_angle_5 = 0; 
//    static uint8_t is_first_custom_data = 1;

//    for (;;){
//        i_test1 ++;
//        i_test++;
//          osDelay(10);
////        current_shift_state = (rc_data->key.v & KEY_PRESSED_OFFSET_SHIFT);
////        // 第一部分：原if切换逻辑
////        if (current_shift_state == 1 && last_shift_state == 0) {
////            arm_toggle_state = !arm_toggle_state;
////            if (arm_toggle_state) {
////                // 发送角度1 给总线舵机
////            } 
////            else {
////                // 发送角度2 给总线舵机
////            }
////        }
////        last_shift_state = current_shift_state;
           ;

////        // 第二部分：原按键控制逻辑
////        if (rc_data->key.v & KEY_PRESSED_OFFSET_W) {
////            arm_target_angle += 1.5f;
////        } else if (rc_data->key.v & KEY_PRESSED_OFFSET_S) {
////            arm_target_angle -= 1.5f;
////        }

////        // 第三部分：原for循环逻辑
////        for(int i=0; i<6; i++) {
////            arm_target_pos[i] = my_arm.joints[i].para.pos / GEAR_RATIO;
////            current_filtered_target[i] = arm_target_pos[i];
////        }

////        // 第四部分：原while(1)内的总线舵机、电机控制逻辑
////        uint16_t current_custom_angle_5 = REF.custom_robot_data.data[5*2] | 
////                                         (REF.custom_robot_data.data[5*2 + 1] << 8);
////        
////        if (is_first_custom_data && current_custom_angle_5 != 0) {
////            last_custom_angle_5 = current_custom_angle_5;
////            is_first_custom_data = 0;
////        }

////        if (!is_first_custom_data){
////            if (current_custom_angle_5 >= 500 && current_custom_angle_5 <= 2500 &&
////                     last_custom_angle_5 >= 500 && last_custom_angle_5 <= 2500){ 
////                int16_t delta = (int16_t)current_custom_angle_5 - (int16_t)last_custom_angle_5;
////                if (abs(delta) > 1)  arm_target_pos[5] += (float)delta * CUSTOM_ANGLE_SCALE;
////                
////            }
////            last_custom_angle_5 = current_custom_angle_5;
////        }

////        if (MAX_LIMIT[5] != MIN_LIMIT[5]) { 
////            if(arm_target_pos[5] > MAX_LIMIT[5]) arm_target_pos[5] = MAX_LIMIT[5];
////            if(arm_target_pos[5] < MIN_LIMIT[5]) arm_target_pos[5] = MIN_LIMIT[5];
////        }
////        
////        current_filtered_target[5] = current_filtered_target[5] * 0.85f + arm_target_pos[5] * 0.15f;

////        for (int i=0; i<6; i++){
////            if (i == 5){
////                float motor_pos_ref = current_filtered_target[5] * GEAR_RATIO;
////                float kp = 6.0f; 
////                float kd = 1.2f; 
////                mit_ctrl_4310(&hfdcan1, 6, motor_pos_ref, 0, kp, kd, 0.0f);
////            }
////            else  mit_ctrl_4310(&hfdcan1, i+1, 0, 0, 0, 0, 0); 
////            
////        }
//                
//        osDelay(2);
    }
//}

