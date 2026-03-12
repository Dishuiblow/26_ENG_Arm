#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

extern UART_HandleTypeDef huart7;

REF_t REF;

/*初始化裁判系统数据结构*/
void init_referee_struct_data(void)
{
			memset(&REF, 0, sizeof(REF_t));
}

/*通过串口读取裁判系统数据*/
void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&REF.referee_receive_header, frame, sizeof(frame_header_struct_t));// 储存帧头数据

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case ID_game_state://0x0001
        {
            memcpy(&REF.game_status, frame + index, sizeof(ext_game_status_t));
        }
        break;
        case ID_game_result://0x0002
        {
            memcpy(&REF.game_result, frame + index, sizeof(ext_game_result_t));
        }
        break;
        case ID_game_robot_survivors://0x0003
        {
            memcpy(&REF.game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;
				
				/* 0x0004：飞镖发射状态 */
				/* 0x0005:	ICRA增益状态*/

        case ID_event_data://0x0101
          {
              memcpy(&REF.field_event, frame + index, sizeof(ext_event_data_t));
          }
        break;
				
        case ID_supply_warm://0x0104
          {
              memcpy(&REF.referee_warning, frame + index, sizeof(ext_referee_warning_t));
          }
        break;
				
				case ID_missile_remain_time://0x0105
        {
            memcpy(&REF.ext_dart_remaining_time, frame + index, sizeof(ext_dart_remaining_time_t));
        }
        break;

        case ID_game_robot_state://0x0201
        {
            memcpy(&REF.robot_status, frame + index, sizeof(ext_game_robot_status_t));
						Determine_ID();
        }
        break;
        case ID_power_heat_data://0x0202
        {
            memcpy(&REF.power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ID_game_robot_pos://0x0203
        {
            memcpy(&REF.game_robot_pos, frame + index, sizeof(ext_game_robot_pos_t));
        }
        break;
        case ID_buff_musk://0x0204
        {
            memcpy(&REF.buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        case ID_aerial_robot_energy://0x0205
        {

        }
        break;
        case ID_robot_hurt://0x0206
        {
            memcpy(&REF.robot_hurt, frame + index, sizeof(ext_robot_hurt_t));
        }
        break;
        case ID_shoot_data://0x0207
        {
            memcpy(&REF.shoot_data, frame + index, sizeof(ext_shoot_data_t));
        }
        break;
        case ID_bullet_remaining://0x0208
        {
            memcpy(&REF.bullet_remaining, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
				case ID_rfid_status://0x0209
        {
            memcpy(&REF.rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
				case ID_dart_client_directive://0x020A
        {
            memcpy(&REF.dart_client_cmd, frame + index, sizeof(ext_dart_client_cmd_t));
        }
        break;
		case ID_robot_position:// 0x020B
		{
				memcpy(&REF.ground_robot_positon, frame + index, sizeof(ground_robot_position_t));
		}
		break;
		case ID_radar_mark:// 0x020C
		  {
			  memcpy(&REF.radar_mark_data, frame + index, sizeof(radar_mark_data_t));
		  }
				break;
		  
		  case ID_sentry_info:// 0x20D
		  {
			  memcpy(&REF.sentry_info, frame + index, sizeof(sentry_info_t));
		  }
				break;
		  case ID_radar_info:// 0x20E
		  {
			  memcpy(&REF.radar_info, frame + index, sizeof(radar_info_t));
		  }
				break;
          
	   case ID_robot_interactive_data://0x0301
		  {
			  memcpy(&REF.ext_student_interactive_data, frame + index, sizeof(ext_student_interactive_data_t));
		  }
		break;
		  
		  case ID_controller_interactive_header_data://0x0302//自定义控制器交互数据接口，通过——客户端触发——发送 30Hz
		  {
			  memcpy(&REF.custom_robot_data, frame + index, sizeof(custom_robot_data_t));
		  }
		break;
          
		case ID_map_interactive_header_data://0x0303
          {
              memcpy(&REF.robot_command, frame + index, sizeof(ext_robot_command_t));
          }
			break;
 
        default:
        {
            break;
        }
    }
}

/*************** 通过裁判系统发送信息 ***************/

uint8_t CliendTxBuffer[200];      // 发送到各客户端的数据存储在这儿
uint8_t RobotTxBuffer[200];				// 发送到各机器人的数据存储在这儿

/* 向机器人发送普通数据，youxiao数据最大113字节 */

// 通信信息，每次发向一个机器人。再次发送向其他机器人时其内容会被覆盖更新
ext_student_interactive_t student_interactive;

void common_data_send(void)
{
	/* 帧头 */
	student_interactive.frame_header.SOF = HEADER_SOF;
	student_interactive.frame_header.data_length = sizeof(ext_student_interactive_header_data_t) +sizeof(sentry_cmd_t) ;
																																													// 数据段段头 + 内容数据段
	student_interactive.frame_header.seq = 0;																								// 包序号
	memcpy(RobotTxBuffer,&student_interactive.frame_header,sizeof(frame_header_struct_t)); 	// 首先向暂存数组中赋入帧头信息，以便于头校验
	Append_CRC8_Check_Sum(RobotTxBuffer, sizeof(frame_header_struct_t)); 										// 头CRC8校验，校验码放在数据末尾字节处
	
	/* 命令码 */
	student_interactive.cmd_id = ID_robot_interactive_data;// 0x0301

	/* 数据段头结构 */
	student_interactive.data_header.data_cmd_id = 0x0120; // 数据内容 ID，0x0200~0x02FF
	student_interactive.data_header.sender_ID   = REF.robot_status.robot_id;	// 发送者为本机(哨兵)
	student_interactive.data_header.receiver_ID = 0x8080; 			  	// 接收者为己方机器人

	/* 数据段 */
	//memcpy(student_interactive.command.sentry_cmd, data, 113);   // 用数组的大小除以元素的大小来获得元素个数
	memcpy(RobotTxBuffer + LEN_FRAME_HEAD, (uint8_t*)&student_interactive.cmd_id, LEN_CMD_ID + student_interactive.frame_header.data_length);// 加上命令码长度2
																																					  // 从帧头后（含CRC8）开始，向暂存数组中赋入命令码、数据段段头、数据段
	/* 帧尾 */
	Append_CRC16_Check_Sum(RobotTxBuffer, sizeof(student_interactive)); 			//尾CRC16校验，校验码放在数据末尾两个字节处，高八位在前
	
	/* 通过串口发送暂存数组RobotTxBuffer内的数据包 */
	uart_send_message(RobotTxBuffer, sizeof(ext_student_interactive_t));
	
}

/* 向客户端绘制字符数据,youxiao数据最多30个字节 */

// 字符包，每次发向一个客户端。再次发送不同内容或不同客户端，数据包覆盖更新
ext_character_data_t character_data;

void char_send(const uint16_t *team_client_id)
{
	/* 帧头 */
	character_data.frame_header.SOF = HEADER_SOF;
	character_data.frame_header.data_length = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_character_t);
																																											// 数据段段头 + 内容数据段
	character_data.frame_header.seq = 0;																								// 包序号
	memcpy(CliendTxBuffer,&character_data.frame_header,sizeof(frame_header_struct_t)); 	// 首先向暂存数组中赋入帧头信息，以便于头校验
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_struct_t)); 							// 校验码放在数据末尾字节处
	
	/* 命令码 */
	character_data.cmd_id = ID_robot_interactive_data;// 0x301

	/* 数据段头结构 */
	character_data.data_header.data_cmd_id = INTERACT_ID_draw_char_graphic; // 数据内容 ID
	character_data.data_header.sender_ID   = REF.robot_status.robot_id;			// 发送者为本机(哨兵)
	character_data.data_header.receiver_ID = *team_client_id; 			 			 	// 接收者为己方客户端

	/* 数据段 */
	// 留待封装
	memcpy(RobotTxBuffer + LEN_FRAME_HEAD, (uint8_t*)&character_data.cmd_id, LEN_CMD_ID + character_data.frame_header.data_length);
																																					// 从帧头后（含CRC8）开始，向暂存数组中赋入命令码、数据段段头、数据段
	/* 帧尾 */
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ext_character_data_t));    // 尾CRC16校验，校验码放在数据末尾两个字节处，高八位在前
	
	/* 通过串口发送暂存数组CliendTxBuffer内的数据包 */
	uart_send_message(CliendTxBuffer, sizeof(ext_character_data_t));
}

/* 数据段摘出来封装一下 */
// 30个字节的空行，每次调用时会使用空行覆盖前一次字符串
char empty_line[30] = {"                              "};
void data_char(ext_client_custom_character_t *char_data, // 最终发送的数据段内容
										// 通用设置
										const char *graphic_name,	
										uint32_t operate_type,
										uint32_t graphic_type,
										uint32_t layer,
										uint32_t color,
										// 字符独有设置
										uint32_t size,
										uint32_t length,
										// 通用设置
										uint32_t width,
										uint32_t start_x,
										uint32_t start_y,
										// 最终展示的字符
										const char *character)
{
	// 重复定义是为了写的时候方便
	graphic_data_struct_t *graphic_setting = &char_data->grapic_data_struct;
	
	memcpy(graphic_setting->graphic_name, graphic_name, sizeof(graphic_name) / sizeof(char));  // 字符索引
	graphic_setting->operate_type = operate_type;                       // 图形操作
	graphic_setting->graphic_type = graphic_type;												// 图形类型
	graphic_setting->layer = layer;																			// 图层数
	graphic_setting->color = color;																			// 颜色
	graphic_setting->start_angle = size;																// 字符大小	
	graphic_setting->end_angle = length;                                // 字符长度
	graphic_setting->width = width;																			// 线条宽度
	graphic_setting->start_x = start_x;																	// 起点 x 坐标
	graphic_setting->start_y = start_y;																	// 起点 y 坐标
	// 以下赋值为空即可，不影响输出内容
	graphic_setting->radius = 0;																				// 空
	graphic_setting->end_x = 0; 																				// 空
	graphic_setting->end_y = 0; 																				// 空
	
  memcpy(char_data->data, empty_line, 19);														// 用空字符串覆盖原信息
	memcpy(char_data->data, character, length);													// 最终显示的字符串

}


/* 向客户端绘制象形数据 */
// 象形图，每次发向一个客户端。再次发送不同内容或不同客户端，数据包覆盖更新
ext_client_custom_graphic_seven_t graphic_data;

void graphic_send(const uint16_t *team_client_id)
{
	/* 帧头 */
	graphic_data.frame_header.SOF = HEADER_SOF;
	graphic_data.frame_header.data_length = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t);
																																											// 数据段段头 + 内容数据段
	graphic_data.frame_header.seq = 0;																								// 包序号
	memcpy(CliendTxBuffer,&graphic_data.frame_header,sizeof(frame_header_struct_t)); 	// 首先向暂存数组中赋入帧头信息，以便于头校验
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_struct_t)); 							// 校验码放在数据末尾字节处
	
	/* 命令码 */
	graphic_data.cmd_id = ID_robot_interactive_data;// 0x301

	/* 数据段头结构 */
	graphic_data.data_header.data_cmd_id = INTERACT_ID_draw_seven_graphic; // 数据内容 ID
	graphic_data.data_header.sender_ID   = REF.robot_status.robot_id;			// 发送者为本机(哨兵)
	graphic_data.data_header.receiver_ID = *team_client_id; 			 			 	// 接收者为己方客户端

	/* 数据段 */
	// 留待封装
	memcpy(RobotTxBuffer + LEN_FRAME_HEAD, (uint8_t*)&graphic_data.cmd_id, LEN_CMD_ID + graphic_data.frame_header.data_length);
																																					// 从帧头后（含CRC8）开始，向暂存数组中赋入命令码、数据段段头、数据段
	/* 帧尾 */
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ext_client_custom_graphic_seven_t));    // 尾CRC16校验，校验码放在数据末尾两个字节处，高八位在前
	
	/* 通过串口发送暂存数组CliendTxBuffer内的数据包 */
	uart_send_message(CliendTxBuffer, sizeof(ext_client_custom_graphic_seven_t));
}

/* 数据段摘出来封装一下 */
void data_graphic(graphic_data_struct_t *graphic_data, // 数据段内容
										// 通用设置
										const char *graphic_name,
										uint32_t operate_type,
										uint32_t graphic_type,
										uint32_t layer,
										uint32_t color,
										// 除圆弧外为空，赋直线、矩形、正圆、椭圆时为0即可
										uint32_t start_angle,
										uint32_t end_angle,
										// 通用设置
										uint32_t width,
										// 弧状都为圆心
										uint32_t start_x,
										uint32_t start_y,
										// 正圆为半径，其余为空
										uint32_t radius,
										// 正圆为空
										uint32_t end_x,
										uint32_t end_y)
{
	memcpy(graphic_data->graphic_name, graphic_name, sizeof(graphic_name) / sizeof(char));  // 字符索引
	graphic_data->operate_type = operate_type;                       // 图形操作
	graphic_data->graphic_type = graphic_type;												// 图形类型
	graphic_data->layer = layer;																			// 图层数
	graphic_data->color = color;																			// 颜色
	graphic_data->start_angle = start_angle;													// 起始角度
	graphic_data->end_angle = end_angle;                             // 终止角度
	graphic_data->width = width;																			// 线条宽度
	graphic_data->start_x = start_x;																	// 起点 x 坐标
	graphic_data->start_y = start_y;																	// 起点 y 坐标
	graphic_data->radius = radius;																			// 字体大小或者半径
	graphic_data->end_x = end_x; 																				// 终点 x 坐标
	graphic_data->end_y = end_y; 																				// 终点 y 坐标
	
}

/* 向客户端绘制浮点数据 */
// 浮点型，每次发向一个客户端。再次发送不同内容或不同客户端，数据包覆盖更新
ext_client_custom_float_seven_t float_data;

void float_send(const uint16_t *team_client_id)
{
	/* 帧头 */
	float_data.frame_header.SOF = HEADER_SOF;
	float_data.frame_header.data_length = sizeof(ext_student_interactive_header_data_t) + sizeof(float_data_struct_t);
																																											// 数据段段头 + 内容数据段
	float_data.frame_header.seq = 0;																								// 包序号
	memcpy(CliendTxBuffer,&float_data.frame_header,sizeof(frame_header_struct_t)); 	// 首先向暂存数组中赋入帧头信息，以便于头校验
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_struct_t)); 							// 校验码放在数据末尾字节处
	
	/* 命令码 */
	float_data.cmd_id = ID_robot_interactive_data;// 0x301

	/* 数据段头结构 */
	float_data.data_header.data_cmd_id = INTERACT_ID_draw_seven_graphic; // 数据内容 ID,因为浮点型数据仍然属于象形，只是单独摘出来重新封装一下
	float_data.data_header.sender_ID   = REF.robot_status.robot_id;			// 发送者为本机(哨兵)
	float_data.data_header.receiver_ID = *team_client_id; 			 			 	// 接收者为己方客户端

	/* 数据段 */
	// 留待封装
	memcpy(RobotTxBuffer + LEN_FRAME_HEAD, (uint8_t*)&float_data.cmd_id, LEN_CMD_ID + float_data.frame_header.data_length);
																																					// 从帧头后（含CRC8）开始，向暂存数组中赋入命令码、数据段段头、数据段
	/* 帧尾 */
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ext_client_custom_float_seven_t));    // 尾CRC16校验，校验码放在数据末尾两个字节处，高八位在前
	
	/* 通过串口发送暂存数组CliendTxBuffer内的数据包 */
	uart_send_message(CliendTxBuffer, sizeof(ext_client_custom_float_seven_t));
}

/* 数据段摘出来封装一下 */
void data_float(float_data_struct_t *float_data, // 数据段内容位数
										// 通用设置
										const char *graphic_name,
										uint32_t operate_type,
										uint32_t graphic_type,
										uint32_t layer,
										uint32_t color,
										// 浮点型独有设置
										uint32_t size,
										uint32_t decimal_point,
										// 通用设置
										uint32_t width,
										uint32_t start_x,
										uint32_t start_y,
										// 最终要展现的数据
										fp32		 number)
{
	memcpy(float_data->graphic_name, graphic_name, sizeof(graphic_name) / sizeof(char));  // 字符索引
	float_data->operate_type = operate_type;                       	// 图形操作
	float_data->graphic_type = graphic_type;												// 图形类型
	float_data->layer = layer;																			// 图层数
	float_data->color = color;																			// 颜色
	float_data->start_angle = size;																	// 起始角度
	float_data->end_angle = decimal_point;                          // 终止角度
	float_data->width = width;																			// 线条宽度
	float_data->start_x = start_x;																	// 起点 x 坐标
	float_data->start_y = start_y;																	// 起点 y 坐标
	float_data->number = number;																		// 真实数据
	
}


/* 向客户端绘制整型数据 */
// 整型，每次发向一个客户端。再次发送不同内容或不同客户端，数据包覆盖更新
ext_client_custom_int_seven_t int_data; // 不要定义为带特定数字的，否则不需要太多数字时需要修改整个发送函数的结构体名称

void int_send(const uint16_t *team_client_id)
{
	/* 帧头 */
	int_data.frame_header.SOF = HEADER_SOF;
	int_data.frame_header.data_length = sizeof(ext_student_interactive_header_data_t) + sizeof(int_data_struct_t);
																																											// 数据段段头 + 内容数据段
	int_data.frame_header.seq = 0;																						// 包序号
	memcpy(CliendTxBuffer,&int_data.frame_header,sizeof(frame_header_struct_t)); 	// 首先向暂存数组中赋入帧头信息，以便于头校验
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_struct_t)); 							// 校验码放在数据末尾字节处
	
	/* 命令码 */
	int_data.cmd_id = ID_robot_interactive_data;// 0x301

	/* 数据段头结构 */
	int_data.data_header.data_cmd_id = INTERACT_ID_draw_seven_graphic; // 数据内容 ID,因为整型数据仍然属于象形，只是单独摘出来重新封装一下
	int_data.data_header.sender_ID   = REF.robot_status.robot_id;			// 发送者为本机(哨兵)
	int_data.data_header.receiver_ID = *team_client_id; 			 			 	// 接收者为己方客户端

	/* 数据段 */
	// 留待封装
	memcpy(RobotTxBuffer + LEN_FRAME_HEAD, (uint8_t*)&int_data.cmd_id, LEN_CMD_ID + int_data.frame_header.data_length);
																																					// 从帧头后（含CRC8）开始，向暂存数组中赋入命令码、数据段段头、数据段
	/* 帧尾 */
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ext_client_custom_int_seven_t));    // 尾CRC16校验，校验码放在数据末尾两个字节处，高八位在前
	
	/* 通过串口发送暂存数组CliendTxBuffer内的数据包 */
	uart_send_message(CliendTxBuffer, sizeof(ext_client_custom_int_seven_t));
}

/* 数据段摘出来封装一下 */
void data_int(int_data_struct_t *int_data, // 数据段内容位数
										// 通用设置
										const char *graphic_name,
										uint32_t operate_type,
										uint32_t graphic_type,
										uint32_t layer,
										uint32_t color,
										// 整型独有设置
										uint32_t size,
										// 通用设置
										uint32_t width,
										uint32_t start_x,
										uint32_t start_y,
										// 最终要展现的数据
										int		 number)
{
	memcpy(int_data->graphic_name, graphic_name, sizeof(graphic_name) / sizeof(char));  // 字符索引
	int_data->operate_type = operate_type;                       	// 图形操作
	int_data->graphic_type = graphic_type;												// 图形类型
	int_data->layer = layer;																			// 图层数
	int_data->color = color;																			// 颜色
	int_data->start_angle = size;																	// 起始角度
	int_data->width = width;																			// 线条宽度
	int_data->start_x = start_x;																	// 起点 x 坐标
	int_data->start_y = start_y;																	// 起点 y 坐标
	int_data->number = number;																		// 真实数据
	
}



	/**
  * @brief  串口一次发送一个字节数据
  * @param  自己打包好的要发给裁判的数据
  * @retval void
  * @attention  串口移位发送
  */
void uart_send_message(uint8_t *string, uint16_t length)
{
	HAL_UART_Transmit(&huart7, string, length, 100);
}


/*************** 获取裁判系统发来的本机各设备信息 ***************/

/* 单条的调用篇幅太多了， */
#if 0
/* 命名规则，单个参数直接返回 */

/*-------------------- PowerHeatData --------------------*/
// 获取底盘能量
fp32 get_chassis_power(void)
{
    return REF.power_heat_data.chassis_power;
}
// 能量增益
fp32 get_chassis_power_buff(void)
{
	return REF.power_heat_data.chassis_power_buffer;
}
// 1号17mm枪管当前热量
uint16_t get_shoot_heat1(void)
{
    return REF.power_heat_data.shooter_id1_17mm_cooling_heat;	
}
// 2号17mm枪管当前热量
uint16_t get_shoot_heat2(void)
{
    return REF.power_heat_data.shooter_id2_17mm_cooling_heat;	
}

/*-------------------- ShootData --------------------*/
// 获取当前射速??无法确认枪管ID
float get_shoot_bullet_speed(void)
{
	return REF.shoot_data.bullet_speed;
}
// 获取射频
uint8_t get_shoot_bullet_freq(void)
{
	return REF.shoot_data.bullet_freq;
}


/*-------------------- RoboStatus --------------------*/
// 获取机器人ID
uint8_t get_robot_id(void)
{
    return REF.robot_status.robot_id;
}
// 1一级，2二级，3三级
uint8_t get_robot_level(void)
{
		return REF.robot_status.robot_level;
}
// 机器人剩余血量
uint16_t get_robot_remain_HP(void)
{
	return REF.robot_status.remain_HP;
}
// 机器人上限血量
uint16_t get_robot_max_HP(void)
{
	return REF.robot_status.max_HP;
}

// 机器人 1 号 17mm 枪口每秒冷却值
uint16_t get_shooter_id1_17mm_cooling_rate(void)
{
	return REF.robot_status.shooter_id1_17mm_cooling_rate;
}
uint16_t get_shooter_id1_17mm_cooling_limit(void)
{
	return REF.robot_status.shooter_id1_17mm_cooling_limit;
}
uint16_t get_shooter_id1_17mm_speed_limit(void)
{
	return REF.robot_status.shooter_id1_17mm_speed_limit;	
}

// 机器人 2 号 17mm 枪口每秒冷却值
uint16_t get_shooter_id2_17mm_cooling_rate(void)
{
	return REF.robot_status.shooter_id2_17mm_cooling_rate;
}
uint16_t get_shooter_id2_17mm_cooling_limit(void)
{
	return REF.robot_status.shooter_id2_17mm_cooling_limit;
}
uint16_t get_shooter_id2_17mm_speed_limit(void)
{
	return REF.robot_status.shooter_id2_17mm_speed_limit;	
}

uint16_t get_chassis_power_limit(void)
{
	return REF.robot_status.chassis_power_limit;
}
uint8_t get_judge_gimbal_output(void)
{
	return REF.robot_status.mains_power_gimbal_output;
}
uint8_t get_judge_chassis_output(void)
{
	return REF.robot_status.mains_power_gimbal_output;
}
// shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
uint8_t get_judge_shoot_output(void)
{
	return REF.robot_status.mains_power_shooter_output;
}

#else 
const REF_t *get_REF_info_point(void)
{
	return &REF;
}

#endif
/*************** TaskInfo ***************/
// 判断自己是哪个队伍，并给己方机器人和客户端ID赋上 */
void Determine_ID(void)
{
	if(REF.robot_status.robot_id < 10)//本机器人的ID，红方
	{ 
		/* 判断己方机器人ID */
		REF.ids.teammate_hero 		 = RED_HERO;
		REF.ids.teammate_engineer  = RED_ENGINEER;
		REF.ids.teammate_infantry3 = RED_STANDARD_1;
		REF.ids.teammate_infantry4 = RED_STANDARD_2;
		REF.ids.teammate_infantry5 = RED_STANDARD_3;
		REF.ids.teammate_plane		 = RED_PLANE;
		REF.ids.teammate_sentry		 = RED_SENTRY;
		/* 判断己方客户端ID */
		REF.ids.client_hero 		 = client_hero_red;
		REF.ids.client_engineer  = client_engineer_red;
		REF.ids.client_infantry3 = client_infantry3_red;
		REF.ids.client_infantry4 = client_infantry4_red;
		REF.ids.client_infantry5 = client_infantry5_red;
		REF.ids.client_plane		 = client_plane_red;
		/* 判断本机对应的客户端ID */
		if     (REF.robot_status.robot_id == RED_HERO)//不断刷新放置在比赛中更改颜色
			REF.self_client = REF.ids.client_hero;
		else if(REF.robot_status.robot_id == RED_ENGINEER)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.robot_status.robot_id == RED_STANDARD_1)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.robot_status.robot_id == RED_STANDARD_2)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.robot_status.robot_id == RED_STANDARD_3)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.robot_status.robot_id == RED_PLANE)
			REF.self_client = REF.ids.client_plane;
	}
	else //蓝方
	{
		/* 判断己方机器人ID */
		REF.ids.teammate_hero 		 	= BLUE_HERO;
		REF.ids.teammate_engineer  = BLUE_ENGINEER;
		REF.ids.teammate_infantry3 = BLUE_STANDARD_1;
		REF.ids.teammate_infantry4 = BLUE_STANDARD_2;
		REF.ids.teammate_infantry5 = BLUE_STANDARD_3;
		REF.ids.teammate_plane		 	= BLUE_PLANE;
		REF.ids.teammate_sentry		= RED_SENTRY;
		/* 判断己方客户端ID */
		REF.ids.client_hero 		 	= client_hero_blue;
		REF.ids.client_engineer  = client_engineer_blue;
		REF.ids.client_infantry3 = client_infantry3_blue;
		REF.ids.client_infantry4 = client_infantry4_blue;
		REF.ids.client_infantry5 = client_infantry5_blue;
		REF.ids.client_plane			= client_plane_blue;
		/* 判断本机对应的客户端ID */
		if     (REF.robot_status.robot_id == BLUE_HERO)
			REF.self_client = REF.ids.client_hero;
		else if(REF.robot_status.robot_id == BLUE_ENGINEER)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.robot_status.robot_id == BLUE_STANDARD_1)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.robot_status.robot_id == BLUE_STANDARD_2)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.robot_status.robot_id == BLUE_STANDARD_3)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.robot_status.robot_id == BLUE_PLANE)
			REF.self_client = REF.ids.client_plane;
		
	}
}



