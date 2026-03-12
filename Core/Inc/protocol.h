#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5										  // 数据帧起始字节，固定值为 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)
/*裁判系统串口协议
*/

/*命令码 数据段长度 功能说明
0x0001 11 比赛状态数据，3Hz 周期发送
0x0002 1 比赛结果数据，比赛结束后发送
0x0003 28 比赛机器人血量数据，3Hz 周期发送
0x0004 3 飞镖发射状态，飞镖发射后发送
0x0005 11 人工智能挑战赛加成与惩罚状态，1Hz 周期发送
0x0101 4 场地事件数据，3Hz 周期发送
0x0102 3 场地补给站动作标识数据，动作改变后发送
0x0103 2 请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
0x0104 2 裁判警告数据，警告发生后发送
0x0105 1 飞镖发射口倒计时，3Hz 周期发送
0x0201 15 机器人状态数据，10Hz 周期发送
0x0202 14 实时功率热量数据，50Hz 周期发送
0x0203 16 机器人位置数据，10Hz 周期发送
0x0204 1 机器人增益数据，1Hz 周期发送
0x0205 3 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
0x0206 1 伤害状态数据，伤害发生后发送
0x0207 6 实时射击数据，子弹发射后发送
0x0208 2 子弹剩余发送数，空中机器人以及哨兵机器人发送，10Hz 周期发送
0x0209 4 机器人 RFID 状态，3Hz 周期发送
0x020A 12 飞镖机器人客户端指令书，10Hz 周期发送
0x0301 n 机器人间交互数据，发送方触发发送，上限 10Hz
0x0302 n 自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
0x0303 15 客户端小地图交互数据，触发发送
0x0304 12 键盘、鼠标信息，通过图传串口发送
0x0305 10 客户端小地图接收信*/

// 命令码ID 说明
typedef enum
{
	ID_game_state       				= 0x0001, // 比赛状态数据，3Hz 周期发送 
	ID_game_result      				= 0x0002, // 比赛结果数据，比赛结束后发送
	ID_game_robot_survivors     = 0x0003,//比赛机器人存活数据
	ID_game_missile_state 			= 0x0004, //飞镖发射状态
	ID_game_buff         				= 0x0005,//		ICRA增益
	
	ID_event_data  							= 0x0101,//场地事件数据 
	ID_supply_projectile_action = 0x0102,//场地补给站动作标识数据
	ID_supply_warm 							= 0x0104,//裁判系统警告数据
	ID_missile_remain_time 			= 0x0105,//飞镖发射口倒计时
	
	ID_game_robot_state    			= 0x0201,//机器人状态数据
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据
	ID_buff_musk								= 0x0204,//机器人增益数据
	ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
	ID_robot_hurt								= 0x0206,//伤害状态数据
	ID_shoot_data								= 0x0207,//实时射击数据
	ID_bullet_remaining         = 0x0208,//剩余发射数
	ID_rfid_status							= 0x0209,//机器人RFID状态，1Hz
	
	ID_dart_client_directive   = 0x020A, // 飞镖机器人客户端指令书, 10Hz
	ID_robot_position						= 0x020B, // ground robots position, 1Hz
	ID_radar_mark								= 0x020C, // radar mark data, 1Hz
  ID_sentry_info							= 0x020D, // 
  ID_radar_info							  = 0x020E, 

  
	ID_robot_interactive_data			= 0x0301,//机器人交互数据，——发送方触发——发送 10Hz
	ID_controller_interactive_header_data = 0x0302,//自定义控制器交互数据接口，通过——客户端触发——发送 30Hz
	ID_map_interactive_header_data        = 0x0303,//客户端小地图交互数据，——触发发送——
	ID_keyboard_information               = 0x0304,//键盘、鼠标信息，通过——图传串口——发送
	ID_map_information										= 0x305  //client map data 

} CmdID;

/* 	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。 */
/* 帧头frame_header 格式 */
typedef __packed struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

/* 帧头各域偏移地址 */
typedef enum
{
	SOF          = 0,				// 起始位,固定值为0xA5
	DATA_LENGTH  = 1,				// 数据帧内data长度,根据这个来获取数据长度
	SEQ          = 3,				// 包序号
	CRC8         = 4 				// CRC8	
}	FrameHeaderOffset;

/* 数据段段头结构 */
typedef __packed struct
{
 uint16_t data_cmd_id;		// 数据段的内容ID
 uint16_t sender_ID;			// 发送者的ID
 uint16_t receiver_ID;		// 接收者的ID
}ext_student_interactive_header_data_t;

/* 
	学生机器人间通信 cmd_id 0x0301，内容 data_ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：数据上下行合计带宽不超过 5000 Byte。 上下行发送频率分别不超过30Hz。
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | data_cmd_id | 0x0200~0x02FF,可以在这些 ID 段选取 |
 * |      |      |             | 具体ID含义由参赛队自定义           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | 需要校验发送者的 ID 正确性					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | 需要校验接收者的 ID 正确性					|
 * |      |      |             | 例如不能发送到敌对机器人的ID				| 
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n 需要小于 113 										|
 * +------+------+-------------+------------------------------------+
*/

/*************** data_ID ***************/
enum
{
	// 队伍自定义命令：0x200-0x02ff 格式  INTERACT_ID_XXXX
	INTERACT_ID_data_of_hero 		   	= 0x0200,	/* 队伍英雄机器人数据ID */
	INTERACT_ID_data_of_engineer 		= 0x0201,	/* 队伍工程机器人数据ID */
	INTERACT_ID_data_of_infantry3 	= 0x0202,	/* 队伍3号步兵机器人数据ID */
	INTERACT_ID_data_of_infantry4 	= 0x0203,	/* 队伍4号步兵机器人数据ID */
	INTERACT_ID_data_of_infantry5 	= 0x0204,	/* 队伍5号步兵机器人数据ID */
	INTERACT_ID_data_of_plane 			= 0x0205,	/* 队伍无人机数据ID */
	INTERACT_ID_data_of_sentry			= 0x0206,	/* 队伍哨兵机器人数据ID */
	INTERACT_ID_data_of_radar				= 0x0207,	/* 队伍雷达站数据ID */
};

enum
{
	// 固定数据内容ID：不可更改     格式  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*客户端绘制字符图形*/
	INTERACT_ID_bigbome_num					= 0x02ff
};

/* 数据长度 */
typedef enum
{
	/* 通信协议格式 */
	LEN_FRAME_HEAD 	              =  5,	// 帧头长度，包含CRC8校验码长度
	LEN_CMD_ID 		                =  2,	// 命令码长度
	LEN_FRAME_TAIL 	              =  2,	// 帧尾CRC16
	
	/* cmd_id 命令码 ID 说明 */ 
	LEN_game_state       					= 11,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_survivors      = 32,	//0x0003  比赛机器人血量数据
	LED_game_missile_state        =  3, //0X0004飞镖发射状态
	LED_game_buff                 = 11, //0X0005
	
	LEN_event_data  							=  4,	//0x0101  场地事件数据 
	LEN_supply_projectile_action  =  4,	//0x0102场地补给站动作标识数据
	LEN_supply_warm        				=  2, //裁判系统警告 0x0104
	LEN_missile_shoot_time   			=  1, //飞镖发射口倒计时
	
	LEN_game_robot_state    			= 27,	//0x0201机器人状态数据
	LEN_power_heat_data   				= 16,	//0x0202实时功率热量数据
	LEN_game_robot_pos        		= 16,	//0x0203机器人位置数据
	LEN_buff_musk        				  =  1,	//0x0204机器人增益数据
	LEN_aerial_robot_energy       =  1,	//0x0205空中机器人能量状态数据
	LEN_robot_hurt        				=  1,	//0x0206伤害状态数据
	LEN_shoot_data       				  =  7,	//0x0207	实时射击数据
	LEN_bullet_remaining          =  6, //剩余发射数
  
	LEN_rfid_status					      =  4,
	LEN_dart_client_directive     = 12,//0x020A
	// 0x030x
	//LEN_robot_interactive_header_data      = n,
	//LEN_controller_interactive_header_data = n,
	LEN_map_interactive_headerdata           = 15,
	LEN_keyboard_information                 = 12,//0x0304

}JudgeDataLength;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
