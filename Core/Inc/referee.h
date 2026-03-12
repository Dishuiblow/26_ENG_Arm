#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_PLANE      = 6,
    RED_SENTRY      = 7,
  
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_PLANE     = 16,
    BLUE_SENTRY     = 17,
  

} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;


/* 比赛状态数据：0x0001。发送频率：3Hz*/
typedef __packed struct        //0x001
{
	uint8_t game_type : 4;     //比赛类型
	uint8_t game_progress : 4; //当前比赛阶段
	uint16_t stage_remain_time;//当前阶段剩余时间 秒
	uint64_t SyncTimeStamp;    //UNIX时间
} ext_game_status_t;

//
/*比赛结果数据：0x0002。发送频率：比赛结束后发送*/
typedef __packed struct //0002
{
    uint8_t winner;
} ext_game_result_t;


typedef __packed struct/* 机器人血量数据：0x0003。发送频率：3Hz*/

{
  uint16_t red_1_robot_HP;//对应机器人的血量
  uint16_t red_2_robot_HP; 
  uint16_t red_3_robot_HP; 
  uint16_t red_4_robot_HP; 
	uint16_t reserved;       //5号步兵没有了 保留
  uint16_t red_7_robot_HP; 
  uint16_t red_outpost_HP;// 红方前哨站血量
  
  uint16_t red_base_HP; 
  uint16_t blue_1_robot_HP; 
  uint16_t blue_2_robot_HP; 
  uint16_t blue_3_robot_HP; 
  uint16_t blue_4_robot_HP; 
	uint16_t reserved1;      //5号步兵没有了 保留
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;// 蓝方前哨站血量
  uint16_t blue_base_HP;
} ext_game_robot_HP_t;


typedef __packed struct //0101    /*场地事件数据：0x0101。发送频率：3Hz*/

{
    uint32_t event_type;
} ext_event_data_t;


typedef __packed struct//0x0104   裁判警告信息：cmd_id
{
    uint8_t level;                //己方最后一次受到判罚的等级 1双方黄牌 2黄牌 3红牌 4判负
    uint8_t offending_robot_id; //己方最后一次受到判罚的违规机器人ID  （判负和双方黄牌时，该值为0）
  	uint8_t count;                //己方最后一次受到判罚的违规机器人对应判罚等级的违规次数

} ext_referee_warning_t;

typedef __packed struct  //0x0105     飞镖发射口倒计时  发送范围：己方机器人
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


typedef __packed struct/* 0x0201。 比赛机器人状态,发送频率：10Hz*/
{
   uint8_t robot_id;                // 本机器人ID，可用来校验发送
   uint8_t robot_level;						// 1一级，2二级，3三级
   uint16_t remain_HP;							// 机器人剩余血量
   uint16_t max_HP;								  // 机器人上限血量
	
   uint16_t shooter_id1_17mm_cooling_rate;		// 机器人 1 号 17mm 枪口每秒冷却值
   uint16_t shooter_id1_17mm_cooling_limit;		// 机器人 1 号 17mm 枪口热量上限 
   uint16_t shooter_id1_17mm_speed_limit;		// 机器人 1 号 17mm 枪口上限速度 单位 m/s
	
   uint16_t shooter_id2_17mm_cooling_rate;		// 机器人 2 号 17mm 枪口每秒冷却值
   uint16_t shooter_id2_17mm_cooling_limit;		// 机器人 2 号 17mm 枪口热量上限
   uint16_t shooter_id2_17mm_speed_limit;		// 机器人 2 号 17mm 枪口上限速度 单位 m/s
	
   uint16_t shooter_id1_42mm_cooling_rate;		// 机器人 42mm 枪口每秒冷却值
   uint16_t shooter_id1_42mm_cooling_limit;		// 机器人 42mm 枪口每秒冷却值
   uint16_t shooter_id1_42mm_speed_limit;		// 机器人 42mm 枪口上限速度 单位 m/s
	
   uint16_t chassis_power_limit;				// 机器人底盘功率限制上限
	
   uint8_t mains_power_gimbal_output : 1;   	// gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
   uint8_t mains_power_chassis_output : 1;		// chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
   uint8_t mains_power_shooter_output : 1;		// shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
} ext_game_robot_status_t;



typedef __packed struct//0x0202   /*实时功率热量数据：0x0202。发送频率：50Hz*/

{
  uint16_t reserved;                       //没了
	uint16_t reserved1;                      //没了
	float reserved2;                         //底盘功率保留没了
  
	uint16_t chassis_power_buffer; 	  // 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
	
	uint16_t shooter_id1_17mm_cooling_heat;   	// 1 号 17mm 枪口热量
	uint16_t shooter_id2_17mm_cooling_heat;		// 2 号 17mm 枪口热量
	uint16_t shooter_id1_42mm_cooling_heat;		// 42mm 枪口热量
} ext_power_heat_data_t;


typedef __packed struct //0x0203 /* 机器人位置：0x0203。发送频率：10Hz*/
{
    float x;      // 位置 x 坐标，单位 m
    float y;			// 位置 y 坐标，单位 m
    float z;			// 位置 z 坐标，单位 m
    float yaw;		// 位置枪口，单位度
} ext_game_robot_pos_t;



typedef __packed struct/*机器人增益：0x0204。发送频率：1Hz*/

{
  uint8_t recovery_buff;      //机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
	uint8_t cooling_buff;       //机器人枪口冷却倍率（直接值，值为5表示5倍冷却）
	uint8_t defence_buff;       //机器人防御增益（百分比，值为50表示50%防御增益）
	uint8_t vulnerability_buff; //机器人负防御增益（百分比，值为30表示-30%防御增益）
	uint16_t attack_buff;       //机器人攻击增益（百分比，值为50表示50%攻击增益）
} ext_buff_t;



typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
  /*伤害状态：0x0206。发送频率：伤害发生后发送*/
/*
bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的
五个装甲片，其他血量变化类型，该变量数值为 0。
bit 4-7：血量变化类型
0x0 装甲伤害扣血；
0x1 模块掉线扣血；
0x2 超射速扣血；
0x3 超枪口热量扣血；
0x4 超底盘功率扣血；
0x5 装甲撞击扣血
*/
} ext_robot_hurt_t;



typedef __packed struct //0x0207  /* 实时射击信息：0x0207。发送频率：射击后发送*/
{
    uint8_t bullet_type;		// 子弹类型: 1：17mm 弹丸 2：42mm 弹丸
    uint8_t shooter_id;			// 发射机构 ID：1：1 号 17mm 发射机构, 2：2 号 17mm 发射机构, 3：42mm 发射机构，用于判断是17mm还是42mm
    uint8_t bullet_freq;		// 子弹射频 单位 Hz
    float bullet_speed;			// 子弹射速 单位 m/s
} ext_shoot_data_t;



typedef __packed struct/*子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人发送*/

{
	uint16_t projectile_allowance_17mm;		// 17mm 子弹剩余发射数目
	uint16_t projectile_allowance_42mm;		// 42mm 子弹剩余发射数目
	uint16_t remaining_gold_coin;					  // 剩余金币数量
} ext_bullet_remaining_t;


typedef __packed struct
{
  uint32_t rfid_status;
  /*机器人 RFID 状态：0x0209。发送频率：3Hz，发送范围：单一机器人*/
/*
bit 0：基地增益点 RFID 状态；
bit 1：高地增益点 RFID 状态；
bit 2：能量机关激活点 RFID 状态；
bit 3：飞坡增益点 RFID 状态；
bit 4：前哨岗增益点 RFID 状态；
bit 6：补血点增益点 RFID 状态；
bit 7：工程机器人复活卡 RFID 状态；
bit 8-31：保留
*/
} ext_rfid_status_t;


typedef __packed struct   //0x020A   飞镖机器人客户端指令数据
{
	uint8_t dart_launch_opening_status; 	// 当前飞镖发射口的状态 ,1: 关闭；2：正在开启或者关闭中, 0：已经开启
	uint8_t reserved;                       //保留
  uint16_t target_change_time;				  // 切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0
  uint16_t operate_launch_cmd_time;			// 最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0
} ext_dart_client_cmd_t;


typedef __packed struct//0x020B
{
	float hero_x;       //己方对应机器人位置x轴坐标，单位：m
  float hero_y;       //己方对应机器人位置y轴坐标，单位：m
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
	float reserved;     //5号步兵没了
	float reserved1;    //5号步兵没了
} ground_robot_position_t;


typedef __packed struct  //0x020C
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
} radar_mark_data_t;//对方机器人易伤情况（需要按位分析解包） 英雄 工程 步兵 步兵 哨兵 


typedef __packed struct //0x20D
{ 
	uint32_t sentry_info;    //哨兵兑换发弹 买活所需金币 （需要按位分析解包）
    int16_t sentry_info_2; //哨兵是否脱战 17mm允许发弹 （需要按位分析解包）
} sentry_info_t; 


typedef __packed struct //0x20E
{ 
	uint8_t radar_info; //雷达易伤信息（需要按位解包）
} radar_info_t; 


typedef __packed struct //0x0301
{
	uint16_t data_cmd_id;//子内容ID
    uint16_t send_ID;    //发送者ID 
    uint16_t receiver_ID;//接收者ID
    uint8_t data[112];   //内容数据段
} ext_student_interactive_data_t;

typedef __packed struct //0x0302自定义控制器发送给机器人
{ 
	uint8_t data[30]; 
} custom_robot_data_t; 

typedef __packed struct //0x0309机器人发送给自定义控制器
{ 
	uint8_t data[30]; 
} robot_custom_data_t; 

/*****************************************************************************************************/
/* 客户端ID */
typedef enum
{
	client_hero_red 	     = 0x0101,
	client_engineer_red   = 0x0102,
	client_infantry3_red  = 0x0103,
	client_infantry4_red  = 0x0104,
	client_infantry5_red  = 0x0105,
	client_plane_red 	     = 0x0106,
	
	client_hero_blue 	     = 0x0165,
	client_engineer_blue  = 0x0166,
	client_infantry3_blue = 0x0167,
	client_infantry4_blue = 0x0168,
	client_infantry5_blue = 0x0169,
	client_plane_blue 	   = 0x016A,
	
} client_ID;

/* 自家机器人和客户端ID结构体 */
typedef struct{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;
	uint16_t teammate_radar;
	
	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;



typedef __PACKED_STRUCT 
{ 
 uint32_t sentry_cmd; 
} sentry_cmd_t; 
/* 交互数据接收信息：0x0301 */
typedef __packed struct
{
	frame_header_struct_t frame_header;										// 帧头
	uint16_t cmd_id;														// 命令码ID
	ext_student_interactive_header_data_t data_header;						// 交互数据段头
//	uint8_t data[113];														// 交互数据段，最大为113
	sentry_cmd_t command;
	uint16_t frame_tail;													// 帧尾
} ext_student_interactive_t;

/*************** UI ***************/

/* 串口协议第23页 */
/*-------------------- 图形数据协议 --------------------*/

/*  图形数据协议：直线、矩形、正圆、椭圆、圆弧、字符 */
typedef __packed struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_type:3; 
	uint32_t graphic_type:3; 
	uint32_t layer:4;        
	uint32_t color:4;        /*直线  矩形  正圆  椭圆  圆弧  浮点  整型  字符*/
	uint32_t start_angle:9;  // 空    空    空    空   起角  大小  大小  大小
	uint32_t end_angle:9;    // 空    空    空    空   终角  位数   空   长度
	uint32_t width:10;       //          线    条    宽    度
	uint32_t start_x:11;     //起点  起点  圆心  圆心  圆心  起点  起点  起点
	uint32_t start_y:11;     //起点  起点  圆心  圆心  圆心  起点  起点  起点
	uint32_t radius:10;      // 空    空   半径   空    空    、    、    空
	uint32_t end_x:11;       //终点  对顶   空   半轴  半轴   、    、    空
	uint32_t end_y:11;       //终点  对顶   空   半轴  半轴   数    数    空
} graphic_data_struct_t;

/* 图形数据协议：浮点数 */
typedef __packed struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_type:3; 
	uint32_t graphic_type:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  fp32 number;       
} float_data_struct_t;

/* 图形数据协议：整型数 */
typedef __packed struct//整型数
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_type:3; 
	uint32_t graphic_type:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} int_data_struct_t;

/* 对数据内容各位做的一些补充 */
// 图形配置1
// bit 0-2 图形操作   
typedef enum
{
	NONE   = 0,/*空操作*/
	ADD    = 1,/*增加图层*/
	MODIFY = 2,/*修改图层*/
	DELETE = 3,/*删除图层*/
} Graphic_Operate;//graphic_data_struct_t：uint32_t operate_tpye

// bit3-5 图层类型
typedef enum
{
	LINE      = 0,//直线
	RECTANGLE = 1,//矩形
	CIRCLE    = 2,//正圆
	OVAL      = 3,//椭圆
	ARC       = 4,//圆弧
	FLOAT     = 5,//浮点数
	INT       = 6,//整型数
	CHAR      = 7 //字符
} Graphic_Type;

// Bit 6-9：图层数，0~9

// Bit 10-13：颜色
typedef enum
{
	RED_BLUE  = 0,  //红蓝主色	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*紫红色*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*青色*/
	BLACK     = 7,
	WHITE     = 8
} Graphic_Color;

// Bit 14-22：起始角度，单位：°，范围[0,360]

// Bit 23-31：终止角度，单位：°，范围[0,360]

//图形配置2
// Bit 0-9：线宽

// Bit 10-20：起点 x 坐标

// Bit 21-31：起点 y 坐标

// 图形配置3
// Bit 0-9：字体大小或者半径

// Bit 10-20：终点 x 坐标

// Bit 21-31：终点 y 坐标

/*-------------------- 绘图操作协议 --------------------*/

/* 删除图层 */

// 数据段（删除图层）
typedef __packed struct
{
uint8_t operate_tpye; 
uint8_t layer; // 图层：0-9
} ext_client_custom_graphic_delete_t;
// 删除类型
typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
} delete_Graphic_Operate;//ext_client_custom_graphic_delete_t：uint8_t operate_type
// 数据包（删除图层）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	ext_client_custom_graphic_delete_t graphic_delete_data;       // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_deleteLayer_data_t;

/* 绘制图像 */
// 数据包（单个图像）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	graphic_data_struct_t graphic_data;											      // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_graphic_single_t;

// 数据包（两个图像）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	graphic_data_struct_t graphic_data[2];											  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_graphic_double_t;

// 数据包（五个图像）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	graphic_data_struct_t graphic_data[5];											  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_graphic_five_t;

// 数据包（七个图像）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	graphic_data_struct_t graphic_data[7];											  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_graphic_seven_t;

/* 绘制浮点型 */
// 数据包（单个浮点型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	float_data_struct_t float_data;															  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_float_single_t;

// 数据包（两个浮点型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	float_data_struct_t float_data[2];  												  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_float_double_t;

// 数据包（五个浮点型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	float_data_struct_t float_data[5];													  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_float_five_t;

// 数据包（七个浮点型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	float_data_struct_t float_data[7];													  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_float_seven_t;

/* 绘制整型 */
// 数据包（单个整型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	int_data_struct_t int_data;															  		// UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_int_single_t;

// 数据包（两个整型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	int_data_struct_t int_data[2];  												  		// UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_int_double_t;

// 数据包（五个整型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	int_data_struct_t int_data[5];															  // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_int_five_t;

// 数据包（七个整型）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	int_data_struct_t int_data[7];													  		// UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_client_custom_int_seven_t;

/* 绘制字符 */
// 数据段（字符）
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct; // 字符配置
	uint8_t data[30];													// 字符
} ext_client_custom_character_t;

// 数据包（字符）
typedef __packed struct
{
	frame_header_struct_t frame_header;														// 帧头
	uint16_t cmd_id;																							// 命令码ID
	ext_student_interactive_header_data_t	data_header;   					// UI交互数据段头
	ext_client_custom_character_t character;								      // UI交互数据
	uint16_t frame_tail;																					// 帧尾						
} ext_character_data_t;

/* robot command:0x0303. to all robots, cycle not less than 3s */
typedef __packed struct
{
	float target_position_x;			 // meter
	float target_position_y;			 // meter
	float target_position_z;			 // meter
	uint8_t commd_keyboard;        // when no touching, it's 0
	uint16_t target_robot_ID;			 // when sending position data, it's 0
} ext_robot_command_t;


/*************************************************************************************************************/
typedef struct
{
	frame_header_struct_t           referee_receive_header;     // 接收者帧头信息
	frame_header_struct_t           referee_send_header;        // 发送者帧头信息

	ext_game_status_t               game_status;                // 比赛状态数据，1Hz 周期发送
	ext_game_result_t               game_result;                // 比赛结果数据，比赛结束后发送
	ext_game_robot_HP_t             game_robot_HP;              // 比赛机器人血量数据，1Hz 周期发送
	ext_event_data_t                field_event;                // 场地事件数据，1Hz 周期发送
	
	ext_referee_warning_t             referee_warning;          	// 裁判警告数据，警告发生后发送
	ext_dart_remaining_time_t         ext_dart_remaining_time;	// 飞镖发射口倒计时, 3Hz 周期发送，发送范围：己方机器人
	ext_game_robot_status_t           robot_status;               // 机器人状态数据，10Hz 周期发送
	ext_power_heat_data_t              power_heat_data;          	// 实时功率热量数据，50Hz 周期发送
  
	ext_game_robot_pos_t              game_robot_pos;           	// 机器人位置数据，10Hz 发送
	ext_buff_t                 			   buff;                		// 机器人增益数据，增益状态改变后发送
	ext_robot_hurt_t                   robot_hurt;					// 伤害状态
	ext_shoot_data_t                   shoot_data;					// 实时射击信息(射频  射速  子弹信息)
	ext_bullet_remaining_t            bullet_remaining;			// 子弹剩余发射数
	ext_rfid_status_t               	   rfid_status;				// 机器人 RFID 状态
  
	ext_dart_client_cmd_t            	 dart_client_cmd;			// 飞镖机器人客户端指令数据
	ground_robot_position_t           	ground_robot_positon;		// team ground robot position
	radar_mark_data_t                 	 radar_mark_data;			// radar mark data
	sentry_info_t                       sentry_info;          //哨兵裁判系统信息（允许发弹量，兑换，脱战，买活）
	
	ext_robot_command_t            	    robot_command;				// map data
	ext_interact_id_t								ids;												//与本机交互的机器人id
	uint16_t                        	self_client;        		//本机客户端
	radar_info_t                    	radar_info;           //雷达易伤信息
	ext_student_interactive_data_t      ext_student_interactive_data; 
	
	custom_robot_data_t  				custom_robot_data;
	robot_custom_data_t					robot_custom_data;
} REF_t;


typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;



extern void init_referee_struct_data(void);

extern void referee_data_solve(uint8_t *frame);

extern void common_data_send(void);

extern void char_send(const uint16_t *team_client_id);
extern void data_char(ext_client_custom_character_t *char_data, // 最终发送的数据段内容
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
										const char *character);
										
extern void graphic_send(const uint16_t *team_client_id);
extern void data_graphic(graphic_data_struct_t *graphic_data, // 数据段内容
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
										uint32_t end_y);

extern void float_send(const uint16_t *team_client_id);
extern void data_float(float_data_struct_t *float_data, // 数据段内容位数
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
										fp32		 number);

extern void int_send(const uint16_t *team_client_id);
extern void data_int(int_data_struct_t *int_data, // 数据段内容位数
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
										int		 number);
										

extern void uart_send_message(uint8_t *string, uint16_t length);

/*-------------------- PowerHeatData --------------------*/
extern fp32 get_chassis_power(void);
extern fp32 get_chassis_power_buff(void);
extern uint16_t get_shoot_heat1(void);
extern uint16_t get_shoot_heat2(void);

/*-------------------- ShootData --------------------*/
extern float get_shoot_bullet_speed(void);
extern uint8_t get_shoot_bullet_freq(void);

/*-------------------- RoboStatus --------------------*/
extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);
extern uint16_t get_robot_remain_HP(void);
extern uint16_t get_robot_max_HP(void);

extern uint16_t get_shooter_id1_17mm_cooling_rate(void);
extern uint16_t get_shooter_id1_17mm_cooling_limit(void);
extern uint16_t get_shooter_id1_17mm_speed_limit(void);

extern uint16_t get_shooter_id2_17mm_cooling_rate(void);
extern uint16_t get_shooter_id2_17mm_cooling_limit(void);
extern uint16_t get_shooter_id2_17mm_speed_limit(void);

extern uint16_t get_chassis_power_limit(void);

extern uint8_t get_judge_gimbal_output(void);
extern uint8_t get_judge_chassis_output(void);
extern uint8_t get_judge_shoot_output(void);
extern ext_student_interactive_t student_interactive;
/*************** TaskInfo ***************/
extern void Determine_ID(void);

extern const REF_t *get_REF_info_point(void); 
// 全局变量
extern REF_t REF;
#endif
