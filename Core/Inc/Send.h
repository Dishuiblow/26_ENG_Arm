#ifndef __SEND_H
#define __SEND_H


#include "main.h"
#include "crc8_crc16.h"

void Send_Angle_Binary(void);
void Send_Servo_Angles(uint16_t angles[6],uint8_t mode);




#endif
