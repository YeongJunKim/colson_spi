/*
 * types.h
 *
 *  Created on: 2018. 10. 31.
 *      Author: COLSON
 */

#ifndef CTYPES_H_
#define CTYPES_H_
#include "stm32f0xx_hal.h"


//define modi can frame

/*
 *
 *
 *..........(********|********)||(********|********)||(********|********)||(********)||(********)*8
 *                 cmd                  sid                  did               len        data
 *
 *
 */


typedef struct _Frame
{
	uint8_t idle[10];
	uint16_t cmd;
	uint16_t sid;
	uint16_t did;
	uint8_t len;
	uint8_t data[8];
}C_dataFrame;

void c_frame_set_message(C_dataFrame* cframe, uint16_t cmd, uint16_t sid, uint16_t did, uint8_t len, uint8_t *data);
void c_frame_reset_message(C_dataFrame* cframe);


#endif /* CTYPES_H_ */

