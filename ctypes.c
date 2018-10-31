/*
 * ctypes.c
 *
 *  Created on: 2018. 10. 31.
 *      Author: COLSON
 */

#include "ctypes.h"




void c_frame_set_message(C_dataFrame* cframe, uint16_t cmd, uint16_t sid, uint16_t did, uint8_t len, uint8_t *data)
{
	/* idle setting */
	for(int i = 0 ; i < 10; i ++)
		cframe->idle[i] = 0x00;
	cframe->idle[8] = 0xFF;
	cframe->idle[9] = 0xFF;

	/* param set */
	cframe->cmd = cmd;
	cframe->sid = sid;
	cframe->did = did;
	cframe->len = len;

	/* data set */
	for(int i = 0 ; i < 8 ; i++)
		cframe->data[i] = data[i];

}



void c_frame_reset_message(C_dataFrame* cframe)
{
	/* idle reset */
	for(int i = 0; i < 10; i++)
		cframe->idle[i] = 0x00;

	/* param reset */
	cframe->cmd = 0x0000;
	cframe->sid = 0x0000;
	cframe->did = 0x0000;
	cframe->len = 0x00;

	/* data reset */
	for(int i = 0; i < 8; i++)
		cframe->data[i] = 0x00;
}
