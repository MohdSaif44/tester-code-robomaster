/*
 * usb.h
 *
 *  Created on: Sep 9, 2023
 *      Author: amer
 */

#ifndef INC_USB_H_
#define INC_USB_H_

#include "../Src/BIOS/system.h"
#include "FreeRTOS.h"
#include "message_buffer.h"

typedef
	 struct {
		uint16_t header;
		union {
		  float motors[14];
		  uint8_t buffer[56];
		};
		uint32_t flags;
		uint16_t footer;
}usb_msg_t;

enum{
	motor1,
	motor2,
	motor3,
	motor4,
	motor5,
	motor6,
	motor7,
	motor8,
	motor9,
	motor10,
	motor11,
	motor12,
	motor13,
	motor14,
};

char usbdata[64];
uint8_t usbBuf[64];
uint8_t ack;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

MessageBufferHandle_t usbrxBuffer;

void usbParserinit(void);
void usbHandler(uint8_t *Buf);
void usbSend(usb_msg_t out_msg);
void usbPrint(void);

#endif /* INC_USB_H_ */
