/*
	This is a library for the ESP32 to access *most* of the features of the keyboard
	featherwing by solder.party

	https://kfw.solder.party/

	Note that touch screen, and lcd backlight is not (yet) supported.

	Code copyright (c) 2020 Adam Clifton <adam@numbatlogic.com>
	All Rights Reserved

	This program is free software. It comes without any warranty, to
	the extent permitted by applicable law. You can redistribute it
	and/or modify it under the terms of the Do What The Fuck You Want
	To Public License, Version 2, as published by Sam Hocevar. See
	http://sam.zoy.org/wtfpl/COPYING for more details.
*/

#ifndef KFW_H
#define KFW_H

#if defined(ESP_PLATFORM)

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

	/*
		These are the pins used for the Adafruit Huzzah32 and SparkFun ESP32 Thing Plus

		#define CONFIG_MOSI			18
		#define CONFIG_MISO			19
		#define CONFIG_SCLK			5
		#define CONFIG_LCD_CS		15
		#define CONFIG_LCD_DC		33
		#define CONFIG_SD_CS		14
		#define CONFIG_NEOPIXEL		27
	*/


	// + SPI
	// the SPI bus is shared between LCD, Touchscreen and SD
	// you need to initialise it before using those

	// max transfer is the maximum you intend to send over spi
	// this is usually the size of youre screen buffer
	// 0 will default to 4097
	extern void espkfw_spi_Begin(int nMosi, int nMiso, int nSclk, int nMaxTransfer);




	// + LCD
	// References:
	//	https://github.com/adafruit/Adafruit_ILI9341
	//	https://github.com/nopnop2002/esp-idf-ili9340
	
	// make sure you call kfw_spi_Begin first

	// we cannot allocate a large enough single memory block to hold the entire screen on a little esp32
	// we could instead:
	//	+ split the screen into smaller buffers, but blitting the whole screen every frame is too slow
	//	something like 25fps
	//	+ split into smaller checksummed buffers like this (arduino) library:
	//		https://github.com/jeanlemotan/jlm-back-buffer
	//	that gives exellent speed and usability if you can avoid updating too much of the screen at once
	//	but it can be wasteful to ram and cpu
	//	+ using a smart dirty rect system would probably be the optimal thing

	#define ESPKFW_SCREEN_WIDTH 320
	#define ESPKFW_SCREEN_HEIGHT 240
	
	// initial setup
	extern void espkfw_lcd_Begin(int nLcdCs, int nLcdDc);

	// The lcd panel takes data at a different endianess than the esp32, this function will
	// correct the data to ready it for blitting to the lcd, by modifying the data in place.
	// Preferbaly you could work entirely in the native format, and avoid having to convert 
	extern void espkfw_lcd_EndianFix(uint8_t* pData, uint16_t nDataSize);

	// 24bit RGB to the native 16bit little endian format
	extern uint16_t espkfw_lcd_RgbToNative(uint8_t nR, uint8_t nG, uint8_t nB);

	// draw data to the screen
	extern void espkfw_lcd_Blit(uint16_t nX, uint16_t nY, uint16_t nWidth, uint16_t nHeight, const uint16_t* pPixels);


	// + Touch
	// References:
	//	https://github.com/adafruit/Adafruit_STMPE610
	//	https://github.com/loboris/ESP32_TFT_library

	// too hard gave up x_x
	//extern void kfw_touch_Begin(int nTouchCs);


	// + SD
	// References:
	//	https://github.com/espressif/esp-idf/tree/master/examples/storage/sd_card

	// mounts the sd card to the given mount point, then you can access files through normal file actions
	// fopen etc.
	// note that the FAT filesystem will be accessible in 8.3 format
	// (there is an idf.py menuconfig option to change this)
	extern void espkfw_sd_begin(int nSdCs, const char* szMountPoint);



	// + Keyboard
	// References:
	//	https://github.com/arturo182/arduino_bbq10kbd/tree/master/src
	// you can see in the references there is the possibility of attaching an interrupt
	// but i didn't bother here
	// it might be good to do as there seems to be some blocking occouring when the cpu is clocked above 80mhz...

	#define ESPKFW_KB_STATE_IDLE 0
	#define ESPKFW_KB_STATE_PRESS 1
	#define ESPKFW_KB_STATE_LONG_PRESS 2
	#define ESPKFW_KB_STATE_RELEASE 3

	#define ESPKFW_KB_KEY_UP 1
	#define ESPKFW_KB_KEY_DOWN 2
	#define ESPKFW_KB_KEY_LEFT 3
	#define ESPKFW_KB_KEY_RIGHT 4

	#define ESPKFW_KB_KEY_SOFTKEY_0 6
	#define ESPKFW_KB_KEY_SOFTKEY_1 17
	#define ESPKFW_KB_KEY_SOFTKEY_2 7
	#define ESPKFW_KB_KEY_SOFTKEY_3 18

	// initial setup
	extern void espkfw_kb_Begin(void);

	// get and set the keyboard backlight 0 - 255 values
	extern uint8_t espkfw_kb_GetBacklight(void);
	extern void espkfw_kb_SetBacklight(uint8_t value);

	// get a keyboard event
	//	pKey and pState are pointers to store the resulting state
	// basically you keep calling it and acting on the incomming characters till it returns 0
	// eg: while (espkfw_kb_GetKey(&cKey, &nState) { do something with cKey and nState }
	// see ESPKFW_KB_STATE_* for the states, usually you only care about PRESS and RELEASE
	extern int espkfw_kb_GetKey(char* pKey, int* pState);



	// + NeoPixel
	// References:
	//	https://github.com/adafruit/Adafruit_NeoPixel
	//	https://spin.atomicobject.com/2018/11/03/neopixels-esp32/ and https://github.com/JSchaenzle/ESP32-NeoPixel-WS2812-RMT
	//	https://cdn-shop.adafruit.com/datasheets/WS2812.pdf
	
	// Initial NeoPixel setup, need to call this before espkfw_np_set will work
	//	nPin is the NeoPixel GPIO pin
	extern void espkfw_np_begin(int nPin);

	// sets a new color on the NeoPixel
	//	nColor is the color to set
	//		8 bits per channel (0 - 255), higher values are brighter
	//		but anything more than 2 is blinding, you have been warned
	//		0xGGRRBB or (RR << 8) | (GG << 16) | (BB << 0)
	// note that the color will persist after soft reset, so you might want to manually set to 0 during startup
	extern void espkfw_np_set(uint32_t nColor);

#ifdef __cplusplus
}
#endif

#endif

#endif