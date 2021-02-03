/*
This is a demo of how to use the espkfw library to access features of the the
very excellent keyboard featherwing by solder.party

https://kfw.solder.party/

I might have over engineered things by adding breakout to a simple demo...

ATTENTION:
Game assets are loaded from SD card, make sure you copy the espkfw folder to the SD card!

Press the buttons ROYGBIV on the keyboard to change the neopixel color.
Use the directions to move your paddle.

Sprites by Buch - https://opengameart.org/content/breakout-set

Code copyright (c) 2020 Adam Clifton <adam@numbatlogic.com>
All Rights Reserved

This program is free software. It comes without any warranty, to
the extent permitted by applicable law. You can redistribute it
and/or modify it under the terms of the Do What The Fuck You Want
To Public License, Version 2, as published by Sam Hocevar. See
http://sam.zoy.org/wtfpl/COPYING for more details.
*/


#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "soc/rtc.h"

#include "espkfw.h"
#include "lodepng.h"

// here are the pins used by the kfw, you'll need to map them to what you are using
// the defaults are for the Sparkfun Thing Plus or Adafruit Huzzah32
#define CONFIG_MOSI			18
#define CONFIG_MISO			19
#define CONFIG_SCLK			5
#define CONFIG_LCD_CS		15
#define CONFIG_LCD_DC		33
#define CONFIG_SD_CS		14
#define CONFIG_NEOPIXEL		27

#define MOUNT_POINT "/sdcard"

// 33ms per frame == 30fps
#define FRAME_TIME 33


// this is the buffer we draw to before blitting onto the screen
// 60x64 16bit pixels
#define SCREEN_BUFFER_SIZE 64
uint16_t* g_pBuffer;


// this is the background gradient
// we precompute it so it will be faster at runtime
#define BG_GRADIENT_SIZE 128
#define BG_GRADIENT_FROM 0x30346D
#define BG_GRADIENT_TO 0x442434
#define BG_GRADIENT_START_Y 50
#define BG_GRADIENT_SLOPE_SHIFT 4
uint16_t g_nBgColorArray[BG_GRADIENT_SIZE];


// a quick and dirty code for loading sprites from PNGs on disk and rendering them to a buffer
// uses the lodepng library
typedef struct Sprite Sprite;
struct Sprite
{
	int nWidth;
	int nHeight;
	uint16_t* pRgbData;
	uint8_t* pAlphaData;
};

Sprite* Sprite_Create(const char* szFileName)
{
	unsigned nError;
	uint8_t* pImage = 0;
	unsigned int nWidth;
	unsigned int nHeight;
	
	Sprite* pSprite = malloc(sizeof(Sprite));

	nError = lodepng_decode32_file(&pImage, &nWidth, &nHeight, szFileName);
	if (nError)
	{
		printf("failed to load %s %u: %s\n", szFileName, nError, lodepng_error_text(nError));

		// make a fallback if we can't find the png
		nWidth = 10;
		nHeight = 10;
		pImage = malloc(nWidth*nHeight*sizeof(uint32_t));

		uint8_t* p = pImage;
		for (unsigned int i = 0; i < nWidth * nHeight; i++)
		{
			*(p++) = 0xFF; // R
			*(p++) = 0xFF; // G
			*(p++) = 0x00; // B
			*(p++) = 0xFF; // A
		}
	}


	pSprite->nWidth = nWidth;
	pSprite->nHeight = nHeight;
	pSprite->pRgbData = malloc(nWidth*nHeight*sizeof(uint16_t));
	pSprite->pAlphaData = malloc(nWidth*nHeight*sizeof(uint8_t));

	// convert 32bit rgba into 16bit rgb plus 8bit alpha
	{
		uint8_t* p = pImage;
		for (unsigned int i = 0; i < nWidth * nHeight; i++)
		{
			uint8_t nR = *(p++);
			uint8_t nG = *(p++);
			uint8_t nB = *(p++);
			uint8_t nA = *(p++);

			pSprite->pRgbData[i] = espkfw_lcd_RgbToNative(nR, nG, nB);
			pSprite->pAlphaData[i] = nA;
		}
	}

	free(pImage);

	return pSprite;
}

void Sprite_Render(Sprite* pSprite, int nX, int nY, int nBufferX, int nBufferY, int nBufferWidth, int nBufferHeight, uint16_t* pBuffer)
{
	int i;
	int j;

	for (j = 0; j < pSprite->nHeight; j++)
	{
		if (nY + j < nBufferY)
			continue;

		if (nY + j >= nBufferY + nBufferHeight)
			break;

		for (i = 0; i < pSprite->nWidth; i++)
		{
			if (nX + i < nBufferX)
				continue;

			if (nX + i >= nBufferX + nBufferWidth)
				break;

			// too lazy to blend when rendering (tho we have enough cpu to do it)
			if (pSprite->pAlphaData[j * pSprite->nWidth + i] > 0)
				pBuffer[(nY + j - nBufferY)*nBufferWidth + (nX + i - nBufferX)] = pSprite->pRgbData[j * pSprite->nWidth + i];
		}
	}
}




// here are the sprites we'll be using
#define NUM_BLOCK 3
Sprite* g_pPaddleSprite;
Sprite* g_pBallSprite;
Sprite* g_pBlockSpriteArray[NUM_BLOCK];




















// game stuff
// shift all pixels up by this ammount as a cheap version of floating point, so we can have smoother acceleration and subpixel movement
#define SCALE_SHIFT 5
#define PADDLE_ACCELERATION 50
#define PADDLE_FRICTION_SHIFT 2
#define PADDLE_MIN_SPEED 10
#define PADDLE_BOUNCE_SHIFT 0

int g_nPaddleX;
int g_nPaddleY;
int g_nPaddleSpeed;

int g_nBallX;
int g_nBallY;
int g_nBallSpeedX;
int g_nBallSpeedY;

#define NUM_BLOCK_X 6
#define NUM_BLOCK_Y 5
#define BLOCK_PADDING 5
#define BLOCK_TOP 20

int g_nBlock[NUM_BLOCK_X][NUM_BLOCK_Y];
int g_nBlockX;
int g_nBlockY;

// input
int g_bLeftDown;
int g_bRightDown;


// draw a region of the screen then blit it to the LCD
void RenderRect(int nX, int nY, int nWidth, int nHeight, uint16_t* pBuffer)
{
	if (nX >= ESPKFW_SCREEN_WIDTH || nY >= ESPKFW_SCREEN_HEIGHT)
		return;

	if (nX < 0)
	{
		nWidth += nX;
		if (nWidth <= 0)
			return;
		nX = 0;
	}

	if (nX + nWidth > ESPKFW_SCREEN_WIDTH)
		nWidth = ESPKFW_SCREEN_WIDTH - nX;


	if (nY < 0)
	{
		nHeight += nY;
		if (nHeight <= 0)
			return;
		nY = 0;
	}

	if (nY + nHeight > ESPKFW_SCREEN_HEIGHT)
		nHeight = ESPKFW_SCREEN_HEIGHT - nY;

	// fill background
	{
		int i;
		int j;
		uint16_t* p = pBuffer;
		for (j = nY; j < nY + nHeight; j++)
		{
			for (i = nX; i < nX + nWidth; i++)
			{
				int nIndex = -BG_GRADIENT_START_Y - (i >> BG_GRADIENT_SLOPE_SHIFT) + j;
				if (nIndex < 0)
					nIndex = 0;
				if (nIndex >= BG_GRADIENT_SIZE)
					nIndex = BG_GRADIENT_SIZE - 1;
				*(p++) = g_nBgColorArray[nIndex];
			}
		}
	}

	// blocks
	{
		int i;
		int j;
		for (j = 0; j < NUM_BLOCK_Y; j++)
			for (i = 0; i < NUM_BLOCK_X; i++)
			{
				int nIndex = g_nBlock[i][j];
				if (nIndex < NUM_BLOCK)
				{
					Sprite* pSprite = g_pBlockSpriteArray[nIndex];
					Sprite_Render(pSprite, (g_nBlockX >> SCALE_SHIFT) + i * (pSprite->nWidth + BLOCK_PADDING), (g_nBlockY >> SCALE_SHIFT) + j * (pSprite->nHeight + BLOCK_PADDING), nX, nY, nWidth, nHeight, pBuffer);
				}
			}
	}

	// render sprites
	Sprite_Render(g_pPaddleSprite, g_nPaddleX >> SCALE_SHIFT, g_nPaddleY >> SCALE_SHIFT, nX, nY, nWidth, nHeight, pBuffer);
	Sprite_Render(g_pBallSprite, g_nBallX >> SCALE_SHIFT, g_nBallY >> SCALE_SHIFT, nX, nY, nWidth, nHeight, pBuffer);

	espkfw_lcd_Blit(nX, nY, nWidth, nHeight, pBuffer);
}


void Reset(void)
{
	// setup game
	g_nPaddleX = ((ESPKFW_SCREEN_WIDTH - g_pPaddleSprite->nWidth) / 2) << SCALE_SHIFT;
	g_nPaddleY = (ESPKFW_SCREEN_HEIGHT - g_pPaddleSprite->nHeight - 10) << SCALE_SHIFT;
	g_nPaddleSpeed = 0;


	// blocks
	{
		int nX;
		int nY;
		int nIndex = 0;
		for (nX = 0; nX < NUM_BLOCK_X; nX++)
			for (nY = 0; nY < NUM_BLOCK_Y; nY++)
				g_nBlock[nX][nY] = (nIndex++)%3;

		g_nBlockX = ((ESPKFW_SCREEN_WIDTH - (g_pBlockSpriteArray[0]->nWidth + BLOCK_PADDING) * NUM_BLOCK_X) / 2) << SCALE_SHIFT;
		g_nBlockY = BLOCK_TOP << SCALE_SHIFT;
	}


	g_nBallX = ((ESPKFW_SCREEN_WIDTH - g_pBallSprite->nWidth) / 2) << SCALE_SHIFT;
	g_nBallY = g_nBlockY + (((g_pBlockSpriteArray[0]->nHeight + BLOCK_PADDING) * NUM_BLOCK_Y + BLOCK_PADDING) << SCALE_SHIFT);

	g_nBallSpeedX = 2 << SCALE_SHIFT;
	g_nBallSpeedY = 2 << SCALE_SHIFT;


	g_bLeftDown = 0;
	g_bRightDown = 0;


	// do a render pass to initially fill the screen
	{
		uint16_t nX;
		uint16_t nY;
		for (nY = 0; nY < ESPKFW_SCREEN_HEIGHT; nY += SCREEN_BUFFER_SIZE)
			for (nX = 0; nX < ESPKFW_SCREEN_WIDTH; nX += SCREEN_BUFFER_SIZE)
				RenderRect(nX, nY, SCREEN_BUFFER_SIZE, SCREEN_BUFFER_SIZE, g_pBuffer);
	}

}

void app_main(void)
{
	espkfw_spi_Begin(CONFIG_MOSI, CONFIG_MISO, CONFIG_SCLK, sizeof(uint16_t) * SCREEN_BUFFER_SIZE * SCREEN_BUFFER_SIZE);
	espkfw_kb_Begin();
	espkfw_lcd_Begin(CONFIG_LCD_CS, CONFIG_LCD_DC);
	espkfw_sd_begin(CONFIG_SD_CS, MOUNT_POINT);
	espkfw_np_begin(CONFIG_NEOPIXEL);
	
	// turn off neopixel if it's still on from previous runs
	espkfw_np_set(0x000000);

	// load sprites
	g_pPaddleSprite = Sprite_Create("/sdcard/espkfw/paddle.png");
	g_pBallSprite = Sprite_Create("/sdcard/espkfw/ball.png");
	g_pBlockSpriteArray[0] = Sprite_Create("/sdcard/espkfw/block0.png");
	g_pBlockSpriteArray[1] = Sprite_Create("/sdcard/espkfw/block1.png");
	g_pBlockSpriteArray[2] = Sprite_Create("/sdcard/espkfw/block2.png");

	// this is our buffer for when we want to fraw to the screen
	g_pBuffer = malloc(sizeof(uint16_t) * SCREEN_BUFFER_SIZE * SCREEN_BUFFER_SIZE);

	// pre compute our background gradient
	{
		int i;

		int nFromR = (BG_GRADIENT_FROM & 0xFF0000) >> 16;
		int nFromG = (BG_GRADIENT_FROM & 0xFF00) >> 8;
		int nFromB = (BG_GRADIENT_FROM & 0xFF) >> 0;

		int nToR = (BG_GRADIENT_TO & 0xFF0000) >> 16;
		int nToG = (BG_GRADIENT_TO & 0xFF00) >> 8;
		int nToB = (BG_GRADIENT_TO & 0xFF) >> 0;

		for (i = 0; i < BG_GRADIENT_SIZE; i++)
		{
			int nR = nFromR + ((nToR - nFromR) * i / BG_GRADIENT_SIZE);
			int nG = nFromG + ((nToG - nFromG) * i / BG_GRADIENT_SIZE);
			int nB = nFromB + ((nToB - nFromB) * i / BG_GRADIENT_SIZE);

			g_nBgColorArray[i] = espkfw_lcd_RgbToNative((uint8_t)nR, (uint8_t)nG, (uint8_t)nB);
		}
	}

	Reset();

	
	// Update + Render loop
	int64_t nNextTime = esp_timer_get_time() / 1000;

	int64_t nFrameCount = 0;
	int64_t nFrameMicroSum = 0;
	int64_t nNextFrameAverage = nNextTime + 1000;

	while (1)
	{
		int64_t nTime = esp_timer_get_time() / 1000;
		if (nTime > nNextTime + FRAME_TIME * 60)
			nNextTime = nTime - 1;

		while (nTime > nNextTime)
		{
			int64_t nFrameMicroStart = esp_timer_get_time();

			// update
			nNextTime += FRAME_TIME;

			// handle input
			{
				char cKey;
				int nState;
				while (espkfw_kb_GetKey(&cKey, &nState))
				{
					// playing with neopixel colours
					if (nState == ESPKFW_KB_STATE_PRESS)
					{
						printf("key pressed: %c %d\n", cKey, cKey);

						uint32_t nR = 0x00;
						uint32_t nG = 0x00;
						uint32_t nB = 0x00;
						int bSet = 0;;

						switch (cKey)
						{
							// fun colours
							case 'r': nR = 0xFF; nG = 0x00; nB = 0x00; bSet = 1; break;
							case 'o': nR = 0xFF; nG = 0xA5; nB = 0x00; bSet = 1; break;
							case 'y': nR = 0xFF; nG = 0xFF; nB = 0x00; bSet = 1; break;
							case 'g': nR = 0x00; nG = 0xFF; nB = 0x00; bSet = 1; break;
							case 'b': nR = 0x00; nG = 0x00; nB = 0xFF; bSet = 1; break;
							case 'i': nR = 0x33; nG = 0x00; nB = 0x99; bSet = 1; break;
							case 'v': nR = 0x80; nG = 0x00; nB = 0x80; bSet = 1; break;
							case ' ': nR = 0x00; nG = 0x00; nB = 0x00; bSet = 1; break;
						}

						if (bSet)
							espkfw_np_set((nR/8 << 8) | (nG/8 << 16) | (nB/8 << 0)); // divide by 8 to make colours less blinding
					}


					// game controls
					if (nState == ESPKFW_KB_STATE_PRESS)
					{
						switch (cKey)
						{
							case ESPKFW_KB_KEY_LEFT: g_bLeftDown = 1; break;
							case ESPKFW_KB_KEY_RIGHT: g_bRightDown = 1; break;
						}
					}
					else if (nState == ESPKFW_KB_STATE_RELEASE)
					{
						switch (cKey)
						{
							case ESPKFW_KB_KEY_LEFT: g_bLeftDown = 0; break;
							case ESPKFW_KB_KEY_RIGHT: g_bRightDown = 0; break;
						}
					}
				}
			}

			// game logic
			{
				// paddle
				if (g_bLeftDown)
					g_nPaddleSpeed -= PADDLE_ACCELERATION;

				if (g_bRightDown)
					g_nPaddleSpeed += PADDLE_ACCELERATION;

				g_nPaddleSpeed -= g_nPaddleSpeed >> PADDLE_FRICTION_SHIFT;

				if (g_nPaddleSpeed > -PADDLE_MIN_SPEED && g_nPaddleSpeed < PADDLE_MIN_SPEED)
					g_nPaddleSpeed = 0;


				g_nPaddleX += g_nPaddleSpeed;

				if (g_nPaddleX < 0)
				{
					g_nPaddleX = 0;
					g_nPaddleSpeed = -g_nPaddleSpeed >> PADDLE_BOUNCE_SHIFT;
				}

				if (g_nPaddleX + (g_pPaddleSprite->nWidth << SCALE_SHIFT) > ESPKFW_SCREEN_WIDTH << SCALE_SHIFT)
				{
					g_nPaddleX = (ESPKFW_SCREEN_WIDTH - g_pPaddleSprite->nWidth) << SCALE_SHIFT;
					g_nPaddleSpeed = -g_nPaddleSpeed >> PADDLE_BOUNCE_SHIFT;
				}


				// ball
				g_nBallX += g_nBallSpeedX;
				g_nBallY += g_nBallSpeedY;

				if (g_nBallX < 0)
				{
					g_nBallX = 0;
					g_nBallSpeedX = -g_nBallSpeedX;
				}

				if (g_nBallX + (g_pBallSprite->nWidth << SCALE_SHIFT) > ESPKFW_SCREEN_WIDTH << SCALE_SHIFT)
				{
					g_nBallX = (ESPKFW_SCREEN_WIDTH - g_pBallSprite->nWidth) << SCALE_SHIFT;
					g_nBallSpeedX = -g_nBallSpeedX;
				}

				if (g_nBallY < 0)
				{
					g_nBallY = 0;
					g_nBallSpeedY = -g_nBallSpeedY;
				}


				// hit block
				{
					int i;
					int j;

					int nBlockY = g_nBlockY;					
					int nBlockWidth = g_pBlockSpriteArray[0]->nWidth << SCALE_SHIFT;
					int nBlockHeight = g_pBlockSpriteArray[0]->nHeight << SCALE_SHIFT;

					int nBallWidth = g_pBallSprite->nWidth << SCALE_SHIFT;
					int nBallHeight = g_pBallSprite->nHeight << SCALE_SHIFT;
					int nBallX0 = g_nBallX;
					int nBallY0 = g_nBallY;
					int nBallX1 = nBallX0 + nBallWidth;
					int nBallY1 = nBallY0 + nBallHeight;
					

					for (j = 0; j < NUM_BLOCK_Y; j++)
					{
						int nBlockY0 = nBlockY;
						int nBlockY1 = nBlockY0 + nBlockHeight;

						int nBlockX = g_nBlockX;
						for (i = 0; i < NUM_BLOCK_X; i++)
						{
							if (g_nBlock[i][j] < NUM_BLOCK)
							{
								int nBlockX0 = nBlockX;
								int nBlockX1 = nBlockX0 + nBlockWidth;

								int bHit = 0;

								int bVertically = nBallY0 < nBlockY1 && nBallY1 > nBlockY0;
								int bHorizontally = nBallX0 < nBlockX1 && nBallX1 > nBlockX0;

								// short side bounce, a bit more precise
								if (bVertically && // lines up vertically
									((nBallX0 < nBlockX0 && nBallX1 > nBlockX0) || // left side overlap
										(nBallX0 < nBlockX1 && nBallX1 > nBlockX1))) // right side overlap
								{
									bHit = 1;
									g_nBallSpeedX = -g_nBallSpeedX;
								}

								// long side bounce, a bit more generous
								else if (bHorizontally && // lines up horizontally
									bVertically) // lines up vertically
								{
									bHit = 1;
									g_nBallSpeedY = -g_nBallSpeedY;
								}

								if (bHit)
								{
									g_nBlock[i][j] = NUM_BLOCK;
									RenderRect(nBlockX0 >> SCALE_SHIFT, nBlockY0 >> SCALE_SHIFT, g_pBlockSpriteArray[0]->nWidth, g_pBlockSpriteArray[0]->nHeight, g_pBuffer);
								}
							}
							nBlockX += nBlockWidth + (BLOCK_PADDING << SCALE_SHIFT);
						}

						nBlockY += nBlockHeight + (BLOCK_PADDING << SCALE_SHIFT);
					}
				}





				// ball bounce
				if (g_nBallY < g_nPaddleY && g_nBallY + (g_pBallSprite->nHeight << SCALE_SHIFT) > g_nPaddleY)
				{
					if (g_nBallX >= g_nPaddleX && g_nBallX + (g_pBallSprite->nWidth << SCALE_SHIFT) <= g_nPaddleX + (g_pPaddleSprite->nWidth << SCALE_SHIFT))
					{
						g_nBallY = g_nPaddleY - (g_pBallSprite->nHeight << SCALE_SHIFT);
						g_nBallSpeedY = -g_nBallSpeedY;

						// adjust xspeed by where it lands on the paddle
						int nDiffX = (g_nBallX + (g_pBallSprite->nWidth << SCALE_SHIFT) / 2) - (g_nPaddleX + (g_pPaddleSprite->nWidth << SCALE_SHIFT) / 2);
						g_nBallSpeedX = nDiffX >> 3;
					}
				}


				// dead
				if (g_nBallY > (ESPKFW_SCREEN_HEIGHT + 20) << SCALE_SHIFT)
					Reset();
			}


			// renders
			// too lazy to work out the perfect rectangle render, just add padding around and overdraw
			RenderRect((g_nPaddleX >> SCALE_SHIFT) - 10, g_nPaddleY >> SCALE_SHIFT, g_pPaddleSprite->nWidth + 20, g_pPaddleSprite->nHeight, g_pBuffer);
			RenderRect((g_nBallX >> SCALE_SHIFT) - 10, (g_nBallY >> SCALE_SHIFT) - 10, g_pBallSprite->nWidth + 20, g_pBallSprite->nHeight + 20, g_pBuffer);


			nFrameCount++;
			nFrameMicroSum += (esp_timer_get_time() - nFrameMicroStart);
		}

		if (nTime > nNextFrameAverage)
		{
			if (nFrameCount > 0)
			{
				rtc_cpu_freq_config_t conf;
				rtc_clk_cpu_freq_get_config(&conf);
				printf("%lld Frames, %lld microseconds avg %d mhz\n", nFrameCount, nFrameMicroSum/nFrameCount, conf.freq_mhz);
			}

			nNextFrameAverage = nTime + 10*1000;
			nFrameCount = 0;
			nFrameMicroSum = 0;
		}

		// sleep till next update
		int64_t nDiff = nNextTime - esp_timer_get_time() / 1000;
		if (nDiff < 0)
			nDiff = 0;
		vTaskDelay(nDiff / portTICK_PERIOD_MS);
	}
}