/*
	Code copyright (c) 2021 Adam Clifton <adam@numbatlogic.com>
	All Rights Reserved

	This program is free software. It comes without any warranty, to
	the extent permitted by applicable law. You can redistribute it
	and/or modify it under the terms of the Do What The Fuck You Want
	To Public License, Version 2, as published by Sam Hocevar. See
	http://sam.zoy.org/wtfpl/COPYING for more details.
*/


#if defined(ESP_PLATFORM)
	#include "espkfw.h"

	#include "driver/rmt.h"
	#include "driver/i2c.h"
	#include "esp_vfs_fat.h"

	#include <string.h>
	
	// + SPI Bus
	bool espkfw_spi_bBeginDone = false;
	void espkfw_spi_Begin(int nMosi, int nMiso, int nSclk, int nMaxTransfer)
	{
		if (!espkfw_spi_bBeginDone)
		{
			spi_bus_config_t config;
			memset(&config, 0, sizeof(config));

			config.mosi_io_num = nMosi;
			config.miso_io_num = nMiso;
			config.sclk_io_num = nSclk;
			config.quadwp_io_num = -1;
			config.quadhd_io_num = -1;
			config.max_transfer_sz = nMaxTransfer;
			//config.flags = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI);

			ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &config, 1));
			espkfw_spi_bBeginDone = true;
		}
	}

	void espkfw_spi_Write(spi_device_handle_t handle, const uint8_t* pData, size_t nDataSize)
	{
		spi_transaction_t transaction;
		memset(&transaction, 0, sizeof(transaction));
		transaction.length = nDataSize * 8;
		transaction.tx_buffer = pData;
		ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &transaction));
	}

	void espkfw_spi_Read(spi_device_handle_t handle, uint8_t* pData, size_t nDataSize)
	{
		spi_transaction_t transaction;
		memset(&transaction, 0, sizeof(transaction));
		transaction.length = nDataSize * 8;
		transaction.rx_buffer = pData;
		ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &transaction));
	}

	void espkfw_spi_Transaction(spi_device_handle_t handle, const uint8_t* pOutData, size_t nOutDataSize, uint8_t* pInData, size_t nInDataSize)
	{
		spi_transaction_t transaction;
		memset(&transaction, 0, sizeof(transaction));
		transaction.length = nOutDataSize * 8;
		transaction.tx_buffer = pOutData;

		transaction.rxlength = nInDataSize * 8;
		transaction.rx_buffer = pInData;

		ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &transaction));
	}




	// + LCD
	spi_device_handle_t espkfw_lcd_handle;
	int espkfw_lcd_nLcdCs;
	int espkfw_lcd_nLcdDc;

	void kfw_lcd_SpiCommand(uint8_t nCommand)
	{
		gpio_set_level(espkfw_lcd_nLcdDc, 0);
		espkfw_spi_Write(espkfw_lcd_handle, &nCommand, 1);
	}

	void kfw_lcd_SpiData(uint8_t* pData, uint16_t nDataSize)
	{
		gpio_set_level(espkfw_lcd_nLcdDc, 1);
		espkfw_spi_Write(espkfw_lcd_handle, pData, nDataSize);
	}

	void espkfw_lcd_Begin(int nLcdCs, int nLcdDc)
	{
		assert(espkfw_spi_bBeginDone);

		espkfw_lcd_nLcdCs = nLcdCs;
		espkfw_lcd_nLcdDc = nLcdDc;

		gpio_pad_select_gpio(espkfw_lcd_nLcdCs);
		gpio_set_direction(espkfw_lcd_nLcdCs, GPIO_MODE_OUTPUT );
		gpio_set_level(espkfw_lcd_nLcdCs, 0 );

		gpio_pad_select_gpio(espkfw_lcd_nLcdDc);
		gpio_set_direction(espkfw_lcd_nLcdDc, GPIO_MODE_OUTPUT );
		gpio_set_level(espkfw_lcd_nLcdDc, 0 );

		spi_device_interface_config_t devcfg={
			.clock_speed_hz = SPI_MASTER_FREQ_80M,
			.spics_io_num = espkfw_lcd_nLcdCs,
			.queue_size = 2,
			.flags = SPI_DEVICE_NO_DUMMY
		};

		ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &espkfw_lcd_handle));

		vTaskDelay(50 / portTICK_PERIOD_MS);

		kfw_lcd_SpiCommand(0x01); // soft reset

		vTaskDelay(50 / portTICK_PERIOD_MS);

		// now start firing commands ot the LCD
		{
			int i = 0;
			uint8_t pData[] = {
				0xEF, 3, 0x03, 0x80, 0x02,
				0xCF, 3, 0x00, 0xC1, 0x30,
				0xED, 4, 0x64, 0x03, 0x12, 0x81,
				0xE8, 3, 0x85, 0x00, 0x78,
				0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
				0xF7, 1, 0x20,
				0xEA, 2, 0x00, 0x00,
				0xC0, 1, 0x23,
				0xC1, 1, 0x10,
				0xC5, 2, 0x3E, 0x28,
				0xC7, 1, 0x86,
				0x36, 1, 0x40 | 0x20 | 0x08,
				0x3A, 1, 0x55,
				0x20, 0,
				0xB1, 2, 0x00, 0x18,
				0xB6, 4, 0x08, 0xA2, 0x27, 0x00,
				0x26, 1, 0x01,
				0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
				0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
				0x11, 0,
				0x29, 0,
			};

			while (i < sizeof(pData))
			{
				uint8_t* pSize;
				kfw_lcd_SpiCommand(pData[i++]);
				pSize = pData + (i++);
				if (*pSize > 0)
				{
					kfw_lcd_SpiData(pData + i, *pSize);
					i += *pSize;
				}
			}
		}
	}

	void espkfw_lcd_EndianFix(uint8_t* pData, uint16_t nDataSize)
	{
		uint16_t* p = (uint16_t*)pData;
		uint16_t* pEnd = p + (nDataSize >> 1);
		while (p < pEnd)
		{
			*p = (((*p) >> 8) & 0xFF) | (((*p) & 0xFF) << 8);
			p++;
		}
	}

	uint16_t espkfw_lcd_RgbToNative(uint8_t nR, uint8_t nG, uint8_t nB)
	{
		// in theory we coudl do this in one step
		uint16_t nTemp = (((nR & 0xf8)<<8) + ((nG & 0xfc)<<3) + (nB>>3)); // convert to 16bit color
		return ((nTemp >> 8) & 0xFF) | ((nTemp & 0xFF) << 8); // endian flip
	}

	void espkfw_lcd_Blit(uint16_t nX, uint16_t nY, uint16_t nWidth, uint16_t nHeight, const uint16_t* pPixels)
	{
		uint16_t nAddressArray[2];

		kfw_lcd_SpiCommand(0x2A); // column
		nAddressArray[0] = nX;
		nAddressArray[1] = nX + nWidth - 1;
		espkfw_lcd_EndianFix((uint8_t*)nAddressArray, 4);
		kfw_lcd_SpiData((uint8_t*)nAddressArray, 4);


		kfw_lcd_SpiCommand(0x2B); // row
		nAddressArray[0] = nY;
		nAddressArray[1] = nY + nHeight - 1;
		espkfw_lcd_EndianFix((uint8_t*)nAddressArray, 4);
		kfw_lcd_SpiData((uint8_t*)nAddressArray, 4);


		kfw_lcd_SpiCommand(0x2C); // ram
		kfw_lcd_SpiData((uint8_t*)pPixels, nWidth * nHeight * 2);
	}




	// + SD
	void espkfw_sd_begin(int nSdCs, const char* szMountPoint)
	{
		assert(espkfw_spi_bBeginDone);

		esp_vfs_fat_sdmmc_mount_config_t config;
		memset(&config, 0, sizeof(config));
		
		config.format_if_mount_failed = false;
		config.max_files = 5;
		config.allocation_unit_size = 16 * 1024;

		sdmmc_host_t host = SDSPI_HOST_DEFAULT();
		host.slot = HSPI_HOST;

		sdspi_device_config_t slotconfig = SDSPI_DEVICE_CONFIG_DEFAULT();
		slotconfig.gpio_cs = (gpio_num_t)nSdCs;

		sdmmc_card_t* card;
		ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(szMountPoint, &host, &slotconfig, &config, &card));
	}




	// Keyboard
	#define ESPKFW_KB_REG_VER 1
	#define ESPKFW_KB_REG_CFG 2
	#define ESPKFW_KB_REG_INT 3
	#define ESPKFW_KB_REG_KEY 4
	#define ESPKFW_KB_REG_BKL 5
	#define ESPKFW_KB_REG_DEB 6
	#define ESPKFW_KB_REG_FRQ 7
	#define ESPKFW_KB_REG_RST 8
	#define ESPKFW_KB_REG_FIF 9

	#define ESPKFW_KB_WRITE_MASK		(1 << 7)
	#define ESPKFW_KB_KEY_COUNT_MASK	(0x1F)

	#define ESPKFW_KB_I2C_PORT I2C_NUM_0
	//#define ESPKFW_KB_I2C_TIMEOUT (100 / portTICK_RATE_MS)

	#define ESPKFW_KB_ADDRESS (0x1F << 1)

	uint8_t espkfw_kb_readRegister8(uint8_t reg)
	{
		uint8_t nTemp = 0;
		i2c_cmd_handle_t cmd;
		
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ESPKFW_KB_ADDRESS | I2C_MASTER_WRITE, 0x1));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 0x1));
		ESP_ERROR_CHECK(i2c_master_start(cmd));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ESPKFW_KB_ADDRESS | I2C_MASTER_READ, 0x1));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &nTemp, 0x1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(ESPKFW_KB_I2C_PORT, cmd, 0x1));
		i2c_cmd_link_delete(cmd);

		return nTemp;
	}

	uint16_t espkfw_kb_readRegister16(uint8_t reg)
	{
		uint8_t nLow;
		uint8_t nHigh;
		i2c_cmd_handle_t cmd;
		
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ESPKFW_KB_ADDRESS | I2C_MASTER_WRITE, 0x1));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 0x1));
		ESP_ERROR_CHECK(i2c_master_start(cmd));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ESPKFW_KB_ADDRESS | I2C_MASTER_READ, 0x1));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &nLow, 0));
			ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &nHigh, 0x1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(ESPKFW_KB_I2C_PORT, cmd, 0x1));
		i2c_cmd_link_delete(cmd);

		return (nHigh << 8) | nLow;
	}

	void espkfw_kb_writeRegister8(uint8_t reg, uint8_t value)
	{
	    uint8_t data[2];
	    data[0] = reg | ESPKFW_KB_WRITE_MASK;
	    data[1] = value;

		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ESPKFW_KB_ADDRESS | I2C_MASTER_WRITE, 0x1));
			ESP_ERROR_CHECK(i2c_master_write(cmd, data, 2, 0x1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(ESPKFW_KB_I2C_PORT, cmd, 0x1));
		i2c_cmd_link_delete(cmd);
	}

	void espkfw_kb_Begin(void)
	{
		i2c_config_t conf;
			conf.mode = I2C_MODE_MASTER;
			conf.sda_io_num = 23;
			conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
			conf.scl_io_num = 22;
			conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
			conf.master.clk_speed = 100000;		//I2C frequency is the clock speed for a complete high low clock sequence
		ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
		ESP_ERROR_CHECK(i2c_driver_install(ESPKFW_KB_I2C_PORT, conf.mode, 0, 0, 0));
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ESPKFW_KB_ADDRESS | I2C_MASTER_WRITE, 0x1));
			ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ESPKFW_KB_REG_RST, 0x1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));

		ESP_ERROR_CHECK(i2c_master_cmd_begin(ESPKFW_KB_I2C_PORT, cmd, 0x1));

		i2c_cmd_link_delete(cmd);
		
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	uint8_t espkfw_kb_GetBacklight(void)
	{
	    return espkfw_kb_readRegister8(ESPKFW_KB_REG_BKL);
	}

	void espkfw_kb_SetBacklight(uint8_t value)
	{
	    espkfw_kb_writeRegister8(ESPKFW_KB_REG_BKL, value);
	}

	uint8_t espkfw_kb_GetKeyCount()
	{
	    return espkfw_kb_readRegister8(ESPKFW_KB_REG_KEY) & ESPKFW_KB_KEY_COUNT_MASK;
	}

	int espkfw_kb_GetKey(char* pKey, int* pState)
	{
		if (espkfw_kb_GetKeyCount() == 0)
	        return 0;

	    const uint16_t buf = espkfw_kb_readRegister16(ESPKFW_KB_REG_FIF);
	    (*pKey) = buf >> 8;
	    (*pState) = buf & 0xFF;

	    return 1;
	}




	// NeoPixel
	#define ESPKFW_NP_CHANNEL	0
	#define ESPKFW_NP_COLOR_BITS 24

	// These are the timings according to adafruit neopixel library
	/*#define ESPKFW_NP_T0H 400
	#define ESPKFW_NP_T0L 850
	#define ESPKFW_NP_T1H 800
	#define ESPKFW_NP_T1L 450*/

	// these are the timings according to the WS2812 pdf, same same but different
	#define ESPKFW_NP_T0H 350
	#define ESPKFW_NP_T0L 800
	#define ESPKFW_NP_T1H 700
	#define ESPKFW_NP_T1L 600

	uint32_t espkfw_np_nT0H;
	uint32_t espkfw_np_nT1H;
	uint32_t espkfw_np_nT0L;
	uint32_t espkfw_np_nT1L;

	void espkfw_np_begin(int nPin)
	{
		rmt_config_t config;
		memset(&config, 0, sizeof(config));
		config.rmt_mode = RMT_MODE_TX;
		config.channel = ESPKFW_NP_CHANNEL;
		config.gpio_num = nPin;
		config.clk_div = 2;
		config.mem_block_num = 1;

		ESP_ERROR_CHECK(rmt_config(&config));
		ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

		// convert nanoseconds to ticks
		uint32_t nClockHz = 0;
		rmt_get_counter_clock(config.channel, &nClockHz);

		// divide by 1 billion to get ticks to nanosecond ratio
		double fCounterClockRatio = nClockHz / 1000000000.0;

		espkfw_np_nT0H = (uint32_t)(fCounterClockRatio * ESPKFW_NP_T0H);
		espkfw_np_nT0L = (uint32_t)(fCounterClockRatio * ESPKFW_NP_T0L);
		espkfw_np_nT1H = (uint32_t)(fCounterClockRatio * ESPKFW_NP_T1H);
		espkfw_np_nT1L = (uint32_t)(fCounterClockRatio * ESPKFW_NP_T1L);
	}

	void espkfw_np_set(uint32_t nColor)
	{	
		rmt_item32_t itemArray[ESPKFW_NP_COLOR_BITS];
		nColor <<= 32 - ESPKFW_NP_COLOR_BITS;

		for (int i = 0; i < ESPKFW_NP_COLOR_BITS; i++)
		{
			if (nColor & 0x80000000) // highest bit set
				itemArray[i] = (rmt_item32_t){{{espkfw_np_nT1H, 1, espkfw_np_nT1L, 0}}};
			else
				itemArray[i] = (rmt_item32_t){{{espkfw_np_nT0H, 1, espkfw_np_nT0L, 0}}};
			nColor <<= 1;
		}

		ESP_ERROR_CHECK(rmt_write_items(ESPKFW_NP_CHANNEL, itemArray, ESPKFW_NP_COLOR_BITS, 0));
		ESP_ERROR_CHECK(rmt_wait_tx_done(ESPKFW_NP_CHANNEL, portMAX_DELAY));
	}
#endif