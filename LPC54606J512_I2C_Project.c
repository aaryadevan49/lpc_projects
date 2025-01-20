/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    LPC54606J512_I2C_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_i2c.h"
#include "clock_config.h"
#include "LPC54606.h"
/* TODO: insert other include files here. */
#define i2c_polling 0
#define spi_polling 1
#if spi_polling
#define EXAMPLE_SPI_MASTER          SPI9
#define EXAMPLE_SPI_MASTER_IRQ      FLEXCOMM9_IRQn
#define EXAMPLE_SPI_MASTER_CLK_SRC  kCLOCK_Flexcomm9
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(9)
#define EXAMPLE_SPI_SSEL            0
#define EXAMPLE_SPI_SPOL            kSPI_SpolActiveAllLow

#define BUFFER_SIZE (64)
static uint8_t srcBuff[BUFFER_SIZE];
static uint8_t destBuff[BUFFER_SIZE];
#endif
#if i2c_polling
#define EXAMPLE_I2C_MASTER_BASE    (I2C3_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define WAIT_TIME                  10U
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)
#define PRINTF  printf
#define I2C_MASTER_SLAVE_ADDR_7BIT 0x36//0x6C//0x1D//0x7EU
#define I2C_BAUDRATE               100000U
#define I2C_DATA_LENGTH            33U


#define MMA8652FC_SLAVE_ADDR_7BIT 0x1D // Address when SA0 pin is connected to VDD
#define MMA8652FC_WHO_AM_I_REG 0x0D
#define MMA8652FC_CTRL_REG1 0x2A
#define MMA8652FC_OUT_X_MSB 0x01



unsigned short g_slave_buff;

uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];
i2c_master_handle_t g_m_handle;
volatile bool completionFlag = false;
volatile bool g_MasterCompletionFlag = false;

uint8_t designcap_register = 0x18;
uint8_t vge_fuel_gauge = 0xff;
uint8_t open_ckt_vge_fuel_gauge = 0xfb;
uint8_t tte_reg = 0x11;
uint8_t ttf_reg = 0x20;
uint8_t age_register= 0x07;
uint8_t soft_wkup = 0x60;
uint8_t min_sys_voltage = 0xD8;
#endif
/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

#if i2c_polling


void I2C_MasterCallback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}


status_t i2c_readreg_max17261(I2C_Type *base,uint8_t slaveAddress, uint8_t regAddress)
{

	status_t reVal        = kStatus_Fail;
	/* Send master blocking data to slave */
		if (kStatus_Success == I2C_MasterStart(base, slaveAddress, kI2C_Write))
		{

			reVal = I2C_MasterWriteBlocking(base, &regAddress, 1, kI2C_TransferDefaultFlag);

			if (reVal != kStatus_Success)
			{
				return -1;
			}

			reVal = I2C_MasterRepeatedStart(base, slaveAddress, kI2C_Read);
			if (reVal != kStatus_Success)
			{
				return -1;
			}

			reVal =
					I2C_MasterReadBlocking(base, g_master_rxBuff, sizeof(g_master_rxBuff), kI2C_TransferDefaultFlag);
				if (reVal != kStatus_Success)
				{
					return -1;
				}

				for (uint32_t i = 0U; i <I2C_DATA_LENGTH ; i++)
				{
					if (i % 8 == 0)
					{
						PRINTF("\r\n");
					}
					{
						PRINTF("0x%2x  ", g_master_rxBuff[i]);
						g_slave_buff = g_master_rxBuff[1]<< 8 |g_master_rxBuff[0];

					}
				}
				PRINTF("\r\n\r\n");
				PRINTF("the result is %2x  ", g_slave_buff);

				if (regAddress == min_sys_voltage)
				{
					g_slave_buff *= 0.001;
					PRINTF("the voltage is %2x  ", g_slave_buff);
				}
		}

		return kStatus_Success;
}

status_t i2c_writereg_max17261(I2C_Type *base,uint8_t slaveAddress, uint8_t regAddress,uint32_t data)
{

	status_t reVal        = kStatus_Fail;
	uint8_t temp_msb,temp_lsb;
	/* Send master blocking data to slave */
		if (kStatus_Success == I2C_MasterStart(base, slaveAddress, kI2C_Write))
		{

			reVal = I2C_MasterWriteBlocking(base, &regAddress, 1, kI2C_TransferNoStopFlag);

			if (reVal != kStatus_Success)
			{
				return -1;
			}

			temp_msb = (data >>8) & 0xff;
			temp_lsb = (data & 0xff);

			printf("MSB:0x%X LSB:0x%X \r\n",temp_msb,temp_lsb );

			reVal = I2C_MasterWriteBlocking(base, &temp_lsb,1, kI2C_TransferNoStopFlag);
			if (reVal != kStatus_Success)
			{
				return -1;
			}
			reVal = I2C_MasterWriteBlocking(base, &temp_msb,1, kI2C_TransferNoStopFlag);
			if (reVal != kStatus_Success)
			{
				return -1;
			}

			reVal = I2C_MasterStop(EXAMPLE_I2C_MASTER);
				if (reVal != kStatus_Success)
				{
					return -1;
				}
			i2c_readreg_max17261(base,slaveAddress,regAddress);
		}

		return kStatus_Success;
}
#endif
int main(void) {
  	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	BOARD_InitDebugConsole();
#if i2c_polling
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
#endif
    printf("Hello World\n");

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
#if i2c_polling
	i2c_master_config_t masterConfig;
	status_t reVal        = kStatus_Fail;
	uint32_t dev_name = 0x21U;




	uint8_t full_caprep_register= 0x10;
	/* attach 12 MHz clock to FLEXCOMM0 (debug console) */
//	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

	/* attach 12 MHz clock to FLEXCOMM8 (I2C master) */
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);

	/* reset FLEXCOMM for I2C */
	RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);

	BOARD_InitPins();
	BOARD_InitBootPeripherals();
	BOARD_InitDebugConsole();
	PRINTF("\r\nI2C board2board polling example -- Master transfer.\r\n");


	I2C_MasterGetDefaultConfig(&masterConfig);

	/* Change the default baudrate configuration */
	masterConfig.baudRate_Bps = I2C_BAUDRATE;

	/* Initialize the I2C master peripheral */
	I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);


#if 0
	/* Send master blocking data to slave */
	if (kStatus_Success == I2C_MasterStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Write))
	{

		reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, &dev_name, 1, kI2C_TransferDefaultFlag);

		if (reVal != kStatus_Success)
		{
			return -1;
		}

		reVal = I2C_MasterRepeatedStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Read);
		if (reVal != kStatus_Success)
		{
			return -1;
		}

		reVal =
				I2C_MasterReadBlocking(EXAMPLE_I2C_MASTER, g_master_rxBuff, sizeof(g_master_rxBuff), kI2C_TransferDefaultFlag);
			if (reVal != kStatus_Success)
			{
				return -1;
			}

			for (uint32_t i = 0U; i <2 ; i++)
			{
				if (i % 8 == 0)
				{
					PRINTF("\r\n");
				}
				{
//					PRINTF("0x%2x  ", g_master_rxBuff[i]);
					g_slave_buff = g_master_rxBuff[1]<< 8 |g_master_rxBuff[0];

				}
			}
			PRINTF("\r\n\r\n");
			PRINTF("the version is %2x  ", g_slave_buff);
			PRINTF("\r\nEnd of I2C example .\r\n");
	}
#endif
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,dev_name);
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,min_sys_voltage);
/*	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,designcap_register);
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,full_caprep_register);
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,age_register);
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,vge_fuel_gauge);
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,open_ckt_vge_fuel_gauge);
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,tte_reg);
	i2c_readreg_max17261(EXAMPLE_I2C_MASTER,I2C_MASTER_SLAVE_ADDR_7BIT,ttf_reg);*/

	i2c_writereg_max17261(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, soft_wkup,0x0000);//clear cmd
	i2c_writereg_max17261(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, soft_wkup,0x0090);//softwkup cmd

	reVal = I2C_MasterStop(EXAMPLE_I2C_MASTER);
	if (reVal != kStatus_Success)
	{
		return -1;
	}
	PRINTF("\r\nEnd of I2C example .\r\n");
#endif
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}
