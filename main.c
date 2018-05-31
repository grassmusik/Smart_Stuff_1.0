/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2016-03-11 11:46:02 -0600 (Fri, 11 Mar 2016) $
 * $Revision: 21838 $
 *
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   SPIM example using the MX25.
 * @details Uses the MX25 on the EvKit to show the SPIM. Erases, writes, and then
 *          verifies the data. BUFF_SIZE, MX25_BAUD, MX25_ADDR, and MX25_SPIM_WIDTH
 *          can be changed to alter the communication between the devices. Refer
 *          to the schematic for the pinout and ensure that there are no switches
 *          blocking the communication to the MX25.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_config.h"
#include "mxc_sys.h"
#include "max3263x.h"
#include "gpio.h"
#include "lp.h"
#include "board.h"
#include "clkman.h"
#include "ioman.h"
#include "spim.h"
#include "mx25.h"
#include "StartUp.h"


/***** Definitions *****/
#define SPIM0A_PORT_0	PORT_0
#define SPIM0A_PIN_0	PIN_0
#define SPIM0A_PIN_1	PIN_1
#define SPIM0A_PIN_2	PIN_2
#define SPIM0A_PIN_3	PIN_3
#define SPIM0A_PIN_4	PIN_4
#define SPIM0A_PIN_5	PIN_5
#define SPIM0A_PIN_6	PIN_6
#define SPIM0A_PIN_7	PIN_7

#define SPIM2A_PORT_1	PORT_1
#define SPIM2A_PIN_0	PIN_0
#define SPIM2A_PIN_1	PIN_1
#define SPIM2A_PIN_2	PIN_2
#define SPIM2A_PIN_3	PIN_3
#define SPIM2A_PIN_4	PIN_4
#define SPIM2A_PIN_5	PIN_5
#define SPIM2A_PIN_6	PIN_6
#define SPIM2A_PIN_7	PIN_7

#define SPIM2C_PORT_2	PORT_2
#define SPIM2C_PIN_0	PIN_0
#define SPIM2C_PIN_1	PIN_1
#define SPIM2C_PIN_2	PIN_2
#define SPIM2C_PIN_3	PIN_3
#define SPIM2C_PIN_4	PIN_4
#define SPIM2C_PIN_5	PIN_5
#define SPIM2C_PIN_6	PIN_6
#define SPIM2C_PIN_7	PIN_7

#define SPI_BAUD           	48000000    // 48 MHz maximum, 20 kHz minimum
#define MX25_ADDR           0x0
#define MX25_SPIM_WIDTH     SPIM_WIDTH_4
#define MX25_EXP_ID         0xc22538

#define BUFF_SIZE           512
#define _port 				3
#define _pin  				8

/***** Globals *****/

gpio_cfg_t 	SPIM0A_0_0,
			SPIM0A_0_1,
			SPIM0A_0_2,
			SPIM0A_0_3,
			SPIM0A_0_4,
			SPIM0A_0_5,
			SPIM0A_0_6,
			SPIM0A_0_7,
			SPIM2A_1_0,
			SPIM2A_1_1,
			SPIM2A_1_2,
			SPIM2A_1_3,
			SPIM2A_1_4,
			SPIM2A_1_5,
			SPIM2A_1_6,
			SPIM2A_1_7,
			SPIM2C_2_0,
			SPIM2C_2_1,
			SPIM2C_2_2,
			SPIM2C_2_3,
			SPIM2C_2_4,
			SPIM2C_2_5,
			SPIM2C_2_6,
			SPIM2C_2_7;

/***** Functions *****/
void pause(void)
{
	for(unsigned int i=0; i<5000000; i++)
		{
		_NOP();
		}
}

/******************************************************************************/
int Setup(error)
{
	LP_EnterLP3();
	gpio_Init();

	// Initialize the SPIM0A
	    spim_cfg_t SPIM0A;
	    SPIM0A.mode = 0;
	    SPIM0A.ssel_pol = 4;
	    SPIM0A.baud = SPI_BAUD;

	    sys_cfg_spim_t sys_SPIM0A_cfg;

	// Initialize the SPIM2A
		spim_cfg_t SPIM2A;
		SPIM2A.mode = 0;
		SPIM2A.ssel_pol = 4;
		SPIM2A.baud = SPI_BAUD;

		sys_cfg_spim_t sys_SPIM2A_cfg;

	// Initialize the SPIM2C
		spim_cfg_t SPIM2C;
		SPIM2C.mode = 0;
		SPIM2C.ssel_pol = 4;
		SPIM2C.baud = SPI_BAUD;

		sys_cfg_spim_t sys_SPIM2C_cfg;

	// SPIM0A IO Config                  core I/O, ss0, ss1, ss2, ss3, ss4, quad, fast I/O
	    sys_SPIM0A_cfg.io_cfg = (ioman_cfg_t)IOMAN_SPIM0(1, 1, 1, 1, 1, 1, 0, 1);
	    sys_SPIM0A_cfg.clk_scale = CLKMAN_SCALE_AUTO;

		if((error = SPIM_Init(MXC_SPIM1, &SPIM0A, &sys_SPIM0A_cfg)) != E_NO_ERROR) {
			printf("Error initializing SPIM %d\n", error);
			while(1) {}
		} else {
			printf("SPIM0A Initialized\n");
		}

	// SPIM2A IO Config                  core I/O, ss0, ss1, ss2, ss3, ss4, quad, fast I/O
		sys_SPIM2A_cfg.io_cfg = (ioman_cfg_t)IOMAN_SPIM0(1, 1, 1, 1, 1, 1, 0, 1);
		sys_SPIM2A_cfg.clk_scale = CLKMAN_SCALE_AUTO;

		if((error = SPIM_Init(MXC_SPIM1, &SPIM2A, &sys_SPIM2A_cfg)) != E_NO_ERROR) {
			printf("Error initializing SPIM %d\n", error);
			while(1) {}
		} else {
			printf("SPIM2A Initialized\n");
		}

	// SPIM2C IO Config                  core I/O, ss0, ss1, ss2, ss3, ss4, quad, fast I/O
		sys_SPIM2C_cfg.io_cfg = (ioman_cfg_t)IOMAN_SPIM0(1, 1, 1, 1, 1, 1, 0, 1);
		sys_SPIM2C_cfg.clk_scale = CLKMAN_SCALE_AUTO;

		if((error = SPIM_Init(MXC_SPIM1, &SPIM2C, &sys_SPIM2C_cfg)) != E_NO_ERROR) {
			printf("Error initializing SPIM %d\n", error);
			while(1) {}
		} else {
			printf("SPIM2C Initialized\n");
		}

}

/******************************************************************************/
int main(void)
{
    int error, i;
    uint8_t txdata[BUFF_SIZE];
    uint8_t rxdata[BUFF_SIZE];

    // Initialize the data buffers
    for(i = 0; i < BUFF_SIZE; i++) {
        txdata[i] = i;
    }
    memset(rxdata, 0x0, BUFF_SIZE);







    // Initialize the MX25
    MX25_init(MXC_SPIM1, 0);
    MX25_reset();
    printf("MX25 Initialized\n");

    // Get the ID of the MX25
    uint32_t id = MX25_ID();
    if(id != MX25_EXP_ID) {
        printf("Error verifying MX25 ID: 0x%x\n", id);
        while(1) {}
    } else {
        printf("MX25 ID verified\n");
    }

    // Erase the test sector
    MX25_erase(MX25_ADDR, MX25_ERASE_4K);

    // Enable Quad mode if we're using quad
    if(MX25_SPIM_WIDTH == SPIM_WIDTH_4) {
        if(MX25_quad(1) != E_NO_ERROR) {
            printf("Error enabling quad mode\n");
        }
    } else {
        if(MX25_quad(0) != E_NO_ERROR) {
            printf("Error disabling quad mode\n");
        }
    }

    // Program the MX25
    MX25_program_page(MX25_ADDR, txdata, BUFF_SIZE, MX25_SPIM_WIDTH);

    // Read back the memory
    MX25_read(MX25_ADDR, rxdata, BUFF_SIZE, MX25_SPIM_WIDTH);

    if((error = memcmp(rxdata, txdata, BUFF_SIZE)) != 0) {
        printf("Error verifying MX25 memory %d\n", error);
    } else {
        printf("MX25 memory verified\n");
    }

    while(1) {}
}
