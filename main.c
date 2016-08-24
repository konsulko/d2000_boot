/*
 * Copyright (c) 2015, 2016 Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include "qm_flash.h"
#include "qm_uart.h"
#include "qm_gpio.h"
#include "qm_pinmux.h"
#include "qm_scss.h"

/*
 * The flash controller segments its memory into pages of 2KB in size
 * (multiples of 0x800). Writes don't cross over to other pages.
 *
 * For Quark Microcontroller D2000, there is 1 flash controller.
 * Controller 0:
 * |  Component                 | Size          | System start address
 * |  System ROM		| 8KB	 	| 0x00000000
 * |  System Flash		| 32KB		| 0x00180000
 * |  Data region		| 4KB		| 0x00200000
 *
 * For Quark SE SOC, there are 2 flash controllers.
 * Controller 0:
 * |  Component                 | Size          | System start address
 * |  System Flash		| 192KB		| 0x40000000
 * |  System ROM		| 8KB           | 0xFFFFE000
 *
 * Controller 1:
 * |  Component                 | Size          | System start address
 * |  System Flash		| 192KB		| 0x40030000
 */

#define NUM_DATA_WORDS (0x03)
#define WR_FLASH_ADDR (0x101C)
#define MASS_ERASE_INCLUDE_ROM (0x00)
#define US_COUNT (0x20)
#define WAIT_STATES (0x01)

#define VERSION_HIGH	'0'
#define VERSION_LOW	'1'

#define DEVTYPE		0x73 /* XXX: placeholder */

#define SIG_BYTE1	0xDE
#define SIG_BYTE2	0xAD
#define SIG_BYTE3	0xBE

/* This buffer must be at least QM_FLASH_PAGE_SIZE. Practically, this buffer */
/* may be shared with other buffers to save space. */
static uint32_t flash_page_buffer[QM_FLASH_PAGE_SIZE];

/* UART 0 */
static const uint32_t qmu = QM_UART_0;

static void sendchar(uint8_t data)
{
	while (qm_uart_get_status(qmu) & QM_UART_TX_BUSY); 
	qm_uart_write_non_block(qmu, data);
}

static uint8_t recvchar(void)
{
	uint8_t data;

	while (qm_uart_get_status(qmu) & QM_UART_RX_BUSY);
	qm_uart_read(qmu, &data);
	return data;
}

static void erase_flash(void)
{
	/* erase only system section (bootloader protection) */
	uint8_t page = 6;

	for (; page < 16; page++) /* XXX: hardcoded */
		qm_flash_page_erase(QM_FLASH_0, QM_FLASH_REGION_SYS, page);
}

static void recv_buffer(size_t size)
{
	size_t cnt;
	uint8_t *tmp = (uint8_t *)flash_page_buffer;

	for (cnt = 0; cnt < sizeof(flash_page_buffer); cnt++) {
		*tmp++ = (cnt < size) ? recvchar() : 0xFF;
	}
}

static uint32_t _assert(int expr)
{
	if (!expr) {
		QM_PUTS("assertion failed");
		return 0;
	}
	return 1;
}

static uint32_t write_flash_page(uint32_t address, size_t size)
{
	int i = 0;

	_assert(!(size % 4));
	while (size > 0) {	
		qm_flash_word_write(QM_FLASH_0, QM_FLASH_REGION_SYS, address,
				flash_page_buffer[i]);
		i++; size -= 4; address += 4;
	}
	return address;
}

static uint32_t write_eeprom_page(uint32_t address, size_t size)
{
	int i = 0;

	_assert(!(size % 4));
	while (size > 0) {	
		qm_flash_word_write(QM_FLASH_0, QM_FLASH_REGION_OTP, address,
				flash_page_buffer[i]);
		i++; size -= 4; address += 4;
	}
	return address;
}

static uint16_t read_flash_page(uint32_t address, size_t size)
{
	uint32_t data;
	uint32_t raddr = address + QM_FLASH_REGION_SYS_0_BASE;

	_assert(!(size % 4));
	do {
		data = *(volatile uint32_t *)raddr;
		sendchar(data & 0xFF);
		sendchar((data & 0xFF00) >> 8);
		sendchar((data & 0xFF0000) >> 16);
		sendchar((data & 0xFFFFFF00) >> 24);

		raddr += 4;
		size -= 4;
	} while (size > 0);

	return raddr - QM_FLASH_REGION_SYS_0_BASE;
}

static uint16_t read_eeprom_page(uint16_t address, size_t size)
{
	uint32_t data;

	_assert(!(size % 4));
	do {
		uint32_t _addr = address;
		data = *(volatile uint32_t *)_addr;
		sendchar(data & 0xFF);
		sendchar((data & 0xFF00) >> 8);
		sendchar((data & 0xFF0000) >> 16);
		sendchar((data & 0xFFFFFF00) >> 24);

		address += 4;
		size -= 4;
	} while (size > 0);

	return address;
}


static void send_boot(void)
{
	sendchar('A');
	sendchar('V');
	sendchar('R');
	sendchar('B');
	sendchar('O');
	sendchar('O');
	sendchar('T');
}

static void (*jump_to_app)(void) = (void *)QM_FLASH_REGION_SYS_0_BASE + 0x2f00;

int main(void)
{
	uint16_t address = 0;
	uint8_t device = 0, val;

	qm_uart_config_t cfg;
	qm_gpio_port_config_t pcfg;

	/* Use PWM0 to check if we should go to loader mode */
	qm_pmux_select(19, QM_PMUX_FN_1);
	qm_gpio_get_config(QM_GPIO_0, &pcfg);
	pcfg.direction &= ~BIT(19);
	qm_gpio_set_config(QM_GPIO_0, &pcfg);

	if (qm_gpio_read_pin(QM_GPIO_0, 19)) {
		jump_to_app();
		return 0;
	}

        /* 
	 * Set divisors to yield 115200bps baud rate.
         * Sysclk is set by boot ROM to hybrid osc in crystal mode (32MHz),
         * peripheral clock divisor set to 1.
         */
		qm_uart_get_config(QM_UART_0, &cfg);

		cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 17, 6);
        cfg.line_control = QM_UART_LC_8N1;
        cfg.hw_fc = false;

        qm_pmux_select(QM_PIN_ID_12, QM_PMUX_FN_2);
        qm_pmux_select(QM_PIN_ID_13, QM_PMUX_FN_2);
        qm_pmux_input_en(QM_PIN_ID_13, true);

        clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_UARTA_REGISTER);

	if (qmu == QM_UART_1) {
		qm_pmux_select(QM_PIN_ID_20, QM_PMUX_FN_2);
		qm_pmux_select(QM_PIN_ID_21, QM_PMUX_FN_2);
		qm_pmux_input_en(QM_PIN_ID_21, true);
		clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_UARTB_REGISTER);
	}

        qm_uart_set_config(qmu, &cfg);
        QM_PUTS("Ready.");

	for(;;) {
		val = recvchar();
		if (val == 'a') {		/* auto-increment */
			sendchar('Y');
		} else if (val == 'A') {	/* read address 8 MSB */
			address = recvchar();
			address = (address<<8) | recvchar();
			sendchar('\r');
		} else if (val == 'b') {	/* Buffer load support */
			sendchar('Y');
			sendchar((sizeof(flash_page_buffer) >> 8) & 0xFF);
			sendchar(sizeof(flash_page_buffer) & 0xFF);
		} else if (val == 'B') {	/* start buffer load */
			size_t size;
			size = recvchar() << 8;
			size |= recvchar();	/* Buffer size loaded */
			val = recvchar();	/* memory type ('E' or 'F') */
			recv_buffer(size);

			if (device == DEVTYPE) {
				if (val == 'F') {
					address = write_flash_page(address, size);
				} else if (val == 'E') {
					address = write_eeprom_page(address, size);
				}
				sendchar('\r');
			} else {
				sendchar(0);
			}
		} else if (val == 'g') { 	/* Block read */ 
			size_t size = recvchar() << 8;
			size |= recvchar();
			val = recvchar();

			if (val == 'F') {
				address = read_flash_page(address, size);
			} else if (val == 'E') {
				address = read_eeprom_page(address, size);
			}
 		} else if (val == 'e') {	/* chip erase */
			if (device == DEVTYPE) {
				erase_flash();
			}
			sendchar('\r');
		} else if (val == 'E') {	/* Exit upgrade */
			sendchar('\r');
		} else if (val == 'P') {	/* Enter programming mode */
			sendchar('\r');
		} else if (val == 'L') {	/* Leave programming mode */
			sendchar('\r');
		} else if (val == 'p') {	/* Programmer type == serial */
			sendchar('S');
		} else if (val == 't') {	/* Device type */
			sendchar(DEVTYPE);
			sendchar(0);
		} else if ((val == 'x') || (val == 'y')) { /* TODO: LED set/clear */
			recvchar();
			sendchar('\r');
		} else if (val == 'T') {	/* Set device */
			device = recvchar();
			sendchar('\r');
		} else if (val == 'S') {	/* SW identifier */
			send_boot();
		} else if (val == 'V') {	/* SW version */
			sendchar(VERSION_HIGH);
			sendchar(VERSION_LOW);
		} else if (val == 's') {	/* Signature bytes */
			sendchar(SIG_BYTE1);
			sendchar(SIG_BYTE2);
			sendchar(SIG_BYTE3);
		} else if(val != 0x1b) {	/* ESC */
			sendchar('?');
		}
	}
	return 0;
}
