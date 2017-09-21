/**
 * \file
 *
 * \brief SAM4S Xplained Pro board initialization
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <board.h>
#include <gpio.h>
#include <ioport.h>
#include <wdt.h>

void board_init(void)
{
	pmc_disable_all_periph_clk();

	pmc_enable_periph_clk(ID_SSC);
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_UART0);
	pmc_enable_periph_clk(ID_UART1);
	//pmc_enable_periph_clk(ID_SPI);

	//pmc_disable_periph_clk(uint32_t ul_id);
	//pmc_disable_udpck(void);
	
	wdt_disable(WDT);

	/* GPIO has been deprecated, the old code just keeps it for compatibility.
	 * In new designs IOPORT is used instead.
	 * Here IOPORT must be initialized for others to use before setting up IO.
	 */
	ioport_init();

	/* Initialize LED0, turned off */
	ioport_set_pin_dir(LED1_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED1_PIN, LED_OFF);
	ioport_set_pin_dir(LED2_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED2_PIN, LED_OFF);
	ioport_set_pin_dir(LED3_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED3_PIN, LED_OFF);

	/* Configure USART pinS */
	//pio_configure_pin_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
	//pio_configure_pin_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);

	/* Configure SSC pins */
	gpio_configure_pin(PIN_SSC_RD, PIN_SSC_RD_FLAGS);
	gpio_configure_pin(PIN_SSC_RF, PIN_SSC_RF_FLAGS);
	gpio_configure_pin(PIN_SSC_RK, PIN_SSC_RK_FLAGS);
	//gpio_configure_pin(PIN_SSC_TD, PIN_SSC_TD_FLAGS);
	//gpio_configure_pin(PIN_SSC_TF, PIN_SSC_TF_FLAGS);
	//gpio_configure_pin(PIN_SSC_TK, PIN_SSC_TK_FLAGS);
	
	/* Configure HSMCI pins */
	gpio_configure_pin(PIN_HSMCI_MCCDA_GPIO, PIN_HSMCI_MCCDA_FLAGS);
	gpio_configure_pin(PIN_HSMCI_MCCK_GPIO, PIN_HSMCI_MCCK_FLAGS);
	gpio_configure_pin(PIN_HSMCI_MCDA0_GPIO, PIN_HSMCI_MCDA0_FLAGS);
	gpio_configure_pin(PIN_HSMCI_MCDA1_GPIO, PIN_HSMCI_MCDA1_FLAGS);
	gpio_configure_pin(PIN_HSMCI_MCDA2_GPIO, PIN_HSMCI_MCDA2_FLAGS);
	gpio_configure_pin(PIN_HSMCI_MCDA3_GPIO, PIN_HSMCI_MCDA3_FLAGS);

	/* Configure SD/MMC card detect pin */
//	gpio_configure_pin(SD_MMC_0_CD_GPIO, SD_MMC_0_CD_FLAGS);
	
	/* Configure SPI pins */
//	gpio_configure_pin(SPI_MISO_GPIO, SPI_MISO_FLAGS);
//	gpio_configure_pin(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
//	gpio_configure_pin(SPI_SPCK_GPIO, SPI_SPCK_FLAGS);

	/**
	 * For NPCS 1, 2, and 3, different PINs can be used to access the same NPCS line.
	 * Depending on the application requirements, the default PIN may not be available.
	 * Hence a different PIN should be selected using the CONF_BOARD_SPI_NPCS_GPIO and
	 * CONF_BOARD_SPI_NPCS_FLAGS macros.
	 */
	gpio_configure_pin(SPI_NPCS0_GPIO, SPI_NPCS0_FLAGS);

	ioport_set_pin_dir(PI_ON_PIN, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(PI_ON_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_mode(PI_ON_PIN, IOPORT_MODE_OPEN_DRAIN);
	
	// power enable pins
	ioport_set_pin_dir(PIN_ENABLE_PI_5V, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ENABLE_PI_5V, 0);

	ioport_set_pin_dir(PIN_ENABLE_EXT_5V, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ENABLE_EXT_5V, 0);

	ioport_set_pin_dir(PIN_ENABLE_ADC_PWR, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 0);

	ioport_set_pin_dir(PIN_RESET_ADC_PDN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_RESET_ADC_PDN, 0);

	ioport_set_pin_dir(PIN_SELECT_SD, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD1);

	ioport_set_pin_dir(PIN_ENABLE_SD1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ENABLE_SD1, 0);

	ioport_set_pin_dir(PIN_ENABLE_SD2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ENABLE_SD2, 0);

//	ioport_set_pin_dir(SSC_ADC_BUF_PIN, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_dir(SD_CARD_WR_PIN, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_dir(SD_CARD_RD_PIN, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
//	ioport_set_pin_level(SD_CARD_WR_PIN, 0);
//	ioport_set_pin_level(SD_CARD_RD_PIN, 0);

}

//
// Set System clock defined as: MCLK = XTAL * MUL / DIV / SYSCLK_PRES
// XTAL = 10 MHz
// Use master clock prescaler to 2 (SYSCLK_PRES_2)
// - System clock 120 Mhz = 10Mhz * 24 / 1 / 2
// - System clock 100 Mhz = 10Mhz * 20 / 1 / 2
// - System clock 80 Mhz = 10Mhz * 16 / 1 / 2
// - System clock 60 Mhz = 10Mhz * 12 / 1 / 2
//
void resetup_system_clocks(uint32_t sclk)
{
	uint32_t xtal = BOARD_FREQ_MAINCK_XTAL;
	uint32_t div = 1;
	uint32_t mul = sclk * 2 * div / xtal;
	
	/* Set flash wait state to max in case the below clock switching. */
	system_init_flash(CHIP_FREQ_CPU_MAX);

	// CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_PLLACK
	struct pll_config pllcfg;
	pll_disable(0);
	pll_enable_source(PLL_SRC_MAINCK_XTAL);
	pll_config_init(&pllcfg, PLL_SRC_MAINCK_XTAL, div, mul);
	pll_enable(&pllcfg, 0);
	pll_wait_for_lock(0);
	pmc_switch_mck_to_pllack(SYSCLK_PRES_2); // prescaler 2
	
	//printf("Clock configuration: sclk = %lu, mul = %lu, div=%lu,\n\r", sclk, mul, div);
	
	/* Update the SystemFrequency variable */
	SystemCoreClockUpdate();

	/* Set a flash wait state depending on the new cpu frequency */
	system_init_flash(sysclk_get_cpu_hz());

	// Turns off the USB clock, does not switch off the PLL.
	//pmc_disable_udpck();

}
