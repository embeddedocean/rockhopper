/**
 * \file
 *
 * \brief User Interface
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
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
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include "usb_ui.h"

static volatile bool usb_vendor_enabled = false;

extern uint8_t *usb_buffer;

//! \brief Initializes the user interface
void usb_ui_init(void)
{
	// Initialize LEDs
	ioport_set_pin_level(LED_0_PIN, !LED_0_ACTIVE);
	//LED_Off(LED0_GPIO);
	//LED_Off(LED1_GPIO);
}

//! \brief Enters the user interface in power down mode
void usb_ui_powerdown(void)
{
	ioport_set_pin_level(LED_0_PIN, !LED_0_ACTIVE);
	//LED_Off(LED0_GPIO);
	//LED_Off(LED1_GPIO);
}

//! \brief Exits the user interface of power down mode
void usb_ui_wakeup(void)
{
	ioport_set_pin_level(LED_0_PIN, LED_0_ACTIVE);
	//LED_On(LED0_GPIO);
}

/*! \brief Notify the state of loopback
 * It is called when a the loopback is started and stopped.
 *
 * \param b_started    loopback started if true, else stopped
 */
void usb_ui_loop_back_state(bool b_started)
{
	if (b_started) {
	//	LED_On(LED1_GPIO);
		ioport_set_pin_level(LED_0_PIN, LED_0_ACTIVE);
	} else {
		ioport_set_pin_level(LED_0_PIN, !LED_0_ACTIVE);
	//	LED_Off(LED1_GPIO);
	}
}

/*! \brief This process is called each 1ms
 * It is called only if the USB interface is enabled.
 *
 * \param framenumber  Current frame number
 */
void usb_ui_process(uint16_t framenumber)
{
	if ((framenumber % 1000) == 0) {
		ioport_set_pin_level(LED_0_PIN, LED_0_ACTIVE);
		//LED_On(LED0_GPIO);
	}
	if ((framenumber % 1000) == 500) {
		ioport_set_pin_level(LED_0_PIN, !LED_0_ACTIVE);
		//LED_Off(LED0_GPIO);
	}
}

/**
 * \defgroup UI User Interface
 *
 * Human interface on SAM4S Xplained:
 * - LED blinks when USB host has checked and enabled vendor interface
 * - LED is on when loopback is running
 */

void usb_callback_vbus_action(bool b_vbus_high)
{
	
}

void usb_callback_suspend_action(void)
{
	usb_ui_powerdown();
}

void usb_callback_resume_action(void)
{
	usb_ui_wakeup();
}

void usb_callback_sof_action(void)
{
	if (!usb_vendor_enabled) return;
	usb_ui_process(udd_get_frame_number());
}

bool usb_callback_vendor_enable(void)
{
	usb_vendor_enabled = true;
	// Start data reception on OUT endpoints
	usb_vendor_bulk_in_received(UDD_EP_TRANSFER_OK, 0, 0);
	return true;
}

void usb_callback_vendor_disable(void)
{
	usb_vendor_enabled = false;
}

bool usb_vendor_setup_out_received(void)
{
	usb_ui_loop_back_state(true);
	udd_g_ctrlreq.payload = usb_buffer;
	udd_g_ctrlreq.payload_size = min( udd_g_ctrlreq.req.wLength, sizeof(usb_buffer));
	return true;
}

bool usb_vendor_setup_in_received(void)
{
	usb_ui_loop_back_state(false);
	udd_g_ctrlreq.payload = usb_buffer;
	udd_g_ctrlreq.payload_size = min( udd_g_ctrlreq.req.wLength, sizeof(usb_buffer) );
	return true;
}


void usb_vendor_bulk_in_received(udd_ep_status_t status, iram_size_t nb_transfered, udd_ep_id_t ep)
{
	UNUSED(nb_transfered);
	UNUSED(ep);
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Transfer aborted, then stop loopback
	}
	usb_ui_loop_back_state(false);
	// Wait a full buffer
	udi_vendor_bulk_out_run(usb_buffer, sizeof(usb_buffer), usb_vendor_bulk_out_received);
}

void usb_vendor_bulk_out_received(udd_ep_status_t status, iram_size_t nb_transfered, udd_ep_id_t ep)
{
	UNUSED(ep);
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Transfer aborted, then stop loopback
	}
	usb_ui_loop_back_state(true);
	// Send on IN endpoint the data received on endpoint OUT
	udi_vendor_bulk_in_run(usb_buffer, nb_transfered, usb_vendor_bulk_in_received);
}

