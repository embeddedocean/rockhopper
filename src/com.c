/*
 * com.c
 *
 * Created: 5/24/2017 2:24:15 PM
 *  Author: chris
 */ 

#include "asf.h"
#include "board.h"
#include <string.h>

/** Size of the receive buffer used by the PDC, in bytes. */
#define COM_BUFFER_SIZE         100

/** USART PDC transfer type definition. */
#define COM_PDC_TRANSFER        1

/** USART FIFO transfer type definition. */
#define COM_BYTE_TRANSFER       0

/** Max buffer number. */
#define COM_MAX_BUF_NUM         1

/** All interrupt mask. */
#define COM_INTERRUPT_MASK  0xffffffff

/** Receive buffer. */
static uint8_t com_buffer[2][COM_BUFFER_SIZE];

/** Next Receive buffer. */
static uint8_t com_nextbuffer[2][COM_BUFFER_SIZE];

/** Current bytes in buffer. */
static uint32_t com_size_buffer = COM_BUFFER_SIZE;

/** Current bytes in next buffer. */
static uint32_t com_size_nextbuffer = COM_BUFFER_SIZE;

/** Byte mode read buffer. */
static uint32_t com_read_buffer = 0;

/** Current transfer mode. */
static uint8_t com_trans_mode = COM_PDC_TRANSFER;

/** Buffer number in use. */
static uint8_t com_buf_num = 0;

/** PDC data packet. */
pdc_packet_t com_packet, com_nextpacket;

/** Pointer to PDC register base. */
Pdc *com_pdc;

/** Flag of one transfer end. */
static uint8_t com_transend_flag = 0;

/**
 * Interrupt handler for USART. Echo the bytes received and start the
 * next receive.
 */
void USART_Handler(void)
{
	uint32_t status;

	/* Read USART Status. */
	status = usart_get_status(COM_UART);

	if (com_trans_mode == COM_PDC_TRANSFER) {
		/* Receive buffer is full. */
		if (status & US_CSR_RXBUFF) {

			/* Echo back buffer. */
			com_packet.ul_addr =
					(uint32_t)com_buffer[com_buf_num];
			com_packet.ul_size = com_size_buffer;
			com_nextpacket.ul_addr =
					(uint32_t)com_nextbuffer[com_buf_num];
			com_nextpacket.ul_size = com_size_nextbuffer;
			pdc_tx_init(com_pdc, &com_packet, &com_nextpacket);

			if (com_transend_flag) {
				com_size_buffer = COM_BUFFER_SIZE;
				com_size_nextbuffer = COM_BUFFER_SIZE;
				com_transend_flag = 0;
			}

			com_buf_num = COM_MAX_BUF_NUM - com_buf_num;

			/* Restart read on buffer. */
			com_packet.ul_addr = (uint32_t)com_buffer[com_buf_num];
			com_packet.ul_size = COM_BUFFER_SIZE;
			com_nextpacket.ul_addr = (uint32_t)com_nextbuffer[com_buf_num];
			com_nextpacket.ul_size = COM_BUFFER_SIZE;
			pdc_rx_init(com_pdc, &com_packet, &com_nextpacket);

		}
	} else {
		/* Transfer without PDC. */
		if (status & US_CSR_RXRDY) {
			usart_getchar(COM_UART, (uint32_t *)&com_read_buffer);
			usart_write(COM_UART, com_read_buffer);
		}
	}
}

/**
 * Configure USART in normal (serial rs232) mode, asynchronous,
 * 8 bits, 1 stop bit, no parity, 115200 bauds and enable its transmitter
 * and receiver.
 */
static void configure_usart(void)
{
	const sam_usart_opt_t usart_console_settings = {
		COM_UART_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		/* This field is only used in IrDA mode. */
		0
	};

	/* Enable the peripheral clock in the PMC. */
	sysclk_enable_peripheral_clock(COM_UART_ID);

	/* Configure USART in serial mode. */
	usart_init_rs232(COM_UART, &usart_console_settings, sysclk_get_peripheral_hz());

	/* Disable all the interrupts. */
	usart_disable_interrupt(COM_UART, COM_INTERRUPT_MASK);

	/* Enable the receiver and transmitter. */
	usart_enable_tx(COM_UART);
	usart_enable_rx(COM_UART);

	/* Configure and enable interrupt of USART. */
	NVIC_EnableIRQ(COM_USART_IRQn);
	
	/* Get board USART PDC base address. */
	com_pdc = usart_get_pdc_base(COM_UART);
	/* Enable receiver and transmitter. */
	pdc_enable_transfer(com_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Start receiving data and start timer. */
	com_packet.ul_addr = (uint32_t)com_buffer[com_buf_num];
	com_packet.ul_size = COM_BUFFER_SIZE;
	com_nextpacket.ul_addr = (uint32_t)com_nextbuffer[com_buf_num];
	com_nextpacket.ul_size = COM_BUFFER_SIZE;
	pdc_rx_init(com_pdc, &com_packet, &com_nex tpacket);

	com_trans_mode = COM_PDC_TRANSFER;

	usart_disable_interrupt(COM_UART, US_IDR_RXRDY);
	usart_enable_interrupt(COM_UART, US_IER_RXBUFF);
}

/**
 *  Configure UART for debug message output.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONSOLE_UART_BAUDRATE,
		.paritytype = UART_MR_PAR_NO
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}

/**
 * Reset the TX & RX, and clear the PDC counter.
 */
static void usart_clear(void)
{
	/* Reset and disable receiver & transmitter. */
	usart_reset_rx(COM_UART);
	usart_reset_tx(COM_UART);

	/* Clear PDC counter. */
	com_packet.ul_addr = 0;
	com_packet.ul_size = 0;
	com_nextpacket.ul_addr = 0;
	com_nextpacket.ul_size = 0;
	pdc_rx_init(com_pdc, &com_packet, &com_nextpacket);

	/* Enable receiver & transmitter. */
	usart_enable_tx(COM_UART);
	usart_enable_rx(COM_UART);
}

/**
 * Display main menu.
 */
static void display_main_menu(void)
{
	puts("-- Menu Choices for this example --\r\n"
			"-- s: Switch mode for USART between PDC and without PDC.--\r\n"
			"-- m: Display this menu again.--\r");
}
