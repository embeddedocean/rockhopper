//
// uart_queue.c  - ring-buffer queue functions
//
//

#include <asf.h>
#include <stdio.h>
#include <string.h>
#include <uart.h>
#include <delay.h>

#include "board.h"

#include "uart_queue.h"

struct  uart_queue {
	int tail;
	int head;
	uint8_t buffer[UART_QUEUE_SIZE];
	int overflow;
	int messages_available;
	bool termination_enabled;
	uint8_t termination;  // message termination char
	int id;
};

//  queue lists
static struct uart_queue uart_queue_list[USART_MAX_PORTS];
static Uart *uart_queue_port[USART_MAX_PORTS];
static uint8_t uart_echo[USART_MAX_PORTS];


//
// this function gets called after a new value has been read into queue->buffer[queue->tail]
// handles overflow differently than the above callback
//
static void uart_read_queue_callback(int port, uint8_t input_char)
{
	
	// get the queue for this uart module 
	struct uart_queue *queue = &uart_queue_list[port];
	if( queue == NULL ) {
		printf("uart_read_queue_callback: queue is not set.\r\n");
		return;
	}
	
	// find size of available space in queue	
	int size = (queue->tail + UART_QUEUE_SIZE - queue->head) % UART_QUEUE_SIZE;

	// check if the queue is full
	if( size == (UART_QUEUE_SIZE - 1) )  {
	
		// if buffer is full signal an overflow, but overwrite the buffer
		queue->overflow = 1;		
	
	} else {
		// there is room in the queue for new input
		queue->buffer[queue->tail] = input_char;
		
		queue->overflow = 0;
		// check for message termination character
		if( queue->termination_enabled && (queue->buffer[queue->tail] == queue->termination) ) {
			queue->messages_available++;  // termination found
			//printf("uart_read_queue_callback: %d messages in queue %d\r\n", queue->messages_available, queue->id);
		}
	
	}
	// increment tail
	queue->tail = (queue->tail + 1) % UART_QUEUE_SIZE;

}

// UART Interrupt handles
// It must be called UART0_Handler because that is the name defined in the sam4s***.h file
//
void UART0_Handler(void)
{
	/* Read USART Status. */
	uint8_t c;
	uint32_t status = uart_get_status(UART0);
	if (status & UART_SR_RXRDY) {
		uart_read(UART0, &c);
		if(uart_echo) uart_write(UART0, c);
		uart_read_queue_callback(0, c);
	}
}

void UART1_Handler(void)
{
	/* Read USART Status. */
	uint8_t c;
	//uint32_t status = uart_get_status(UART1);
	//if (status & UART_SR_RXRDY) {
	if (uart_is_rx_ready(UART1)) {
		uart_read(UART1, &c);
		if(uart_echo) uart_write(UART1, c);
		uart_read_queue_callback(1, c);
	}
}


enum status_code uart_read_message_queue(int port, uint8_t *buf, int len)
{
	enum status_code stat = STATUS_ERR_BUSY;
	
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return(ERR_INVALID_ARG);
	}
	
	// get the queue for this uart module
	struct uart_queue *queue = &uart_queue_list[port];
	if( queue == NULL ) {
		printf("uart_read_queue: queue is not set.\r\n");
		return(ERR_UNSUPPORTED_DEV);
	}

	//debug_printf("uart_read_queue: %d available in queue\r\n", queue->messages_available);

	// get the message termination char, which can be different for different uarts
	uint8_t termination = queue->termination;

	int n = 0;
	int go = 1;
	
	if( queue->termination_enabled && (queue->messages_available > 0) ) {

		// copy queue values into buffer until a termination char is found
		// or the queue is empty
		while( go ) {
			if ( queue->head == queue->tail ) {   // queue is empty
				//printf("uart_read_queue: queue %d empty\r\n", queue->id);
				queue->messages_available = 0; // Obviously, no messages if an empty queue.
				break;
			}
			uint8_t value = queue->buffer[queue->head];  // get next value
			queue->head = (queue->head + 1) % UART_QUEUE_SIZE; // increment queue head
			// break if termination char has been read  
			if( queue->termination_enabled && (value == termination) ) {
				//value = 0;  // replace the termination with null to terminate the string
				queue->messages_available--;
				go = 0;
			}
			buf[n++] = value;  // copy value into user buffer
			
			// break if max bytes have been read, leaving space for the terminating null
			if( n >= len-1 ) {
				go = 0;
			}
		}
		stat = STATUS_OK;
	}
	
	buf[n] = 0; // terminate the string with null
	
	//debug_printf("uart_read_queue: %d byte message read, %d left in queue %d\r\n", nrd, queue->messages_available, queue->id);

	//	if( queue->overflow && (queue->messages_available == 0) ) {
	if( queue->overflow ){
		printf("uart queue %d overflow\r\n", queue->id);
		stat |= ERR_IO_ERROR;
		//queue->overflow = 0; // So we only print the warning once per overflow.
	}
	
	return (stat);
}

enum status_code uart_read_queue(int port, uint8_t *buf, int len)
{
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return(ERR_INVALID_ARG);
	}
	
	enum status_code stat = STATUS_ERR_BUSY;

	// get the queue for this uart module
	struct uart_queue *queue = &uart_queue_list[port];
	if( queue == NULL ) {
		printf("uart_read_queue: queue is not set.\r\n");
		return(ERR_UNSUPPORTED_DEV);
	}

	//debug_printf("uart_read_queue: %d available in queue\r\n", queue->messages_available);
	
	int size = (queue->tail + UART_QUEUE_SIZE - queue->head) % UART_QUEUE_SIZE;
	if( size >= len) {
		for(int n = 0; n < len; n++) {
			uint8_t value = queue->buffer[queue->head];  // get next value
			queue->head = (queue->head + 1) % UART_QUEUE_SIZE; // increment queue head
			buf[n] = value;  // copy value into user buffer
		}
		stat = STATUS_OK;
	}

	//	if( queue->overflow && (queue->messages_available == 0) ) {
	if( queue->overflow ){
		printf("uart queue %d overflow\r\n", queue->id);
		stat |= ERR_IO_ERROR;
		//queue->overflow = 0; // So we only print the warning once per overflow.
	}
		
	return (stat);
}

enum status_code uart_start_queue(int port)
{
	enum status_code stat = STATUS_OK;
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return(ERR_INVALID_ARG);
	}

	Uart *uart = uart_queue_port[port];
	if( uart == NULL ) {
		printf("uart_start_queue: queue is not set.\r\n");
		return(ERR_UNSUPPORTED_DEV);
	}

	// enable interrupt
	uart_enable_interrupt(uart, UART_IER_RXRDY);
	uart_enable_rx(uart);

	if( port == 0) NVIC_EnableIRQ(UART0_IRQn);
	if( port == 1) NVIC_EnableIRQ(UART1_IRQn);
	
	return(stat);
}

enum status_code uart_stop_queue(int port)
{
	enum status_code stat = STATUS_OK;
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return(ERR_INVALID_ARG);
	}

	// get the queue for this uart module
	//struct uart_queue *queue = &uart_queue_list[port];
	Uart *uart = uart_queue_port[port];
	if( uart == NULL ) {
		printf("uart_stop_queue: queue is not set.\r\n");
		return(ERR_UNSUPPORTED_DEV);
	}

	// disable interrupt
	uart_disable_interrupt(uart, UART_IDR_RXRDY);

	if( port == 0) NVIC_DisableIRQ(UART0_IRQn);
	if( port == 1) NVIC_DisableIRQ(UART1_IRQn);

	return(stat);
}

// clear queue
void uart_clear_queue(int port)
{
	enum status_code stat = STATUS_OK;
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return;
	}
	// get the queue for this uart module
	struct uart_queue *queue = &uart_queue_list[port];
	if( queue == NULL ) {
		printf("uart_stop_queue: queue is not set.\r\n");
		return;
	}
	// clear the queue
	queue->tail = 0;
	queue->head = 0;
	queue->overflow = 0;
	queue->messages_available = 0;
	for(int m = 0; m < UART_QUEUE_SIZE; m++) queue->buffer[m] = 0;
}

// clear the queue list
void uart_clear_all_queues(void)
{
	for(int n = 0; n < USART_MAX_PORTS; n++) {
		uart_clear_queue(n);
	}
}


enum status_code uart_set_termination(int port, uint8_t term)
{
	enum status_code stat = STATUS_OK;	
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return(ERR_INVALID_ARG);
	}
	// get the queue for this uart module
	struct uart_queue *queue = &uart_queue_list[port];
	if( queue == NULL ) {
		printf("uart_stop_queue: queue is not set.\r\n");
		return(ERR_UNSUPPORTED_DEV);
	}

	if( term == USART_NO_TERMINATION ) {
		queue->termination_enabled = 0;
	} else {
		queue->termination_enabled = 1;
		queue->termination = term;
	}
	return(stat);
}

enum status_code uart_set_echo(int port, uint8_t on)
{
	enum status_code stat = STATUS_OK;
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return(ERR_INVALID_ARG);
	}
	uart_echo[port] = on;
	return(stat);
}

//enum status_code uart_configure(int port, uint32_t baud, int stdio)
enum status_code uart_configure_queue(int port, uint32_t baud)
{
	enum status_code stat = STATUS_OK;
	if( (port < 0) || (port >= USART_MAX_PORTS) ) {
		return(ERR_INVALID_ARG);
	}

	//const usart_serial_options_t uart_stdio_opts = {
	//	.baudrate = baud,
	//	.paritytype = UART_MR_PAR_NO
	//};
	
	sam_uart_opt_t uart_opts;
	uart_opts.ul_mck = sysclk_get_peripheral_hz();
	uart_opts.ul_baudrate = baud;
	uart_opts.ul_mode = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;

	/* Configure console UART. */
	if(port == 0) { // UART0
		sysclk_enable_peripheral_clock(ID_UART0);
		pio_configure_pin_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
		//if(stdio) stdio_serial_init(UART0, &uart_stdio_opts);
		//else uart_init(UART0, &uart_opts);
		uart_init(UART0, &uart_opts);
		uart_queue_port[0] = UART0;
	} else if(port == 1) {
		sysclk_enable_peripheral_clock(ID_UART1);
		pio_configure_pin_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);
		//if(stdio) stdio_serial_init(UART1, &uart_stdio_opts);
		//else uart_init(UART1, &uart_opts);
		uart_init(UART1, &uart_opts);
		uart_queue_port[1] = UART1;
	} else {
		stat = ERR_UNSUPPORTED_DEV;
	}

	// setup receiving queue
	uart_clear_queue(port);
	uart_set_termination(port, USART_NO_TERMINATION);
	uart_start_queue(port);
	uart_set_echo(port, 0);

	return(stat);
}

