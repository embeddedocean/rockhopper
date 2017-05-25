/*
 * console.c
 *
 * Created: 5/24/2017 
 *  Author: chris
 */ 

#include <asf.h>
#include <string.h>
#include <stdio.h>
#include "console.h"

/**
 *  Configure UART console.
 */
void setup_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONSOLE_UART_BAUDRATE,
		.paritytype = UART_MR_PAR_NO
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	//pio_configure_pin_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}

int console_gets(char *str)
{
	uint32_t c;
	int n = 0;
	int go = 1;
	while( go ) {
		if(uart_is_rx_ready(UART0)) {
			usart_read(UART0, &c);
			if(c == 13) { // newline
				str[n] = 0;
				go = 0;
			} else {
				usart_write(UART0, c);
				str[n] = (char)c;
			}
			if(c==8 && n>0) n--; // backspace
			n++;
		}
	}
	return(n);
}

int console_prompt_int(char *prompt, int default_value, int timeout)
{
	uint32_t c;
	char str[64];
	int n = 0;
	int go = 1;
	int count = 0;
	int timeout_ms = timeout*1000;
	fprintf(stdout, "\r\n%s [%d]: ", prompt, default_value);
	while( go ) {
		if(uart_is_rx_ready(UART0)) {
			usart_read(UART0, &c);
			if(c == 13) { // newline
				str[n] = 0;
				go = 0;
			} else {
				usart_write(UART0, c);  // echo
				str[n] = (char)c;
			}
			if(c==8 && n>0) n--; // backspace
			n++;
		} else {
			delay_ms(1);
			count++;
		}
		if(count >= timeout_ms) go = 0;
	}
	
	int value = default_value;
	if(n > 1 && count < timeout_ms) value = atoi(str);
	
	//fprintf(stdout, "\r\n %d chars, %s, value = %d \r\n", n, str, value);
	fprintf(stdout, "\r\n");

	return(value);
}


int read_console_input(void)
{
	uint8_t str[128]; 
	uint8_t c;
	int n = 0;
	int go = 1;
	while( go ) {

		char delims[] = " =:,";
		char *tok;
		tok = strtok(str, delims);
		while(tok) {
			// sample rate		
			if(!strcmp("sampling_rate", tok)) {
				tok = strtok(NULL, delims);
				//sampling_rate = (uint32_t)atoi(tok);
				//fprintf(stdout, "\r\n set sample_rate = %d\r\n", sampling_rate);
			}
			// upload interval
			if(!strcmp("upload_interval_in_minutes", tok)) {
				tok = strtok(NULL, delims);
				//upload_interval_in_minutes = (uint32_t)atoi(tok);
				//fprintf(stdout, "\r\n set upload_interval_in_minutes = %d\r\n", upload_interval_in_minutes);
			}	
			tok = strtok(NULL, delims);
		}

	}
	
	return(1);
}

