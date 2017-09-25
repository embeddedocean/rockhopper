/**
 *
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "board.h"
#include "sd_card.h"
#include "spi_pdc.h"
#include "ak5552.h"
#include "console.h"
#include "rtc_time.h"
#include "uart_queue.h"
#include "gps.h"

COMPILER_WORD_ALIGNED uint8_t adc_buffer[AK5552_BUFFER_NBYTES];
uint8_t *adc_buffer_header = adc_buffer;  // header is at start of buffer
uint8_t *adc_buffer_data = &adc_buffer[AK5552_HEADER_NBYTES]; // data follows header

uint8_t Verbose = 1;

uint32_t sampling_rate = 312500;
//uint32_t sampling_rate = 156250;
uint32_t upload_interval_in_minutes = 60;

uint32_t number_buffers_per_upload = 512;

uint32_t sd_write_addr = SD_CARD_START_BLOCK;
uint32_t sd_read_addr = SD_CARD_START_BLOCK;
uint32_t sd_start_addr = SD_CARD_START_BLOCK;
uint32_t sd_end_addr =  0;
	
// nblocks_per_sector must be and even multiple of nblocks_per_buffer
uint32_t nblocks_per_upload = 1024 * AK5552_BUFFER_NBLOCKS;
uint32_t write_count = 0;

uint8_t shutdown_pi_after_upload = 1;

#define READING_ADC 0x01
#define WRITING_SD1 0x02
#define WRITING_SD2 0x04
#define _POWER_OFF 0x10
#define READY 0x20
#define BOOTING 0x40
#define SHUTTING_DOWN 0x80

//uint8_t sys_state = READING;
//uint8_t pi_state = POWER_OFF;

#define GPS_PORT 1
#define GPS_UART_BAUDRATE 38400

// local function prototypes
sd_card_t *select_sd_card(uint8_t card_num);

int main (void)
{
	int result = 0;

	sysclk_init();	
	board_init();

	/* Initialize the console uart */
	setup_console();

	ioport_set_pin_level(LED2_PIN, LED_ON);

	/* Output header string  */
    fprintf(stdout, "-- RockHopper V2.0 --\r\n" );
	fprintf(stdout, "-- Compiled: "__DATE__ " "__TIME__ " --\r\n");

	// configuration prompt
	int timeout = 10;
	fprintf(stdout, "\r\nEnter configuration or hit Enter to accept default values\r\n" ); 
	fprintf(stdout, "Prompt will timeout in %d seconds.\r\n", timeout );

	// read sampling rate
	sampling_rate = (uint32_t)console_prompt_int("Enter sampling rate in Hz", sampling_rate, timeout);

	// read upload interval
	upload_interval_in_minutes = (uint32_t)console_prompt_int("Enter upload interval in minutes", upload_interval_in_minutes, timeout);

	// read upload interval
	shutdown_pi_after_upload = (uint8_t)console_prompt_int("Shutdown PI after upload (1=yes, 0=no)", shutdown_pi_after_upload, timeout);

	// GPS read and set date and time
	rtc_time_t rtc = {.century=20, .year=17, .month=5, .day=26, .hour=1, .minute=0, .second=0};
	ioport_set_pin_level(LED1_PIN, LED_ON);
	uart_configure_queue(GPS_PORT, GPS_UART_BAUDRATE);
	uart_set_termination(GPS_PORT, USART_NEWLINE_TERMINATION);  // * termination
	uart_start_queue(GPS_PORT);
	int gps_timeout = 60000; // 60 seconds
	if( console_prompt_int("Set time with GPS connected to UART1 (1=yes, 0=no)", 0, timeout) ) {
		fprintf(stdout, "\r\nLooking for GPS Time on UART1 (timeout in %d seconds)\r\n", gps_timeout );
		while( gps_timeout ) {
			ioport_toggle_pin_level(LED1_PIN);
			uint8_t str[128];
			enum status_code status = uart_read_message_queue(GPS_PORT, str, 128);
			// if a ZDA message is found, use it to set the time
			if( status == STATUS_OK && gps_parse_zda(str, &rtc)) {
				// sometimes a bad time is found, so keep looking
				if(rtc.year > 06) {
					fprintf(stdout, "Found GPS message: %s\r\n", str);
					break;
				}
			} else {
				gps_timeout--;
				delay_ms(1);
			}
		}
		uart_stop_queue(GPS_PORT);
	} else {
		gps_timeout = 0;
	}

	// if no GPS message found 
	if( gps_timeout == 0) {
		fprintf(stdout, "No GPS Time message found.\r\n" );
		fprintf(stdout, "Enter time manually using epoch (or Linux) time.\r\n" );
		uint32_t epoch = rtc_time_to_epoch(&rtc);
		epoch = (uint32_t)console_prompt_int("Enter epoch time", epoch, timeout);
		epoch_to_rtc_time(&rtc, epoch);
		fprintf(stdout, "\r\n");	
	}

	// set the system RTC
	setup_rtc(&rtc);
	
	/* Initialize SD MMC stack */
	sd_mmc_init();

	// active sd card pointer
	sd_card_t *active_sd_card = NULL;

	// Select SD2
	active_sd_card = select_sd_card(2);
	// Show SD2 card info.
	sd_card_get_info(active_sd_card);

	// Select SD1
	select_sd_card(1);
	// Show SD1 card info.
	sd_card_get_info(active_sd_card);

	uint8_t sd_card_number = 1;

	// power up and initialize adc	
	ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 1);
	ioport_set_pin_level(PIN_RESET_ADC_PDN, 1);
	//	ioport_set_pin_level(PIN_PREAMP_SHDN, 1);

	ak5552_init(SSC_AUDIO_MONO_LEFT, 24, AK5552_INT_MCLK, AK5552_MCLK_32FS, &sampling_rate);
//	ak5552_init(SSC_AUDIO_MONO_LEFT, 24, AK5552_EXT_MCLK, AK5552_MCLK_32FS, &sampling_rate);

	printf("ADC Initialized: fs = %lu\n\r", sampling_rate);

	printf("Init SSC DMA\n\r");
	uint16_t nsamps = ak5552_init_dma();

	float buffer_duration =  (float)AK5552_NUM_SAMPLES / (float)sampling_rate;
	number_buffers_per_upload = (uint32_t)(60.0f * (float)upload_interval_in_minutes / buffer_duration);
	nblocks_per_upload = number_buffers_per_upload * AK5552_BUFFER_NBLOCKS;
	sd_end_addr = active_sd_card->last_block;
	sd_start_addr = active_sd_card->first_block;

	if(Verbose) {
		printf("sd start_addr %lu \n\r", sd_start_addr);
		printf("sd end_addr %lu \n\r", sd_end_addr);
		printf("nblocks_per_adc_buffer %lu \n\r", AK5552_BUFFER_NBLOCKS);
		printf("nblocks_per_upload %lu \n\r", nblocks_per_upload);
	}

	//printf("buffer_duration %f \n\r", buffer_duration);
	printf("sampling_rate %lu \n\r", sampling_rate);
 	printf("number_buffers_per_upload %lu \n\r", number_buffers_per_upload);

	ioport_set_pin_level(PIN_ENABLE_PI_5V, 0);
	ioport_set_pin_level(PIN_ENABLE_EXT_5V, 0);
	
	ioport_set_pin_level(LED1_PIN, LED_ON);
	delay_ms(1000);
	
	// start adc conversions and dma
	ak5552_start_dma();
	ak5552_start_conversion();
	
	ioport_set_pin_level(LED2_PIN, LED_OFF);

	printf("Start read loop \n\r");
	int go = 1;

	// throw away the first adc buffer
	while( ak5552_read_dma(adc_buffer_header, adc_buffer_data) == 0 ) {}

	while (go) {
		
		nsamps = ak5552_read_dma(adc_buffer_header, adc_buffer_data);

		if( nsamps == AK5552_NUM_SAMPLES ) {

			ioport_toggle_pin_level(LED1_PIN);

			result = sd_card_write_raw(active_sd_card, adc_buffer, AK5552_BUFFER_NBLOCKS, sd_write_addr);

			if( result == SD_MMC_OK ) {
				sd_write_addr +=  AK5552_BUFFER_NBLOCKS;
				if(sd_write_addr >= sd_end_addr) sd_write_addr = sd_start_addr;
				write_count += AK5552_BUFFER_NBLOCKS;
			} else {
				printf("sd_card_write_raw: 0x%x.\n\r", result);
			}
			
			if( write_count == nblocks_per_upload ) {
				write_count = 0;  // reset counter
				// switch sd cards
				if(sd_card_number == 1 ) {
					sd_card_number = 2;
					active_sd_card = select_sd_card(2);
				} else {
					sd_card_number = 1;
					active_sd_card = select_sd_card(1);
				}
				// start PI ....
			}
		}
		
		// sleep between dma buffers, the next dma interrupt will wake from sleep
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
				
	}

}

// 
// select the sd card and disable the other
// 
#define sd_startup_delay_msec  1  

// sd card objects
sd_card_t sd_card1;
sd_card_t sd_card2;

sd_card_t *select_sd_card(uint8_t card_num)
{
	sd_card_t *sd = NULL;
	
	if(card_num == 2) {
		printf("Enabling SD2 Card: ");
		// select and enable SD2
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_ENABLE);
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_DISABLE);
		if(sd_startup_delay_msec) delay_ms(sd_startup_delay_msec);
		ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD2);
		if(sd_card_init(&sd_card2) == SD_MMC_OK) {
			sd = &sd_card2;
			printf("done\n\r");
		} else {
			printf("Failed\n\r");
		}
	} else if(card_num == 1) {
		// select and enable SD1
		printf("Enabling SD1 Card: ");
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_ENABLE);
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_DISABLE);
		if(sd_startup_delay_msec) delay_ms(sd_startup_delay_msec);
		ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD1);
		if(sd_card_init(&sd_card1) == SD_MMC_OK) {
			sd = &sd_card1;
			printf("done\n\r");
		} else {
			printf("Failed\n\r");
		}
	} else {
		printf("Unknown SD number: select SD Card Failed\n\r");
	}

	return(sd);
}

static inline uint8_t checksum(uint8_t *data, uint32_t len)
{
	uint32_t nbytes = len;
	uint8_t *buf = data;
	uint8_t sum = 0;
	while(nbytes--) sum += *buf++;
	return(sum);
}



