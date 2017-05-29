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

#define SPI_DATA_NBLOCKS (6)
#define SPI_DATA_NBYTES (SPI_DATA_NBLOCKS * SD_MMC_BLOCK_SIZE)

#define SPI_HEADER_NBYTES 12
#define SPI_BUFFER_NBYTES (SPI_HEADER_NBYTES + SPI_DATA_NBYTES)

COMPILER_WORD_ALIGNED uint8_t spi_buffer[SPI_BUFFER_NBYTES];
uint8_t *spi_buffer_header = spi_buffer;
uint8_t *spi_buffer_data = &spi_buffer[SPI_HEADER_NBYTES];

uint8_t Verbose = 1;

uint32_t sampling_rate = 312500;
//uint32_t sampling_rate = 156250;
uint32_t upload_interval_in_minutes = 60;

uint32_t number_buffers_per_upload = 512;

int read_data(void);
int upload_data(void);
void update_spi_header(uint32_t count);

sd_card_info_t sd_card;
uint32_t sd_write_addr = SD_CARD_START_BLOCK;
uint32_t sd_read_addr = SD_CARD_START_BLOCK;
uint32_t sd_start_addr = SD_CARD_START_BLOCK;
uint32_t sd_end_addr =  0;
	
// nblocks_per_sector must be and even multiple of nblocks_per_buffer
uint32_t nblocks_per_upload = 1024 * AK5552_BUFFER_NBLOCKS;
uint32_t input_count = 0;
uint32_t output_count = 0;
uint8_t upload_to_pi = 0;

uint8_t shutdown_pi_after_upload = 1;

#define READING 0x01
#define UPLOADING 0x02
#define POWER_OFF 0x10
#define READY 0x20
#define BOOTING 0x40
#define SHUTTING_DOWN 0x80

uint8_t sys_state = READING;
uint8_t pi_state = POWER_OFF;

#define GPS_PORT 1
#define GPS_UART_BAUDRATE 38400

int main (void)
{
	int result = 0;

	uint32_t main_clk = 120000000;
	sysclk_init();	
	board_init();

	/* Initialize the console uart */
	setup_console();

	/* Output header string  */
    fprintf(stdout, "-- RockHopper V1.0 --\r\n" );
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

	// read and set date and time
	fprintf(stdout, "\r\nLooking for GPS Time on UART1\r\n" );
	rtc_time_t rtc = {.century=20, .year=17, .month=5, .day=26, .hour=1, .minute=0, .second=0};
	ioport_set_pin_level(LED_PIN, LED_ON);
	uart_configure_queue(GPS_PORT, GPS_UART_BAUDRATE);
	uart_set_termination(GPS_PORT, USART_NEWLINE_TERMINATION);  // * termination
	uart_start_queue(GPS_PORT);
	int gps_timeout = 10000; // 10 seconds
	while( gps_timeout ) {
		ioport_toggle_pin_level(LED_PIN);
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
	
	// if not GPS message is found 
	if( gps_timeout == 0) {
		fprintf(stdout, "No GPS Time found.\r\n" );
		fprintf(stdout, "Enter time manually using epoch (or Linux) time.\r\n" );
		uint32_t epoch = rtc_time_to_epoch(&rtc);
		epoch = (uint32_t)console_prompt_int("Enter epoch time", epoch, timeout);
		epoch_to_rtc_time(&rtc, epoch);
		fprintf(stdout, "\r\n");	
	}

	// set the system RTC
	setup_rtc(&rtc);
	
	/* Initialize SD MMC stack */
	if(sd_card_init(&sd_card) != SD_MMC_OK) {
		printf("Init SD Card Failed\n\r");
	}
		
	// Show SD card info.
	if(sd_card.state == SD_MMC_OK) {
		sd_card_print_info(&sd_card);
	}

	// initialize adc	
	ak5552_init(&sampling_rate);
	printf("ADC Initialized: fs = %lu\n\r", sampling_rate);

	printf("Init SSC DMA\n\r");
	uint16_t nsamps = ak5552_init_dma();

	float buffer_duration =  (float)AK5552_NUM_SAMPLES / (float)sampling_rate;
	number_buffers_per_upload = (uint32_t)(60.0f * (float)upload_interval_in_minutes / buffer_duration);
	nblocks_per_upload = number_buffers_per_upload * AK5552_BUFFER_NBLOCKS;
	sd_end_addr = sd_card.last_block;
	sd_start_addr = sd_card.first_block;

	if(Verbose) {
		printf("sd start_addr %lu \n\r", sd_start_addr);
		printf("sd end_addr %lu \n\r", sd_end_addr);
		printf("nblocks_per_spi_buffer %lu \n\r", SPI_DATA_NBLOCKS);
		printf("nblocks_per_adc_buffer %lu \n\r", AK5552_BUFFER_NBLOCKS);
		printf("nblocks_per_upload %lu \n\r", nblocks_per_upload);
	}

	//printf("buffer_duration %f \n\r", buffer_duration);
	printf("sampling_rate %lu \n\r", sampling_rate);
 	printf("number_buffers_per_upload %lu \n\r", number_buffers_per_upload);

	ioport_set_pin_level(PIN_ENABLE_ADC, 1); 
	ioport_set_pin_level(PIN_ENABLE_TVDD, 1);
//	ioport_set_pin_level(PIN_PREAMP_SHDN, 1);

	ioport_set_pin_level(PIN_ENABLE_3V3_OUT, 0);
	ioport_set_pin_level(PIN_ENABLE_5V_OUT, 0);
	
	ioport_set_pin_level(LED_PIN, LED_ON);
	delay_ms(1000);
	
	// start adc conversions and dma
	ak5552_start_dma();
	ak5552_start_conversion();
	
	printf("Start read loop \n\r");
	int go = 1;

	// throw away the first adc buffer
	while( ak5552_read_dma(adc_buffer_header, adc_buffer_data) == 0 ) {}

	sys_state = READING;
	pi_state = POWER_OFF;

	while (go) {
		
		read_data();
		
		if( upload_data() == 0 ) {
			// sleep between dma buffers, the next dma interrupt will wake from sleep
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		}
				
	}

}


int read_data(void)
{	
	uint16_t nsamps = 0;
	
	if( sys_state & READING ) {
		
		nsamps = ak5552_read_dma(adc_buffer_header, adc_buffer_data);

		if( nsamps == AK5552_NUM_SAMPLES ) {

			ioport_toggle_pin_level(LED_PIN);

			int result = sd_card_write_raw(&sd_card, adc_buffer, AK5552_BUFFER_NBLOCKS, sd_write_addr);

			if( result == SD_MMC_OK ) {
				sd_write_addr +=  AK5552_BUFFER_NBLOCKS;
				if(sd_write_addr >= sd_end_addr) sd_write_addr = sd_start_addr;
				input_count += AK5552_BUFFER_NBLOCKS;
			} else {
				printf("sd_card_write_raw: 0x%x.\n\r", result);
			}
			
			if( input_count == nblocks_per_upload ) {
				input_count = 0;
				//sys_state = READING & UPLOADING; // continue reading while uploading
				sys_state = UPLOADING;  // stop reading while uploading
				printf("Ready to upload %lu blocks: read addr 0x%x, write addr 0x%x.\n\r",
				   nblocks_per_upload, sd_read_addr, sd_write_addr);
			}
		}
	}
	
	return(nsamps);
}

int upload_data(void)
{
	// shutoff PI, but wait for the PI to signal that it's OK to shutdown power
	// this happens after uploading has finished
	if( pi_state == SHUTTING_DOWN ) {
		if ( ioport_get_pin_level(PI_ON_PIN) == IOPORT_PIN_LEVEL_LOW ) {
			printf("Shutting down PI.\r\n");
			if(shutdown_pi_after_upload) ioport_set_pin_level(PIN_ENABLE_5V_OUT, 0);
			pi_state = POWER_OFF;
		}
		return(0);
	}
	
	// 
	if ( (sys_state & UPLOADING) == 0 ) return(0);

	if ( pi_state == POWER_OFF ) {
		if ( ioport_get_pin_level(PI_ON_PIN) == IOPORT_PIN_LEVEL_LOW ) {
			printf("Waking up PI ... ");
			ioport_set_pin_level(PIN_ENABLE_5V_OUT, 1); // turn the PI ON
			pi_state = BOOTING;
		}
		return(0);
	}
	
	if ( pi_state == BOOTING)  {
		if ( ioport_get_pin_level(PI_ON_PIN) == IOPORT_PIN_LEVEL_HIGH ) {
			printf("ready.\r\n");
			// initialize SPI slave to transfer data to the PI which is a spi master
			spi_slave_initialize();
			//delay_ms(100);
			pi_state = READY;
		}
		return(0);
	}
	
	// if PI is ready
	if ( pi_state == READY ) {
				
		if( output_count == 0 ) {
			printf("Starting upload to PI.\r\n");
		}
				
		int result = sd_card_read_raw(&sd_card, spi_buffer_data, SPI_DATA_NBLOCKS, sd_read_addr);

		if( result == SD_MMC_OK ) {
					
			ioport_toggle_pin_level(LED_PIN);
					
			update_spi_header(output_count + SPI_DATA_NBLOCKS);
					
			// write padded buffer to SPI port
			result = spi_slave_write_wait((void *)spi_buffer, SPI_BUFFER_NBYTES);
					
			if( result == STATUS_OK ) {
				sd_read_addr += SPI_DATA_NBLOCKS;
				if(sd_read_addr >= sd_end_addr) sd_read_addr = sd_start_addr;
				output_count += SPI_DATA_NBLOCKS;
			} else {
				printf("spi_slave_write_wait error 0x%x.\n\r", result);
			}
					
			if( output_count == nblocks_per_upload ) {
				printf("Finished uploading %lu blocks to PI.\n\r", output_count);
				output_count = 0;
				sys_state = READING;
				pi_state = SHUTTING_DOWN;
			}

		}
		
		return(output_count);
	}
	
	printf("Unknown state %d.\r\n", pi_state);
	return(0);

}


static inline uint8_t checksum(uint8_t *data, uint32_t len)
{
	uint32_t nbytes = len;
	uint8_t *buf = data;
	uint8_t sum = 0;
	while(nbytes--) sum += *buf++;
	return(sum);
}

void update_spi_header(uint32_t count)
{
	
	uint8_t chksum = checksum(spi_buffer_data, SPI_DATA_NBYTES);

	spi_buffer_header[0] = 0; // padding
	spi_buffer_header[1] = 1; // padding
	spi_buffer_header[2] = 2; // padding
	spi_buffer_header[3]  = chksum; //
	spi_buffer_header[4]  = (uint8_t)(count >> 0);
	spi_buffer_header[5]  = (uint8_t)(count >> 8);
	spi_buffer_header[6]  = (uint8_t)(count >> 16);
	spi_buffer_header[7] = (uint8_t)(count >> 24);
	spi_buffer_header[8]  = (uint8_t)(nblocks_per_upload >> 0);
	spi_buffer_header[9]  = (uint8_t)(nblocks_per_upload >> 8);
	spi_buffer_header[10]  = (uint8_t)(nblocks_per_upload >> 16);
	spi_buffer_header[11] = (uint8_t)(nblocks_per_upload >> 24);

	//if(output_count == 0) {
	//	for(uint16_t n = 0; n < 36; n++) printf("%d ", spi_buffer[n]);
	//	printf("\r\n");
	//	for(uint16_t n = SPI_BUFFER_NBYTES-36; n < SPI_BUFFER_NBYTES; n++) printf("%d ", spi_buffer[n]);
	//	printf("\n");
	//}
}



