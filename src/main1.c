/**
 *
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "sd_card.h"
#include "spi_pdc.h"
#include "ak5552.h"
#include "console.h"

//void time_tick_init(void);
//uint32_t time_tick_get(void);
//uint32_t time_tick_calc_delay(uint32_t tick_start, uint32_t tick_end);


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

//uint32_t sampling_rate = 300000;
uint32_t sampling_rate = 150000;
uint32_t upload_interval_in_minutes = 60;

uint32_t number_buffers_per_upload = 512;

int read_console_input(void);
int read_data(void);
int upload_data(void);
void update_spi_header(uint32_t count);
void setup_rtc(void);

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

#define READING 0x01
#define UPLOADING 0x02
#define POWER_OFF 0x10
#define READY 0x20
#define BOOTING 0x40
#define SHUTTING_DOWN 0x80

uint8_t sys_state = READING;
uint8_t pi_state = POWER_OFF;

int main (void)
{
	int result = 0;

	uint32_t main_clk = 120000000;
	//setup_system_clocks(main_clk);
	sysclk_init();
	
	board_init();
	
	//pmc_disable_periph_clk(uint32_t ul_id);
	//pmc_disable_udpck(void);

	ioport_set_pin_level(LED_PIN, LED_ON);

	/* Initialize the console uart */
	setup_console();

	/* Output header string  */
    fprintf(stdout, "-- RockHopper V1.0 --\r\n" );
	fprintf(stdout, "-- Compiled: "__DATE__ " "__TIME__ " --\r\n");

	// read configuration 
    fprintf(stdout, "\r\nEnter new configuration values or hit Enter to accept default values\r\n" );
	int timeout = 4;
    fprintf(stdout, "Prompt will timeout in %d seconds.\r\n", timeout );
	sampling_rate = (uint32_t)console_prompt_int("Enter sampling rate in Hz", sampling_rate, timeout);
	upload_interval_in_minutes = (uint32_t)console_prompt_int("Enter upload interval in minutes", upload_interval_in_minutes, timeout);
    fprintf(stdout, "\r\n" );

	setup_rtc();
	
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
	printf("upload_interval_in_minutes %lu \n\r", upload_interval_in_minutes);
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
			ioport_set_pin_level(PIN_ENABLE_5V_OUT, 0);
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




/**
 * Calculate week from year, month, day.
 */
static uint32_t calculate_week(uint32_t ul_year, uint32_t ul_month, uint32_t ul_day)
{
	uint32_t ul_week;

	if (ul_month == 1 || ul_month == 2) {
		ul_month += 12;
		--ul_year;
	}

	ul_week = (ul_day + 2 * ul_month + 3 * (ul_month + 1) / 5 + ul_year +
			ul_year / 4 - ul_year / 100 + ul_year / 400) % 7;

	++ul_week;

	return ul_week;
}

void setup_rtc(void)
{
	uint32_t year = 2017;
	uint32_t month = 4;
	uint32_t day = 28;
	uint32_t week = calculate_week(year, month, day);
	uint32_t hour = 0;
	uint32_t minute = 0;
	uint32_t sec = 0;
	uint32_t usec = 0;
	
	rtc_set_hour_mode(RTC, 0);
	rtc_set_time(RTC, hour, minute, sec);
	rtc_set_date(RTC, year, month, day, week);

	rtc_get_time(RTC, &hour, &minute, &sec);
	rtc_get_date(RTC, &year, &month, &day, &week);
	
	printf("RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, sec);
}
