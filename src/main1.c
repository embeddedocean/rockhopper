/**
 *
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <string.h>

#include "sd_card.h"
#include "spi_pdc.h"
#include "ak5552.h"

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

uint32_t adc_fs = 300000;
uint32_t number_buffers_per_upload = 1024;

void setup_rtc(void);

static inline uint8_t checksum(uint8_t *data, uint32_t len)
{
	uint32_t nbytes = len;
	uint8_t *buf = data;
	uint8_t sum = 0;
	while(nbytes--) sum += *buf++;
	return(sum);
}

void configuration_menu(void)
{

}


/**
 *  Configure UART console.
 */
static void setup_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	pio_configure_pin_group(CONF_UART_PIO, CONF_PINS_UART, CONF_PINS_UART_FLAGS);
	stdio_serial_init(CONF_UART, &uart_serial_options);
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
void setup_system_clocks(uint32_t sclk)
{
	uint32_t xtal = BOARD_FREQ_MAINCK_XTAL;
	uint32_t div = 1;
	uint32_t mul = sclk * 2 * div / xtal;
	
	/* Set flash wait state to max in case the below clock switching. */
	system_init_flash(CHIP_FREQ_CPU_MAX);

	// CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_PLLACK
	struct pll_config pllcfg;
	pll_enable_source(PLL_SRC_MAINCK_XTAL);
	pll_config_init(&pllcfg, PLL_SRC_MAINCK_XTAL, div, mul);
	pll_enable(&pllcfg, 0);
	pll_wait_for_lock(0);
	pmc_switch_mck_to_pllack(SYSCLK_PRES_2); // prescaler 2
	
	printf("Clock configuration: sclk = %lu, mul = %lu, div=%lu,\n\r", sclk, mul, div);
		
	/* Update the SystemFrequency variable */
	SystemCoreClockUpdate();

	/* Set a flash wait state depending on the new cpu frequency */
	system_init_flash(sysclk_get_cpu_hz());

	// Turns off the USB clock, does not switch off the PLL.	
	//pmc_disable_udpck();

}


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

	configuration_menu();
	
	setup_rtc();
	
	/* Initialize SD MMC stack */
	sd_card_info_t sd;
	if(sd_card_init(&sd) != SD_MMC_OK) {
		printf("Init SD Card Failed\n\r");
	}
		
	// Show SD card info.
	if(sd.state == SD_MMC_OK) {
		sd_card_print_info(&sd);
	}

	// initialize adc	
	ak5552_init(&adc_fs);
	printf("ADC Initialized: fs = %lu\n\r", adc_fs);

	printf("Init SSC DMA\n\r");
	uint16_t nsamps = ak5552_init_dma();

	//uint32_t error = 0;
	uint32_t write_addr = SD_CARD_START_BLOCK;
	uint32_t read_addr = SD_CARD_START_BLOCK;

	uint32_t end_addr = sd.last_block;
	uint32_t start_addr = sd.first_block;
	
	// nblocks_per_sector must be and even multiple of nblocks_per_buffer
	uint32_t nblocks_per_upload = number_buffers_per_upload * AK5552_BUFFER_NBLOCKS;
	uint32_t input_count = 0;
	uint32_t output_count = 0;
	uint8_t upload_to_pi = 0;
	uint8_t pi_is_awake = 0;
	uint8_t pi_is_asleep = 1;
	
	if(Verbose) {
		printf("sd start_addr %lu \n\r", start_addr);
		printf("sd end_addr %lu \n\r", end_addr);
		printf("nblocks_per_spi_buffer %lu \n\r", SPI_DATA_NBLOCKS);
		printf("nblocks_per_adc_buffer %lu \n\r", AK5552_BUFFER_NBLOCKS);
		printf("nblocks_per_upload %lu \n\r", nblocks_per_upload);	
	} 

	ioport_set_pin_level(PIN_ENABLE_3V3_OUT, 0);
	ioport_set_pin_level(PIN_ENABLE_5V_OUT, 0);
	ioport_set_pin_level(PIN_ENABLE_ADC, 1);
	ioport_set_pin_level(PIN_ENABLE_TVDD, 1);
//	ioport_set_pin_level(PIN_PREAMP_SHDN, 1);
	
	ioport_set_pin_level(LED_PIN, LED_ON);
	delay_ms(1000);
	
	// start adc conversions and dma
	ak5552_start_dma();
	ak5552_start_conversion();
	
	printf("Start read loop \n\r");
	int go = 1;

	// throw away the first adc buffer
	while( ak5552_read_dma(adc_buffer_header, adc_buffer_data, 0) == 0 ) {}

	while (go) {
		
		if( ak5552_read_dma(adc_buffer_header, adc_buffer_data) == AK5552_NUM_SAMPLES ) {

			ioport_toggle_pin_level(LED_PIN);

			result = sd_card_write_raw(&sd, adc_buffer, AK5552_BUFFER_NBLOCKS, write_addr);
			
			if( result == SD_MMC_OK ) {
				write_addr +=  AK5552_BUFFER_NBLOCKS;
				if(write_addr >= end_addr) write_addr = start_addr; 
				input_count += AK5552_BUFFER_NBLOCKS;
				if( input_count == nblocks_per_upload ) {
					input_count = 0;
					upload_to_pi = 1;
					printf("Ready to upload %lu blocks: read addr 0x%x, write addr 0x%x \n\r",
						nblocks_per_upload, read_addr, write_addr);
				}

			} else {
				printf("sd_card_write_raw: 0x%x \n\r", result);
			}
			
		}
		
		if( upload_to_pi ) {
			
			if (ioport_get_pin_level(PI_ON_PIN) == IOPORT_PIN_LEVEL_HIGH) {
				pi_is_ready = 1;
			} else {
				pi_is_ready = 0;
			}
			
			if( !pi_is_awake ) {
				// turn the PI ON
				ioport_set_pin_level(PIN_ENABLE_5V_OUT, 1);
				printf("Waking up PI\r\n");
			}

			// if PI is awake
			if ( pi_is_awake ) {
				
				if( output_count == 0 ) {
					printf("Starting upload to PI\r\n");
				}
				
				result = sd_card_read_raw(&sd, spi_buffer_data, SPI_DATA_NBLOCKS, read_addr);

				if( result == SD_MMC_OK ) {
				
					ioport_toggle_pin_level(LED_PIN);
				
					update_spi_header(output_count + SPI_DATA_NBLOCKS);

					// write padded buffer to SPI port
					result = spi_slave_write_wait((void *)spi_buffer, SPI_BUFFER_NBYTES);
				
					if( result == STATUS_OK ) {
						read_addr += SPI_DATA_NBLOCKS;
						if(read_addr >= end_addr) read_addr = start_addr;
						output_count += SPI_DATA_NBLOCKS;
						// if upload is finished
					} else {
						printf("spi_slave_write_wait error 0x%x \n\r", result);
					}
			
					if( output_count == nblocks_per_upload ) {
						printf("Finished uploading %lu blocks to PI\n\r", output_count);
						output_count = 0;
						upload_to_pi = 0;
					}

				}
				 
			}		
		
		} else {	// sleep between dma buffers
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);	// the dma interrupt wakes this
		}
		

		// shutoff PI, but wait for it to be ready
		printf("Shutdown PI ... ");
		// wait for the PI to signal that it's OK to shutdown power
		if(ioport_get_pin_level(PI_ON_PIN) == IOPORT_PIN_LEVEL_HIGH) {
			ioport_set_pin_level(PIN_ENABLE_5V_OUT, 1);
		}
		printf("done\n\r");

	}

}

void update_spi_header(uint32_t count)
{
	
	uint8_t chksum = checksum(spi_buffer_data, SPI_DATA_NBYTES);

	spi_buffer_header[0] = 0; // padding
	spi_buffer_header[1] = 1; // padding
	spi_buffer_header[2] = 2; // padding
	spi_buffer_header[3]  = chksum; //
	uint32_t count = ;
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
