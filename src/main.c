/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <string.h>

#include "sd_card.h"
#include "spi_pdc.h"
#include "ltc2380.h"


void time_tick_init(void);
uint32_t time_tick_get(void);
uint32_t time_tick_calc_delay(uint32_t tick_start, uint32_t tick_end);


#define STRING_EOL    "\r"
#define STRING_HEADER "-- RockHopper SAM4S --\r\n" \
"-- "BOARD_NAME " --\r\n" \
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
	
#define SPI_DMA_BUFFER_NBLOCKS (8)
#define SPI_DMA_BUFFER_NBYTES (SPI_DMA_BUFFER_NBLOCKS*SD_MMC_BLOCK_SIZE)

uint8_t spi_buffer[SPI_DMA_BUFFER_NBYTES];

uint8_t Verbose = 1;
	
/**
 *  Configure UART console.
 */
static void configure_console(void)
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
 
int main (void)
{
	uint32_t fs;
	//uint32_t tick_start;
	uint32_t time_ms = 0;
	uint32_t max_time_ms = 0;

	sysclk_init();
	board_init();

	/* Initialize the console uart */
	if(Verbose) configure_console();

	/* Output header string  */
	puts(STRING_HEADER);

	/* Initialize SD MMC stack */
	sd_card_info_t sd;
	if(sd_card_init(&sd) != SD_MMC_OK) {
		if(Verbose) printf("Init SD Card Failed\n\r");
	}
	
	//time_tick_init();
	
	// Show SD card info.
	if(sd.state == SD_MMC_OK) {
		sd_card_print_info(&sd);
	}
	
	//char Filename[32];
	//sprintf(Filename, "test.data");
	//sd_card_init_fat();
	//sd_card_open_file(&sd, Filename);
	
	// initialize SPI master
	//spi_master_initialize();
//	spi_slave_initialize();

	if(Verbose) printf("Init SSC ADC\n\r");
	fs = 300000;
	ltc2380_init(fs);

	// Insert application code here, after the board has been initialized.
	if(Verbose) printf("Init SSC DMA\n\r");
	ltc2380_dma_init();

	if(Verbose) printf("Start SSC DMA\n\r");
	ltc2380_dma_start();

	ltc2380_start_convertion(fs);
	
	if(Verbose) printf("Start read loop \n\r");
	int go = 1;

	//uint32_t error = 0;

	uint32_t write_addr = SD_CARD_START_BLOCK;
	uint32_t read_addr = SD_CARD_START_BLOCK;

	uint32_t end_addr = sd.last_block;
	uint32_t start_addr = sd.first_block;
	// nblocks_per_buffer is 3/4 the size of the dma buffer because we only need 24 of the 32 bits
	uint16_t nblocks_per_buffer = (LTC2380_DMA_BUFFER_SIZE * 3) / SD_MMC_BLOCK_SIZE;
	// nblocks_per_sector must be and even multiple of nblocks_per_buffer
	uint32_t nblocks_per_sector = 256 * (LTC2380_DMA_BUFFER_SIZE * 3) / SD_MMC_BLOCK_SIZE;
	uint32_t input_count = 0;
	uint32_t output_count = 0;
	uint8_t upload_sector = 0;
	
	printf("nblocks_per_buffer %d \n\r", nblocks_per_buffer);
	printf("nblocks_per_sector %d \n\r", nblocks_per_sector);
	
	int nsamps = 0;
	uint32_t *adc_buf = NULL;

	ioport_set_pin_level(PIN_ENABLE_PI_3V3, 0);
	ioport_set_pin_level(PIN_ENABLE_5V_SW, 1);
	ioport_set_pin_level(PIN_ENABLE_ADC, 1);
	int vout_on = 0;
	
	while (go) {
		
		adc_buf = NULL;
		nsamps = ltc2380_dma_read(&adc_buf);
		//nsamps = 1;
		
		if( nsamps > 0 ) {

			//tick_start = time_tick_get();
			
			// overwrite 32 bit data word with 24 bit word
			int m = 0;
			uint8_t *obuf = (uint8_t *)adc_buf;
			for(int n = 0; n < nsamps; n++) {
				//adc_buf[n] = write_addr;  // testing only
				uint8_t *ibuf = (uint8_t *)&adc_buf[n];
				obuf[m++] = ibuf[1];
				obuf[m++] = ibuf[2];
				obuf[m++] = ibuf[3];
			}

			int ret = 1;
			ret = sd_card_write_raw(&sd, (uint8_t *)adc_buf, nblocks_per_buffer, write_addr);
			
			if( ret == SD_MMC_OK ) {
				
				write_addr +=  (uint32_t)nblocks_per_buffer;
				if(write_addr >= end_addr) write_addr = start_addr;
			
				input_count += (uint32_t)nblocks_per_buffer;
				if( input_count == nblocks_per_sector ) {
					input_count = 0;
					upload_sector = 1;
				}
			
			} //else {
			//	printf("sd_card_write_raw error %d \n\r", ret);
			//}

			//printf("sd_card_write_raw %d \n\r", write_addr);
			
		}
		
		if( upload_sector ) {
	 	
			if( input_count == 0 ) {
				if(Verbose) printf("ADC Buffer upload %ld \n\r", read_addr);
			}
			 

			int ret = 1;
			ret = sd_card_read_raw(&sd, spi_buffer, SPI_DMA_BUFFER_NBLOCKS, read_addr);
			
			if( ret == SD_MMC_OK) {
				
				// write buffer to SPI
				//spi_master_write_wait((void *)spi_buffer, SPI_DMA_BUFFER_NBYTES);
//				spi_slave_write_wait((void *)spi_buffer, SPI_DMA_BUFFER_NBYTES);

//				for(int n = 0; n < 128; n++) {
//					printf("%x ", spi_buffer[n]);
//				}
				
				read_addr += (uint32_t)nblocks_per_buffer;
				if(read_addr >= end_addr) read_addr = start_addr;
			
				output_count += (uint32_t)nblocks_per_buffer;
				if( output_count == nblocks_per_sector ) {
					output_count = 0;
					upload_sector = 0;
				}
			
			}
			//sd_card_write_file((uint8_t *)adc_buf, nblocks);
			//ioport_set_pin_level(LED_0_PIN, !LED_0_ACTIVE);
			
			//time_ms = time_tick_calc_delay(tick_start, time_tick_get());
			//if(time_ms > max_time_ms) max_time_ms = time_ms;

		}
		else {
			
			ioport_set_pin_level(LED0_PIN, LED0_OFF);
			
			//printf("sleep ... ");
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
			
			ioport_set_pin_level(LED0_PIN, LED0_ON);

		}
		
	}

//	printf("Stop SSC DMA\n\r");
	ltc2380_dma_stop();

//	float t = (1000.0f * (float)LTC2380_DMA_BUFFER_SIZE / (float)fs);
//	printf("ADC Buffer Duration %d.%d msec \r\n", (int)(t), (int)(100.0f*(t - (int)t)) );
//	printf("SD Card write time %lu msec, %lu KSPS \r\n", time_ms, ( (int)(LTC2380_DMA_BUFFER_SIZE * 1000lu) / 1024lu) / time_ms);
//	printf("SD Card max write time %lu msec, %lu KSPS \r\n", max_time_ms, ((LTC2380_DMA_BUFFER_SIZE * 1000lu) / 1024lu) / max_time_ms);

}


//--------------------------------------------------------------------------------------------

/** Counts for 1ms time ticks. */
volatile uint32_t g_ms_ticks = 0;

#define TICK_US 1000
/**
 * \brief Handler for Sytem Tick interrupt.
 *
 * Process System Tick Event
 * Increments the g_ms_ticks counter.
 */
void SysTick_Handler(void)
{
	g_ms_ticks++;
}

void time_tick_init(void)
{
	g_ms_ticks = 0;

	/* Configure systick */
	if (SysTick_Config(sysclk_get_cpu_hz() / TICK_US)) {
		Assert(false);
	}
}

uint32_t time_tick_get(void)
{
	return g_ms_ticks;
}

uint32_t time_tick_calc_delay(uint32_t tick_start, uint32_t tick_end)
{
	if (tick_end >= tick_start) {
		return (tick_end - tick_start) * (1000 / TICK_US);
	} else {
		/* In the case of 32-bit couter number overflow */
		return (tick_end + (0xFFFFFFFF - tick_start)) * (1000 / TICK_US);
	}
}
