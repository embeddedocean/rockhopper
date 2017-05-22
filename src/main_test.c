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
#include "ak5552.h"


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

//int32_t test_buffer[2*AK5552_DMA_BUFFER_SIZE];

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
	sysclk_init();
	board_init();

	ioport_set_pin_level(LED0_PIN, LED_ON);
	ioport_set_pin_level(LED1_PIN, LED_ON);

	/* Initialize the console uart */
	if(Verbose) configure_console();

	/* Output header string  */
	puts(STRING_HEADER);

	/* Initialize SD MMC stack */
	sd_card_info_t sd;
	if(sd_card_init(&sd) != SD_MMC_OK) {
		if(Verbose) printf("Init SD Card Failed\n\r");
	}
		
	// Show SD card info.
	if(sd.state == SD_MMC_OK) {
		sd_card_print_info(&sd);
	}
	
	// initialize SPI slave to transfer data to the PI which is a spi master 
	// because linux can only handle spi master made
	spi_slave_initialize();

	if(Verbose) printf("Init SSC ADC\n\r");
	ak5552_init();

	// Insert application code here, after the board has been initialized.
	if(Verbose) printf("Init SSC DMA\n\r");
	ak5552_dma_init();

	if(Verbose) printf("Start SSC DMA\n\r");
	ak5552_dma_start();

	uint32_t fs = 150000;
	uint32_t mclk = 32*fs;
	ak5552_start_convertion(&mclk);
	fs = mclk/32;
	if(Verbose) printf("Start conversion: fs = %lu\n\r", fs);
	
	if(Verbose) printf("Start read loop \n\r");
	int go = 1;

	//uint32_t error = 0;
	uint32_t write_addr = SD_CARD_START_BLOCK;
	uint32_t read_addr = SD_CARD_START_BLOCK;

	uint32_t end_addr = sd.last_block;
	uint32_t start_addr = sd.first_block;
	// nblocks_per_buffer is 3/4 the size of the dma buffer because we only need 24 of the 32 bits
	uint32_t nbytes_per_buffer = (AK5552_DMA_BUFFER_NBYTES * AK5552_BYTES_PER_SAMPLE / AK5552_DMA_BYTES_PER_SAMPLE);
	uint32_t nblocks_per_buffer =  nbytes_per_buffer / SD_MMC_BLOCK_SIZE;
	
	// nblocks_per_sector must be and even multiple of nblocks_per_buffer
	uint32_t nblocks_per_sector = 64 * nblocks_per_buffer;
	uint32_t input_count = 0;
	uint32_t output_count = 0;
	uint8_t upload_sector = 0;
	
	printf("sd start_addr %lu \n\r", start_addr);
	printf("sd end_addr %lu \n\r", end_addr);
	printf("nbytes_per_buffer %lu \n\r", nbytes_per_buffer);
	printf("nblocks_per_buffer %lu \n\r", nblocks_per_buffer);
	printf("nblocks_per_sector %lu \n\r", nblocks_per_sector);
	
	int nsamps = 0;
	uint32_t *adc_buf = NULL;

//	uint8_t buf[SD_MMC_BLOCK_SIZE];
//	int ret = 0;
//	ret = sd_card_read_raw(&sd, buf, 1, 0);
//	for(int i = 0; i < 32; i++) {
//		for(int j = 0; j < 16; j++) {
//			printf("%02X ", buf[j+16*i]);
//		}
//		printf("\r\n");
//	}

//	uint32_t nfats = (uint32_t)buf[16];
//	uint32_t fatsize = (uint32_t)buf[36] + ((uint32_t)buf[37] << 8); // + ((uint32_t)buf[38] << 16) + ((uint32_t)buf[39] << 24);
//	uint32_t start_block = nfats * fatsize + 1;
//	printf("start at %X, (%X, %X)\r\n", start_block, nfats, fatsize);

//	start_block = 0;
//	int nblocks = 128;
//	for(int n = start_block; n < start_block+nblocks; n++) {
//		ret = sd_card_read_raw(&sd, buf, 1, n);
//		if(ret != SD_MMC_OK) break;
//		//printf("Block %d\r\n", n);
//		for(int i = 0; i < 32; i+=32) {
//			for(int j = 0; j < 16; j++) {
//				printf("%02X ", buf[j+16*i]);
//			}
//			printf("   ");
//			for(int j = 0; j < 16; j++) {
//				if(buf[j+16*i] == 0) printf(".");
//				else printf("%c", buf[j+16*i]);
//			}
//			printf("\r\n");
//		}
//		printf("\r\n");
//	}
//	return(0);

	ioport_set_pin_level(PIN_ENABLE_5V, 1);
	ioport_set_pin_level(PIN_ENABLE_ADC, 1);
//	ioport_set_pin_level(PIN_PREAMP_SHDN, 1);
	
	ioport_set_pin_level(LED0_PIN, LED_OFF);
	ioport_set_pin_level(LED1_PIN, LED_OFF);

	delay_ms(1000);
	
	uint32_t count = 0;
	
	while (go) {
		
		adc_buf = NULL;
		nsamps = ak5552_dma_read(&adc_buf);
		
		if( nsamps > 0 ) {

			//tick_start = time_tick_get();
			ioport_toggle_pin_level(LED1_PIN);

			// overwrite 32 bit data word with 24 bit word
			int m = 0;
			uint8_t *obuf = (uint8_t *)adc_buf;
			//int32_t *tbuf = &test_buffer[count*nsamps];
			for(int n = 0; n < nsamps; n++) {
				//adc_buf[n] = count;
				//tbuf[n] = ((int32_t)adc_buf[n] << 8) >> 8; // testing only
				uint8_t *ibuf = (uint8_t *)&adc_buf[n];
				obuf[m++] = ibuf[1];
				obuf[m++] = ibuf[2];
				obuf[m++] = ibuf[3];
			}

 			//count++;
			//if(count >= 128) break;
			
			int ret = 1;
			ret = sd_card_write_raw(&sd, (uint8_t *)adc_buf, (uint16_t)nblocks_per_buffer, write_addr);
			
			if( ret == SD_MMC_OK ) {
				
				write_addr +=  nblocks_per_buffer;
				if(write_addr >= end_addr) write_addr = start_addr;
			
				input_count += nblocks_per_buffer;
				if( input_count == nblocks_per_sector ) {
					input_count = 0;
					upload_sector = 1;
					printf("ADC Buffer ready to upload from %ld \n\r", read_addr);
				}
			
			} else {
				printf("sd_card_write_raw: 0x%x \n\r", ret);				
			}
			
		}

		//			
		if( upload_sector ) {
	 	
			//if( output_count == 0 ) {
			//	if(Verbose) printf("ADC Buffer upload %ld \n\r", read_addr);
			//}

			int ret = 1;
			
			if (ioport_get_pin_level(SPI_MASTER_READY_PIN) == IOPORT_PIN_LEVEL_HIGH) {
				
				ret = sd_card_read_raw(&sd, spi_buffer, SPI_DMA_BUFFER_NBLOCKS, read_addr);	

				if( ret == SD_MMC_OK) {
				
					// write buffer to SPI
					ret = spi_slave_write_wait((void *)spi_buffer, SPI_DMA_BUFFER_NBYTES);
					
					if( ret == STATUS_ERR_TIMEOUT ) {
						printf("spi_slave_write_wait error 0x%x \n\r", ret);
					}
					if( ret == STATUS_ERR_BUSY ) {
						printf("spi_slave_write_wait error 0x%x \n\r", ret);
					}

					read_addr += SPI_DMA_BUFFER_NBLOCKS;
					if(read_addr >= end_addr) read_addr = start_addr;
			
					output_count += SPI_DMA_BUFFER_NBLOCKS;
					if( output_count == nblocks_per_sector ) {
						output_count = 0;
						upload_sector = 0;
					}
			
				} else {
					if(Verbose) printf("sd_card_read_raw: 0x%x \n\r", ret);
				}
			}
			
		}
		else {
				
			//printf("sleep ... ");
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		}
		
	}

	//	printf("Stop SSC DMA\n\r");
//	ltc2380_dma_stop();

//	for(int n = 0; n < 2*AK5552_DMA_BUFFER_SIZE; n++) {
//	for(int n = AK5552_DMA_BUFFER_SIZE-512; n < 1024; n++) {
//	int m = AK5552_DMA_BUFFER_SIZE-512;
//	for(int n = 0; n < 1024; n++) {
//		printf("%ld ", test_buffer[m+n]);
//	}
//	printf("\n\r");

}


