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


#define STRING_EOL    "\r"
#define STRING_HEADER "-- RockHopper SAM4S --\r\n" \
"-- "BOARD_NAME " --\r\n" \
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

#define NUMBER_BUFFERS_PER_UPLOAD 128

#define SPI_HEADER_NBYTES (16)
#define SPI_DMA_BUFFER_NBLOCKS (4)
#define SPI_DMA_BUFFER_NBYTES (SPI_HEADER_NBYTES + SPI_DMA_BUFFER_NBLOCKS*SD_MMC_BLOCK_SIZE)

uint8_t spi_buffer[SPI_DMA_BUFFER_NBYTES];
uint8_t *spi_header = spi_buffer;
uint8_t *spi_data = &spi_buffer[SPI_HEADER_NBYTES];

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

static uint8_t checksum(uint8_t *data)
{
	uint32_t nbytes = SPI_DMA_BUFFER_NBLOCKS*SD_MMC_BLOCK_SIZE;
	uint8_t *buf = data;
	uint8_t sum = 0;
	while(nbytes--) sum += *buf++;
	return(-sum);
}

 
int main (void)
{
	sysclk_init();
	board_init();

	ioport_set_pin_level(LED_PIN, LED_ON);

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
//	uint32_t nbytes_per_buffer = (AK5552_DMA_BUFFER_NBYTES * AK5552_BYTES_PER_SAMPLE / AK5552_DMA_BYTES_PER_SAMPLE);
//	uint32_t nblocks_per_buffer =  nbytes_per_buffer / SD_MMC_BLOCK_SIZE;
	
	// nblocks_per_sector must be and even multiple of nblocks_per_buffer
	uint32_t nblocks_per_upload = NUMBER_BUFFERS_PER_UPLOAD * AK5552_BUFFER_NBLOCKS;
	uint32_t input_count = 0;
	uint32_t output_count = 0;
	uint8_t upload_sector = 0;
	uint16_t sequence_number = 0;
	
	printf("sd start_addr %lu \n\r", start_addr);
	printf("sd end_addr %lu \n\r", end_addr);
	printf("nblocks_per_buffer %lu \n\r", AK5552_BUFFER_NBLOCKS);
	printf("nblocks_per_upload %lu \n\r", nblocks_per_upload);

	ioport_set_pin_level(PIN_ENABLE_3V3_OUT, 0);
	ioport_set_pin_level(PIN_ENABLE_5V_OUT, 0);
	ioport_set_pin_level(PIN_ENABLE_ADC, 1);
	ioport_set_pin_level(PIN_ENABLE_TVDD, 1);
//	ioport_set_pin_level(PIN_PREAMP_SHDN, 1);
	
	ioport_set_pin_level(LED_PIN, LED_ON);

	delay_ms(1000);
	
//	uint32_t count = 0;
//	while(1) {
//		delay_ms(1000);
//		printf("test: 0x%x \n\r", count++);
//		ioport_toggle_pin_level(LED_PIN);
//	}
		
	while (go) {
	
		input_count = 0;
		
		while( input_count < nblocks_per_upload ) {
		
			uint32_t *adc_buf = NULL;
			uint32_t nsamps = ak5552_dma_read(&adc_buf);
		
			if( nsamps == AK5552_DMA_BUFFER_NSAMPS ) {

				//tick_start = time_tick_get();
				ioport_toggle_pin_level(LED_PIN);

				// overwrite 32 bit data word with 24 bit word
				int m = 0;
				uint8_t *obuf = (uint8_t *)adc_buf;
				//int32_t *tbuf = &test_buffer[count*nsamps];
				for(uint32_t n = 0; n < nsamps; n++) {
					//adc_buf[n] = count;
					//tbuf[n] = ((int32_t)adc_buf[n] << 8) >> 8; // testing only
					adc_buf[n] = n;
					uint8_t *ibuf = (uint8_t *)&adc_buf[n];
					obuf[m++] = ibuf[0];
					obuf[m++] = ibuf[1];
					obuf[m++] = ibuf[2];
					//obuf[m++] = ibuf[3];
				}
			
				//for(int n = 0; n < 8; n++) {
				//	printf("%08x (%d) ", adc_buf[n], (int32_t)adc_buf[n]);
				//}
				//printf("\r\n");

				int ret = sd_card_write_raw(&sd, (uint8_t *)adc_buf, (uint16_t)AK5552_BUFFER_NBLOCKS, write_addr);
			
				if( ret == SD_MMC_OK ) {
					write_addr +=  AK5552_BUFFER_NBLOCKS;
					if(write_addr >= end_addr) write_addr = start_addr;
					input_count += AK5552_BUFFER_NBLOCKS;
				} else {
					printf("sd_card_write_raw: 0x%x \n\r", ret);				
				}
			
			} else {		
				//printf("sleep ... ");
//				pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);	
			}

		}

		printf("Ready to upload %lu blocks: read addr 0x%x, write addr 0x%x \n\r", nblocks_per_upload, read_addr, write_addr);

		// turn the PI ON and wait for ready signal
		ioport_set_pin_level(PIN_ENABLE_5V_OUT, 1);
		printf("Waiting for PI ... ");
		while (ioport_get_pin_level(PI_ON_PIN) == IOPORT_PIN_LEVEL_LOW) {}
		printf("ready\n\r");

		//
		output_count = 0;			
		sequence_number++;
		while( output_count < nblocks_per_upload ) {
	 	
			int ret = sd_card_read_raw(&sd, spi_data, SPI_DMA_BUFFER_NBLOCKS, read_addr);

			if( ret == SD_MMC_OK ) {
				
				uint8_t chksum = checksum(spi_data);
				
			 	ioport_toggle_pin_level(LED_PIN);

				// update spi buffer header
				spi_header[0] = 0xaa;
				spi_header[1] = 0xaa;
				spi_header[2] = SPI_DMA_BUFFER_NBLOCKS;
				spi_header[3] = (uint8_t)(nblocks_per_upload >> 24);
				spi_header[4] = (uint8_t)(nblocks_per_upload >> 16);
				spi_header[5] = (uint8_t)(nblocks_per_upload >> 8);
				spi_header[6] = (uint8_t)(nblocks_per_upload >> 0);
				spi_header[7] = (uint8_t)(sequence_number >> 8);
				spi_header[8] = (uint8_t)(sequence_number >> 0);
				spi_header[9] = (uint8_t)(output_count >> 24);
				spi_header[10] = (uint8_t)(output_count >> 16);
				spi_header[11] = (uint8_t)(output_count >> 8);
				spi_header[12] = (uint8_t)(output_count >> 0);
				spi_header[13] = chksum;
				spi_header[14] = 0xaa;
				spi_header[15] = 0xaa;

				//while (ioport_get_pin_level(SPI_MASTER_READY_PIN) == IOPORT_PIN_LEVEL_LOW) {}

				// write buffer to SPI
				ret = spi_slave_write_wait((void *)spi_buffer, SPI_DMA_BUFFER_NBYTES);
					
				if( ret == STATUS_OK ) {
					read_addr += SPI_DMA_BUFFER_NBLOCKS;
					if(read_addr >= end_addr) read_addr = start_addr;
					output_count += SPI_DMA_BUFFER_NBLOCKS;
					//printf("spi_slave_write_wait: output count %lu\n\r", output_count);
				} else {
					printf("spi_slave_write_wait error 0x%x \n\r", ret);
				}
			
			} else {
				if(Verbose) printf("sd_card_read_raw: 0x%x \n\r", ret);
			}
			
		}
		
		printf("%lu blocks written\n\r", output_count);

		// shutoff PI, but wait for it to be ready
		printf("Shutdown PI ... ");
		while (ioport_get_pin_level(PI_ON_PIN) == IOPORT_PIN_LEVEL_LOW) {}
		printf("done\n\r");
		ioport_set_pin_level(PIN_ENABLE_5V_OUT, 1);


	}


}


