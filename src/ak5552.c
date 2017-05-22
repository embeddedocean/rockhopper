/*
 * ak5552.c
 *
 * Created: 5/6/2016 11:19:06 AM
 *  Author: chris
 */ 

//----------------------------------------------------------------------------------------
// AK5552
#include <asf.h>
 #include "ak5552.h"

/** The SSC interrupt IRQ priority. */
#define SSC_ADC_IRQ_PRIO 1

/* DMA channel */
//#define SSC_ADC_DMA_CH 0

/* Pdc transfer buffer */
COMPILER_WORD_ALIGNED uint8_t ssc_adc_dma_buffer1[AK5552_DMA_BUFFER_NBYTES];
COMPILER_WORD_ALIGNED uint8_t ssc_adc_dma_buffer2[AK5552_DMA_BUFFER_NBYTES];

//uint32_t *ssc_adc_data1 = (uint32_t *)&ssc_adc_dma_buffer1[AK5552_HEADER_NBYTES];
//uint32_t *ssc_adc_data2 = (uint32_t *)&ssc_adc_dma_buffer2[AK5552_HEADER_NBYTES];

/* PDC data packet for transfer */
pdc_packet_t pdc_ssc_packet1;
pdc_packet_t pdc_ssc_packet2;

/* Pointer to SSC PDC register base */
Pdc *ssc_pdc;

/* active buffer number, that is the number o the buffer being read now */
volatile int pdc_active_buffer_number = 1;

// finished buffer
uint8_t *ssc_adc_buffer;
uint16_t ssc_adc_nsamps;
uint32_t ssc_adc_sampling_freq; // Hz

int ssc_adc_buffer_number = 1;
uint32_t ssc_adc_time = 0;
uint32_t ssc_adc_date = 0;

uint32_t ssc_adc_time1 = 0;
uint32_t ssc_adc_date1 = 0;
uint32_t ssc_adc_time2 = 0;
uint32_t ssc_adc_date2 = 0;

uint8_t ak5552_settings = 0;

int ak5552_init(uint32_t *fs)
{	
	/* Initialize the SSC module */
	pmc_enable_periph_clk(ID_SSC);
	ssc_reset(SSC);
	
	ssc_i2s_set_receiver(SSC, SSC_I2S_SLAVE_IN, SSC_RCMR_CKS_RK, SSC_AUDIO_MONO_LEFT, AK5552_BITS_PER_SAMPLE);
	//ssc_i2s_set_receiver(SSC, SSC_I2S_SLAVE_IN, SSC_RCMR_CKS_RK, SSC_AUDIO_STERO, AK5552_SAMPLE_SIZE);
	
	//ssc_enable_tx(SSC);
	//ssc_enable_rx(SSC);

	/* TESTING ONLY - Enable the loop mode. */
	//ssc_set_loop_mode(SSC);

	// Configure TC TC_CHANNEL_WAVEFORM in waveform operating mode.
	//see: http://www.allaboutcircuits.com/projects/dma-digital-to-analog-conversion-with-a-sam4s-microcontroller-the-timer-cou/

	uint32_t ra, rc, sck;
	uint16_t dutycycle = 50; /** Duty cycle in percent (positive).*/

	/* Configure PIO Pins for TC */
	//ioport_set_pin_mode(PIN_TC0_TIOB0, PIN_TC0_TIOB0_MUX);
	/* Disable IO to enable peripheral mode) */
	//ioport_disable_pin(PIN_TC0_TIOB0);

	ioport_set_pin_mode(PIN_TC0_TIOA0, PIN_TC0_TIOA0_MUX);
	/* Disable IO to enable peripheral mode) */
	ioport_disable_pin(PIN_TC0_TIOA0);

	/* Configure the PMC to enable the TC module. */
	sysclk_enable_peripheral_clock(ID_TC0);

	/* Init TC to waveform mode. */
	tc_init(TC0, 0,
	  TC_CMR_TCCLKS_TIMER_CLOCK1	//MCK/2
	  | TC_CMR_WAVE	//waveform mode
	  | TC_CMR_WAVSEL_UP_RC	//UP mode with automatic trigger on RC Compare
	  //		| TC_CMR_WAVSEL_UP	//UP mode without automatic trigger on RC Compare
	  | TC_CMR_ACPA_TOGGLE	//toggle TIOA on RA match
	  | TC_CMR_ACPC_TOGGLE	//toggle TIOA on RC match
	);
	
	// desired mclk
	uint32_t mclk = *fs * 32;

	/* Configure waveform frequency and duty cycle. */
	sck = sysclk_get_peripheral_bus_hz(TC0);
	rc =  sck / 2 / mclk;
	if(rc < 2) rc = 2;
	mclk = sck / 2 / rc; // actual mclk
	tc_write_rc(TC0, 0, rc);
	ra = rc/2; //(100 - dutycycle) * rc / 100;
	tc_write_ra(TC0, 0, ra);
	//tc_write_rb(TC0, 0, ra);

	// actual sampling freq
	*fs = mclk / 32;
	ssc_adc_sampling_freq = *fs;

	printf("TC0 configuration: sysclk = %lu, mclk = %lu (ra=%lu,rc=%lu)\n\r", sck, mclk, ra, rc);
	
	return(1);
}


void ak5552_start_conversion(void)
{
	/* Enable TC0 Channel 0. */
	tc_start(TC0, 0);
}

void ak5552_stop_conversion(void)
{
	/* Disable TC0 Channel 0. */
	tc_stop(TC0, 0);
}

/* RTC functions */
#define BCD_SHIFT      4
#define BCD_MASK       0xfu
#define BCD_FACTOR     10

void ak5552_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec)
{
	uint32_t time = ssc_adc_time;
	uint32_t temp;

	/* Hour */
	temp = (time & RTC_TIMR_HOUR_Msk) >> RTC_TIMR_HOUR_Pos;
	*hour = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));

	if ((time & RTC_TIMR_AMPM) == RTC_TIMR_AMPM) {
		*hour += 12;
	}

	/* Minute */
	temp = (time & RTC_TIMR_MIN_Msk) >> RTC_TIMR_MIN_Pos;
	*minute = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR +  (temp & BCD_MASK));

	/* Second */
	temp = (time & RTC_TIMR_SEC_Msk) >> RTC_TIMR_SEC_Pos;
	*second = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));

	// temporary !!!
	*usec = 0;
}

void ak5552_get_date(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *week)
{
	uint32_t date = ssc_adc_date;
	uint32_t cent;
	uint32_t temp;

	/* Retrieve year */
	if (year) {
		temp = (date & RTC_CALR_CENT_Msk) >> RTC_CALR_CENT_Pos;
		cent = (temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK);
		temp = (date & RTC_CALR_YEAR_Msk) >> RTC_CALR_YEAR_Pos;
		*year = (uint8_t)((cent * BCD_FACTOR * BCD_FACTOR) + (temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
	}

	/* Retrieve month */
	if (month) {
		temp = (date & RTC_CALR_MONTH_Msk) >> RTC_CALR_MONTH_Pos;
		*month = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
	}

	/* Retrieve day */
	if (day) {
		temp = (date & RTC_CALR_DATE_Msk) >> RTC_CALR_DATE_Pos;
		*day = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
	}

	/* Retrieve week */
	if (week) {
		*week = (uint8_t)(((date & RTC_CALR_DAY_Msk) >> RTC_CALR_DAY_Pos));
	}
}

/**
 * Synchronous Serial Controller IRQ Handler.
 * It must be called SSC_Handle because that is the name defined in the sam4s***.h file
 
 When the current transfer counter reaches zero, the channel checks its next transfer counter. 
 If the value of the next counter is zero, the channel stops transferring data and sets the appropriate flag. 
 If the next counter value is greater than zero, the values of the next pointer/next counter are
 copied into the current pointer/current counter and the channel resumes the transfer, whereas next pointer/next counter
 get zero/zero as values.
 At the end of this transfer, the PDC channel sets the appropriate flags in the Peripheral Status register.
 
 */
void SSC_Handler(void)
{
	uint32_t status;
	
	status = ssc_get_status(SSC);
	
	/* Get SSC status and check if the current PDC receive buffer is full */
	// ENDRX flag is set when the PDC Receive Counter register (PERIPH_RCR) reaches zero.
	if ((status & SSC_SR_ENDRX) == SSC_SR_ENDRX) {
		// Current DMA buffer is done.
		/* Configure PDC for next data transfer */ 
		// writing to RNCR reset status flag
		// signal that the buffer is ready to be read
		if(pdc_active_buffer_number == 1) {
			// buffer1 is done, the dma is now reading buffer2, so set buffer1 as next buffer
			ssc_adc_time2 = RTC->RTC_TIMR;
			ssc_adc_date2 = RTC->RTC_CALR;
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet1);
			pdc_active_buffer_number = 2;
			ssc_adc_buffer = ssc_adc_dma_buffer1;
			ssc_adc_time = ssc_adc_time1;
			ssc_adc_date = ssc_adc_date1;
			//fprintf(stdout, "SSC_Handler: buffer 1 done, status = %x\r\n", status);
			//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
		} else if(pdc_active_buffer_number == 2) {
			// buffer2 is done, now reading buffer1, set buffer2 as next
			ssc_adc_time1 = RTC->RTC_TIMR;
			ssc_adc_date1 = RTC->RTC_CALR;
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet2);
			pdc_active_buffer_number = 1;
			ssc_adc_buffer = ssc_adc_dma_buffer2;
			ssc_adc_time = ssc_adc_time2;
			ssc_adc_date = ssc_adc_date2;
			//ioport_set_pin_level(SSC_ADC_BUF_PIN, 1);
			//fprintf(stdout, "SSC_Handler: buffer 2 done, status = %x\r\n", status);
		} 
		else {
			fprintf(stdout, "SSC_Handler: confused\r\n");
		}
		
//		fprintf(stdout, "pdc_current_buffer = %d, ", g_pdc_current_buffer);
//		fprintf(stdout, "rx_cnt=%ld, rx_ptr=%ld, ", 
//			pdc_read_rx_counter(g_p_ssc_pdc), pdc_read_rx_ptr(g_p_ssc_pdc));
//		fprintf(stdout, "rx_next_cnt=%ld, rx_next_ptr=%ld \n\r", 
//			pdc_read_rx_next_counter(g_p_ssc_pdc), pdc_read_rx_next_ptr(g_p_ssc_pdc));

	}

	// RXBUFF flag is set when both PERIPH_RCR and the PDC Receive Next Counter register (PERIPH_RNCR) reach zero.
	if ((status & SSC_SR_RXBUFF) == SSC_SR_RXBUFF) {
		fprintf(stdout, "SSC_Handler: buffer overflow\r\n");
		// buffer overflow
	}

}


static uint8_t checksum(uint8_t *data, uint32_t len)
{
	uint32_t nbytes = len;
	uint8_t *buf = data;
	uint8_t sum = 0;
	while(nbytes--) sum += *buf++;
	return(sum);
}

void ak5552_update_header(uint8_t *hdr, uint8_t flags, uint8_t chksum)
{
	// update the data header info
	uint8_t year,month,day,hour,minute,sec;
	uint32_t usec = 0;
	ak5552_get_time(&hour, &minute, &sec, &usec);
	ak5552_get_date(&year, &month, &day, NULL);

	hdr[0]  = 0xaa; // padding
	hdr[1]  = flags;
	hdr[2]  = (uint8_t)(ak5552_settings);
	hdr[3]  = (uint8_t)(AK5552_BUFFER_NBLOCKS);
	hdr[4]  = (uint8_t)(AK5552_BYTES_PER_SAMPLE);
	hdr[5]  = (uint8_t)(AK5552_NUM_SAMPLES >> 0);
	hdr[6]  = (uint8_t)(AK5552_NUM_SAMPLES >> 8);
	hdr[7]  = (uint8_t)(ssc_adc_sampling_freq >> 0);
	hdr[8]  = (uint8_t)(ssc_adc_sampling_freq >> 8);
	hdr[9]  = (uint8_t)(ssc_adc_sampling_freq >> 16);
	hdr[10] = (uint8_t)(ssc_adc_sampling_freq >> 24);
	hdr[11] = (uint8_t)(year - 2000);
	hdr[12] = (uint8_t)(month);
	hdr[13] = (uint8_t)(day);
	hdr[14] = (uint8_t)(hour);
	hdr[15] = (uint8_t)(minute);
	hdr[16] = (uint8_t)(sec);
	hdr[17] = (uint8_t)(usec >> 0);
	hdr[18] = (uint8_t)(usec >> 8);
	hdr[19] = (uint8_t)(usec >> 16);
	hdr[20] = (uint8_t)(usec >> 24);
	hdr[21] = chksum;   // checksum for data
	hdr[22] = checksum(&hdr[1], 21);  // checksum for header, not including the padding
	hdr[23] = 0xaa;  // padding	
}

uint8_t *ak5552_get_dma_buffer(void)
{
	// no buffer ready or buffer has already been read
	if(ssc_adc_buffer == NULL) return(NULL);
	
	// set the pointer to the finished buffer
	uint8_t *buffer = ssc_adc_buffer;

	// indicate that the buffer has already been read
	ssc_adc_buffer = NULL;
	
	// return the buffer pointer
	return(buffer);	
}

//
// copy finished dma buffer into user buffer and update data header.
// flags are anything extra that you want to put in the header.
// be careful with timing here because the dma buffer gets overwritten.
//
uint16_t ak5552_read_dma(uint8_t *hdr, uint8_t *data, uint8_t flags)
{
	// no buffer ready or buffer has already been read
	if(ssc_adc_buffer == NULL) return(0);
	
	// save 32 bit data word as 24 bit word
	uint8_t *obuf = data;  // output buffer
	uint8_t *ibuf = ssc_adc_buffer;  // finished dma buffer
	uint8_t chksum = 0;
	uint32_t nbytes = AK5552_NUM_SAMPLES * 4;
	uint32_t m = 0;
	for(uint32_t n = 0; n < nbytes; n += 4) {
		*obuf++ = ibuf[n]; // = (uint8_t)m;
		*obuf++ = ibuf[n+1]; // = (uint8_t)(m >> 8);
		*obuf++ = ibuf[n+2]; // = (uint8_t)(m++ >> 16);
		//obuf[m++] = ibuf[n+3];
		chksum += ibuf[n] + ibuf[n+1] + ibuf[n+2];
	}

	// indicate that the dma buffer has been read
	ssc_adc_buffer = NULL;
	
	// update the data header info
	ak5552_update_header(hdr, flags, chksum);
	
	// return the buffer pointer
	return(AK5552_NUM_SAMPLES);
}

/**
 * \brief DMA driver configuration
 */
uint16_t ak5552_init_dma()
{
	//if( nsamps > AK5552_MAX_DMA_BUFFER_NSAMPS ) {
	//	printf("ak5552_dma_init: error, nsamps (%d) too large, use %lu max \n\r", nsamps, AK5552_MAX_DMA_BUFFER_NSAMPS);
	//	return(-1);
	//}
	
	/* Get pointer to UART PDC register base */
	ssc_pdc = ssc_get_pdc_base(SSC);

	/* Initialize PDC data packets for transfer */
	pdc_ssc_packet1.ul_addr = (uint32_t)&ssc_adc_dma_buffer1;
	pdc_ssc_packet1.ul_size = (uint32_t)AK5552_NUM_SAMPLES;
	pdc_ssc_packet2.ul_addr = (uint32_t)&ssc_adc_dma_buffer2;
	pdc_ssc_packet2.ul_size = (uint32_t)AK5552_NUM_SAMPLES;

	pdc_active_buffer_number = 1;
	ssc_adc_buffer = NULL;  // no buffer ready
	ssc_adc_nsamps = AK5552_NUM_SAMPLES;
	
	/* Configure PDC for data receive */
	pdc_rx_init(ssc_pdc, &pdc_ssc_packet1, &pdc_ssc_packet2);

	printf("ak5552_adc_nsamps %lu \n\r", ssc_adc_nsamps);
	
//	fprintf(stdout, "rx_cnt=%ld, rx_ptr=%lx, ", 
//		pdc_read_rx_counter(ssc_pdc), pdc_read_rx_ptr(ssc_pdc));
//	fprintf(stdout, "rx_next_cnt=%ld, rx_next_ptr=%lx \n\r", 
//		pdc_read_rx_next_counter(ssc_pdc), pdc_read_rx_next_ptr(ssc_pdc));

	/* Configure the RX End of Reception interrupt. */
	ssc_enable_interrupt(SSC, SSC_IER_ENDRX);

	/* Enable SSC interrupt line from the core */
	NVIC_DisableIRQ(SSC_IRQn);
	NVIC_ClearPendingIRQ(SSC_IRQn);
	NVIC_SetPriority(SSC_IRQn, SSC_ADC_IRQ_PRIO);
	NVIC_EnableIRQ(SSC_IRQn);

	/* Enable PDC receive transfers */
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	//ioport_set_pin_dir(SSC_ADC_BUF_PIN, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
	return(ssc_adc_nsamps);
}

/**
 *  \brief Start DMA .
 */
void ak5552_start_dma(void) 
{
	// enable pdc receiver channel requests
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	ssc_enable_rx(SSC);
	//ssc_enable_tx(SSC);
	
	ssc_adc_time1 = RTC->RTC_TIMR;
	ssc_adc_date1 = RTC->RTC_CALR;
	ssc_adc_time2 = RTC->RTC_TIMR;
	ssc_adc_date2 = RTC->RTC_CALR;

}

void ak5552_stop_dma(void)
{
	// enable pdc receiver channel requests
	pdc_disable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	ssc_disable_rx(SSC);
	//ssc_enable_tx(SSC);
}
