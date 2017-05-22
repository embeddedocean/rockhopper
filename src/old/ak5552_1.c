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
COMPILER_WORD_ALIGNED uint32_t ssc_adc_dma_buffer1[AK5552_DMA_BUFFER_NSAMPS];
COMPILER_WORD_ALIGNED uint32_t ssc_adc_dma_buffer2[AK5552_DMA_BUFFER_NSAMPS];

/* PDC data packet for transfer */
pdc_packet_t pdc_ssc_packet1;
pdc_packet_t pdc_ssc_packet2;

/* Pointer to SSC PDC register base */
Pdc *ssc_pdc;

/* active buffer number, that is the number o the buffer being read now */
volatile int pdc_active_buffer_number = 1;

// finished buffer
uint32_t *ssc_adc_buffer;
int ssc_adc_buffer_number = 1;

int ak5552_init(void)
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

	return(1);
}


/**
 * \brief Configure TC TC_CHANNEL_WAVEFORM in waveform operating mode.
 * see: http://www.allaboutcircuits.com/projects/dma-digital-to-analog-conversion-with-a-sam4s-microcontroller-the-timer-cou/
 */

void ak5552_start_convertion(uint32_t *mclk)
{
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
	
	/* Configure waveform frequency and duty cycle. */
	sck = sysclk_get_peripheral_bus_hz(TC0);
	uint32_t desired_fs = *mclk;
	rc =  sck / 2 / desired_fs;
	if(rc < 2) rc = 2;
	*mclk = sck / 2 / rc;
	tc_write_rc(TC0, 0, rc);
	ra = rc/2; //(100 - dutycycle) * rc / 100;
	tc_write_ra(TC0, 0, ra);
	//tc_write_rb(TC0, 0, ra);

	/* Enable TC0 Channel 0. */
	tc_start(TC0, 0);
	printf("Start waveform: sysclk = %lu, mclk = %lu (ra=%lu,rc=%lu)\n\r", sck, *mclk, ra, rc);
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
			// buffer1 is done, now reading buffer2, so set buffer1 as next buffer
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet1);
			pdc_active_buffer_number = 2;
			ssc_adc_buffer = ssc_adc_dma_buffer1;
			//fprintf(stdout, "SSC_Handler: buffer 1 done, status = %x\r\n", status);
			//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
		} else if(pdc_active_buffer_number == 2) {
			// buffer2 is done, now reading buffer1, set buffer2 as next
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet2);
			pdc_active_buffer_number = 1;
			ssc_adc_buffer = ssc_adc_dma_buffer2;
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


uint32_t ak5552_dma_read(uint32_t **buffer)
{
	// no buffer ready or buffer has already been read
	if(ssc_adc_buffer == NULL) return(0);
	
	// set the pointer to the finished buffer
	*buffer = ssc_adc_buffer;

	// indicate that the buffer has already been read
	ssc_adc_buffer = NULL;
	
	// return the buffer size in bytes
	return(AK5552_DMA_BUFFER_NSAMPS);
}

/**
 * \brief DMA driver configuration
 */
void ak5552_dma_init(void)
{

	/* Get pointer to UART PDC register base */
	ssc_pdc = ssc_get_pdc_base(SSC);

	/* Initialize PDC data packets for transfer */
	pdc_ssc_packet1.ul_addr = (uint32_t) ssc_adc_dma_buffer1;
	pdc_ssc_packet1.ul_size = AK5552_DMA_BUFFER_NSAMPS;
	pdc_ssc_packet2.ul_addr = (uint32_t) ssc_adc_dma_buffer2;
	pdc_ssc_packet2.ul_size = AK5552_DMA_BUFFER_NSAMPS;

	pdc_active_buffer_number = 1;
	ssc_adc_buffer = NULL;  // no buffer ready

	/* Configure PDC for data receive */
	pdc_rx_init(ssc_pdc, &pdc_ssc_packet1, &pdc_ssc_packet2);

	printf("ak5552_dma_buffer_nsamps %lu \n\r", AK5552_DMA_BUFFER_NSAMPS);
	
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

}

/**
 *  \brief Start DMA .
 */
void ak5552_dma_start(void) 
{
	
	// enable pdc receiver channel requests
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	ssc_enable_rx(SSC);
	//ssc_enable_tx(SSC);
	
}

void ak5552_dma_stop(void)
{
	
	// enable pdc receiver channel requests
	pdc_disable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	ssc_disable_rx(SSC);
	//ssc_enable_tx(SSC);
	
}
