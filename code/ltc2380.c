/**
 * \brief  ltc2380 ssc interface.
 */

#include "ltc2380.h"

#define LTC2380_TCONV (0.0000004)
#define LTC2380_FSYNC_SIZE (2)
#define LTC2380_FSYNC_DELAY (2)

/** The SSC interrupt IRQ priority. */
#define LTC2380_IRQ_PRIO 1

/* DMA channel */
//#define LTC2380_DMA_CH 0

/* Pdc transfer buffer */
COMPILER_WORD_ALIGNED uint32_t ltc2380_dma_buffer1[LTC2380_DMA_BUFFER_SIZE];
COMPILER_WORD_ALIGNED uint32_t ltc2380_dma_buffer2[LTC2380_DMA_BUFFER_SIZE];

/* PDC data packet for transfer */
pdc_packet_t pdc_ssc_packet1;
pdc_packet_t pdc_ssc_packet2;

/* Pointer to SSC PDC register base */
Pdc *ssc_pdc;

/* active buffer number, that is the number o the buffer being read now */
uint8_t pdc_active_buffer_number = 1;

// finished buffer
uint32_t *ltc2380_buffer;
int ltc2380_buffer_number = 0;

extern uint8_t Verbose;

int ltc2380_init(uint32_t fs)
{
    if( (fs <= 1) || (fs > 2000000) ) {
		if(Verbose) fprintf(stdout, "init_ltc2380: invalid sampling frequency\n\r");
		return(-1);
	}
	
	/* Initialize the local variable. */
	clock_opt_t rx_clk_option;
	data_frame_opt_t rx_data_frame_option;
	memset((uint8_t *)&rx_clk_option, 0, sizeof(clock_opt_t));
	memset((uint8_t *)&rx_data_frame_option, 0, sizeof(data_frame_opt_t));

	/* Initialize the SSC module */
	pmc_enable_periph_clk(ID_SSC);
	ssc_reset(SSC);

	uint32_t mck = sysclk_get_cpu_hz();
	float T = 1.0 / (float)fs;
	float tcnv = 0.0000005;
	//uint32_t rsck = (uint32_t)((float)LTC2380_SAMPLE_SIZE/(T-tcnv) + 0.5);  // samples per second
	uint32_t rsck = 12000000;  // samples per second
	uint32_t rsck_div = mck / (2 * rsck);

	if(Verbose) fprintf(stdout, "mck = %lu, rsck = %lu, rsck_div = %lu \n\r", mck, rsck, rsck_div);

	SSC->SSC_CMR = SSC_CMR_DIV(rsck_div);
	
	/* Transmitter clock mode configuration. */
//	tx_clk_option.ul_cks = SSC_TCMR_CKS_MCK;
//	tx_clk_option.ul_cko = SSC_TCMR_CKO_CONTINUOUS;
//	tx_clk_option.ul_cki = 0;
//	tx_clk_option.ul_ckg = 0;
//	tx_clk_option.ul_start_sel = SSC_TCMR_START_CONTINUOUS;
//	tx_clk_option.ul_sttdly = 0;
//	tx_clk_option.ul_period = 0;
	
	/* Transmitter frame mode configuration. */
//	tx_data_frame_option.ul_datlen = LTC2380_SAMPLE_SIZE - 1;
//	tx_data_frame_option.ul_msbf = SSC_TFMR_MSBF;
//	tx_data_frame_option.ul_datnb = 0;
//	tx_data_frame_option.ul_fslen = fslen;
//	tx_data_frame_option.ul_fslen_ext = 0;
//	tx_data_frame_option.ul_fsos = SSC_TFMR_FSOS_LOW;
//	tx_data_frame_option.ul_fsedge = SSC_TFMR_FSEDGE_POSITIVE;
	
	/* Configure the SSC transmitter. */
	//ssc_set_transmitter(SSC, &tx_clk_option, &tx_data_frame_option);

	/* Receiver clock mode configuration. */
	rx_clk_option.ul_cks = SSC_RCMR_CKS_MCK;  // select divided clock source 
	rx_clk_option.ul_cko = SSC_RCMR_CKO_TRANSFER; // Receive Clock only during data transfers, RK pin is an output
	//rx_clk_option.ul_cko = SSC_RCMR_CKO_CONTINUOUS; // Receive Clock only during data transfers, RK pin is an output
	rx_clk_option.ul_cki = 0; // sampled on Receive Clock falling edge.
	rx_clk_option.ul_ckg = SSC_RCMR_CKG_EN_RF_LOW; // clock gating selection, enable clock only when RF is low
	rx_clk_option.ul_start_sel = SSC_RCMR_START_RF_LOW; // Detection of a low level on RF signal	
	//rx_clk_option.ul_start_sel = SSC_RCMR_START_RF_FALLING; // Detection of a falling edge on RF signal
	//rx_clk_option.ul_start_sel = SSC_RCMR_START_CONTINUOUS; // receive start selection
	rx_clk_option.ul_sttdly = 0; // num clock cycles delay between start event and reception 
	rx_clk_option.ul_period = 0; //ul_rfck_div;  // length of frame in clock cycles
	
	/* Receiver frame mode configuration. */
	rx_data_frame_option.ul_datlen = LTC2380_SAMPLE_SIZE - 1; // number of bits per data word, should be 0 to 31
	rx_data_frame_option.ul_msbf = SSC_RFMR_MSBF;  // MSB
	rx_data_frame_option.ul_datnb = 0;  //  number of data words per frame, should be 0 to 15.
	rx_data_frame_option.ul_fslen = 15; // Frame Sync. length should be 0 to 15
	rx_data_frame_option.ul_fslen_ext = 1; // Frame Sync. length extension field, should be 0 to 15.
	rx_data_frame_option.ul_fsos = SSC_RFMR_FSOS_NONE; // Frame Sync. is an input
	rx_data_frame_option.ul_fsedge = SSC_RFMR_FSEDGE_NEGATIVE; // Frame Sync. edge detection
	
	/* Configure the SSC receiver. */
	ssc_set_receiver(SSC, &rx_clk_option, &rx_data_frame_option);

	//ssc_enable_tx(SSC);
	ssc_enable_rx(SSC);

	/* TESTING ONLY - Enable the loop mode. */
	//ssc_set_loop_mode(SSC);

	return(1);
}


/**
 * \brief Configure TC TC_CHANNEL_WAVEFORM in waveform operating mode.
 * see: http://www.allaboutcircuits.com/projects/dma-digital-to-analog-conversion-with-a-sam4s-microcontroller-the-timer-cou/
 */

void ltc2380_start_convertion(uint32_t fs)
{
	uint32_t ra, rc;
	uint16_t dutycycle = 1; /** Duty cycle in percent (positive).*/

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
		| TC_CMR_WAVSEL_UP_RC	//count up, reset on RC match
		| TC_CMR_ACPA_TOGGLE	//toggle TIOA on RA match
		| TC_CMR_ACPC_TOGGLE	//toggle TIOA on RC match
	);
	
	/* Configure waveform frequency and duty cycle. */
	rc = sysclk_get_peripheral_bus_hz(TC0) / 2 / fs;
	tc_write_rc(TC0, 0, rc);
	ra = (100 - dutycycle) * rc / 100;
	tc_write_ra(TC0, 0, ra);
	//tc_write_rb(TC0, 0, ra);

	/* Enable TC0 Channel 0. */
	tc_start(TC0, 0);
	if(Verbose) printf("Start waveform: Frequency = %lu (ra=%lu,rc=%lu)\n\r", fs, ra, rc);
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
			ltc2380_buffer = ltc2380_dma_buffer1;
			ltc2380_buffer_number = 1;
			//fprintf(stdout, "SSC_Handler: buffer 1 done, status = %x\r\n", status);
			//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
		} else {
			// buffer2 is done, now reading buffer1, set buffer2 as next
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet2);
			pdc_active_buffer_number = 1;
			ltc2380_buffer = ltc2380_dma_buffer2;
			ltc2380_buffer_number = 2;
			//ioport_set_pin_level(SSC_ADC_BUF_PIN, 1);
			//fprintf(stdout, "SSC_Handler: buffer 2 done, status = %x\r\n", status);
		} 
		//else {
		//	fprintf(stdout, "SSC_Handler: confused, status = %x\r\n", status);
		//}
		
//		fprintf(stdout, "pdc_current_buffer = %d, ", g_pdc_current_buffer);
//		fprintf(stdout, "rx_cnt=%ld, rx_ptr=%ld, ", 
//			pdc_read_rx_counter(g_p_ssc_pdc), pdc_read_rx_ptr(g_p_ssc_pdc));
//		fprintf(stdout, "rx_next_cnt=%ld, rx_next_ptr=%ld \n\r", 
//			pdc_read_rx_next_counter(g_p_ssc_pdc), pdc_read_rx_next_ptr(g_p_ssc_pdc));

	}

	// RXBUFF flag is set when both PERIPH_RCR and the PDC Receive Next Counter register (PERIPH_RNCR) reach zero.
	if ((status & SSC_SR_RXBUFF) == SSC_SR_RXBUFF) {
		if(Verbose) fprintf(stdout, "SSC_Handler: buffer overflow\r\n");
		// buffer overflow
	}

}


int ltc2380_dma_read(uint32_t **buffer)
{
	// no buffer ready or buffer has already been read
	if(ltc2380_buffer == NULL) return(0);
	
	// set the pointer to the finished buffer
	*buffer = ltc2380_buffer;

	// indicate that the buffer has already been read
	ltc2380_buffer = NULL;
	
	// return the number of the finished buffer
	return(LTC2380_DMA_BUFFER_SIZE);
}

/**
 * \brief DMA driver configuration
 */
void ltc2380_dma_init(void)
{
	/* Get pointer to UART PDC register base */
	ssc_pdc = ssc_get_pdc_base(SSC);

	/* Initialize PDC data packet for transfer */
	pdc_ssc_packet1.ul_addr = (uint32_t) ltc2380_dma_buffer1;
	pdc_ssc_packet1.ul_size = LTC2380_DMA_BUFFER_SIZE;
	pdc_ssc_packet2.ul_addr = (uint32_t) ltc2380_dma_buffer2;
	pdc_ssc_packet2.ul_size = LTC2380_DMA_BUFFER_SIZE;

	pdc_active_buffer_number = 1;
	ltc2380_buffer = NULL;  // no buffer ready
	ltc2380_buffer_number = 0; // no buffer ready

	/* Configure PDC for data receive */
	pdc_rx_init(ssc_pdc, &pdc_ssc_packet1, &pdc_ssc_packet2);
	
	//fprintf(stdout, "rx_cnt=%lu, rx_ptr=%x, ", pdc_read_rx_counter(ssc_pdc), pdc_read_rx_ptr(ssc_pdc));
	//fprintf(stdout, "rx_next_cnt=%lu, rx_next_ptr=%x \n\r", pdc_read_rx_next_counter(ssc_pdc), pdc_read_rx_next_ptr(ssc_pdc));

	/* Configure the RX End of Reception interrupt. */
	ssc_enable_interrupt(SSC, SSC_IER_ENDRX);

	/* Enable SSC interrupt line from the core */
	NVIC_DisableIRQ(SSC_IRQn);
	NVIC_ClearPendingIRQ(SSC_IRQn);
	NVIC_SetPriority(SSC_IRQn, LTC2380_IRQ_PRIO);
	NVIC_EnableIRQ(SSC_IRQn);

	/* Enable PDC receive transfers */
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	ioport_set_pin_dir(SSC_ADC_BUF_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);

}

/**
 *  \brief Start DMA .
 */
void ltc2380_dma_start(void) 
{
	
	// enable pdc receiver channel requests
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	ssc_enable_rx(SSC);
	//ssc_enable_tx(SSC);
	
	//int m, n;
    //for(n = 0; n < 10; n++) {
	//	for(m = 0; m < DMA_BUFFER_SIZE; m++) {
	//		ssc_write(SSC, (uint16_t)m);
	//	}
	//}
}

void ltc2380_dma_stop(void) 
{	
	ssc_disable_rx(SSC);
	pdc_disable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	ssc_disable_tx(SSC);
	ssc_disable_interrupt(SSC, SSC_IDR_ENDRX);
	NVIC_DisableIRQ(SSC_IRQn);
}


/*
// PWM frequency in Hz 
#define PWM_FREQUENCY  1000
// PWM period value 
#define PERIOD_VALUE   100

int ltc2380_start_convertion_pwm(uint32_t fs)
{

// Configure PIO Pins for TC 
ioport_set_pin_mode(EXT3_PIN_7, IOPORT_MODE_MUX_B);
// Disable IO to enable peripheral mode) 
ioport_disable_pin(EXT3_PIN_7);

// Enable PWMC peripheral clock 
pmc_enable_periph_clk(ID_PWM);

// Disable PWM channel 
pwm_channel_disable(PWM, EXT1_PWM_CHANNEL);

// Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) 
pwm_clock_t clock = {
.ul_clka = fs,
.ul_clkb = 0,
.ul_mck = sysclk_get_cpu_hz()
};
pwm_init(PWM, &clock);

//PWM->PWM_CLK = 1;

pwm_channel_t channel = {
// Use PWM clock A as source clock
.ul_prescaler = PWM_CMR_CPRE_CLKA,
// Period value 
.ul_period = PERIOD_VALUE,
};

// Initialize PWM channel 
channel.channel = EXT1_PWM_CHANNEL;
pwm_channel_init(PWM, &channel);

// Enable PWM channel 
pwm_channel_enable(PWM, EXT1_PWM_CHANNEL);

return(1);
}

*/
