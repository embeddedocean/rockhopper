/**
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#include "asf.h"
#include "conf_board.h"
#include "spi_pdc.h"
#include "delay.h"
#include "board.h"


/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 20000000;

/* Pointer to UART PDC register base */
Pdc *spi_master_pdc;
Pdc *spi_slave_pdc;

volatile uint8_t spi_slave_transmitting = 0; //
volatile uint8_t spi_slave_receiving = 0; // ready to send

volatile uint8_t spi_slave_receive_buffer_full = 0;
volatile uint8_t spi_slave_transmit_buffer_empty = 1;
 
extern uint8_t Verbose; 
 
/**
 * \brief Initialize SPI as slave.
 */
void spi_slave_initialize(void)
{
	// setup pins
// 	ioport_set_pin_dir(SPI_MISO_GPIO, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_mode(SPI_MISO_GPIO, SPI_MISO_FLAGS);
// 	
// 	ioport_set_pin_dir(SPI_MOSI_GPIO, IOPORT_DIR_INPUT);
// 	ioport_set_pin_mode(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
// 	
// 	ioport_set_pin_dir(SPI_SPCK_GPIO, IOPORT_DIR_INPUT);
// 	ioport_set_pin_mode(SPI_SPCK_GPIO, SPI_SPCK_FLAGS);
// 		
 	ioport_set_pin_dir(SPI_NPCS0_GPIO, IOPORT_DIR_INPUT);
 	ioport_set_pin_mode(SPI_NPCS0_GPIO, SPI_NPCS0_FLAGS);

	/* Get pointer to SPI slave PDC register base */
	spi_slave_pdc = spi_get_pdc_base(SPI_SLAVE_BASE);
	
	/* Configure an SPI peripheral. */
	pmc_enable_periph_clk(SPI_ID);
	
	spi_disable(SPI_SLAVE_BASE);
	spi_reset(SPI_SLAVE_BASE);
	spi_set_slave_mode(SPI_SLAVE_BASE);
	spi_disable_mode_fault_detect(SPI_SLAVE_BASE);
	spi_set_peripheral_chip_select_value(SPI_SLAVE_BASE, SPI_CHIP_SEL);
	spi_set_clock_polarity(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
		
	//spi_set_fixed_peripheral_select(SPI_SLAVE_BASE);
	//spi_disable_peripheral_select_decode(SPI_SLAVE_BASE);
	//spi_set_peripheral_chip_select_value(SPI_SLAVE_BASE, SPI_CHIP_SEL);
	//spi_configure_cs_behavior(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CS_RISE_NO_TX);
		
	/* Configure SPI interrupts for slave only. */
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);

	spi_enable(SPI_SLAVE_BASE);

	pdc_disable_transfer(spi_slave_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	
	//spi_slave_transfer(gs_uc_spi_s_tbuffer, gs_ul_spi_tbuffer_size, gs_uc_spi_s_rbuffer, gs_ul_spi_rbuffer_size);

	// slave READY pin is an output
	ioport_set_pin_dir(SPI_SLAVE_READY_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
	
	// master ready pin is an input
	ioport_set_pin_dir(SPI_MASTER_READY_PIN, IOPORT_DIR_INPUT);
//	ioport_set_pin_mode(SPI_MASTER_READY_PIN, IOPORT_MODE_OPEN_DRAIN);
	ioport_set_pin_mode(SPI_MASTER_READY_PIN, IOPORT_MODE_PULLDOWN);

	spi_slave_transmitting = 0;
	spi_slave_receiving = 0;
	spi_slave_receive_buffer_full = 0;
	spi_slave_transmit_buffer_empty = 1;

}

/**
 * \brief Set SPI slave write and wait.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
status_code_t spi_slave_write_wait(void *buf, uint32_t size)
{
	// if active transfer in progress
	if( spi_slave_transmitting ) {
//		return STATUS_ERR_BUSY;
	};
	
	spi_slave_pdc->PERIPH_TPR = (uint32_t)buf;
	spi_slave_pdc->PERIPH_TCR = size;
	spi_slave_pdc->PERIPH_TNPR = (uint32_t)NULL;
	spi_slave_pdc->PERIPH_TNCR = 0;
	
	// set RTS - slave is ready to send to master.
	//ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_HIGH);

	// check if master is ready to receive
	uint32_t timeout = SPI_TIMEOUT * size;
	while( ioport_get_pin_level(SPI_MASTER_READY_PIN) == IOPORT_PIN_LEVEL_LOW ) {
		if (!timeout--) {
			ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
			return STATUS_ERR_BUSY;
		}
	}

	spi_slave_transmitting = 1;
	spi_slave_transmit_buffer_empty = 0;

	/* Enable the TX PDC transfer requests */
	pdc_enable_transfer(spi_slave_pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	status_code_t ret = STATUS_OK;
	timeout = SPI_TIMEOUT * size;
	while(!(SPI_SLAVE_BASE->SPI_SR & SPI_SR_TXBUFE)) {
		if (!timeout--) {
			ret = STATUS_ERR_TIMEOUT;
			break;
		}
	}
	
	pdc_disable_transfer(spi_slave_pdc, PERIPH_PTCR_TXTDIS);

	// clear RTS
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
	spi_slave_transmitting = 0;
	spi_slave_transmit_buffer_empty = 1;

	return(ret);
}

/**
 * \brief Set SPI slave read and wait.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
status_code_t spi_slave_read_wait(void *buf, uint32_t size)
{	
	// if active transfer in progress
	if( spi_slave_receiving ) {
//		return STATUS_ERR_BUSY;
	};
	
	spi_slave_pdc->PERIPH_RPR = (uint32_t)buf;
	spi_slave_pdc->PERIPH_RCR = size;
	spi_slave_pdc->PERIPH_RNPR = (uint32_t)NULL;
	spi_slave_pdc->PERIPH_RNCR = 0;
	
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(spi_slave_pdc, PERIPH_PTCR_RXTEN);

	// set RTR  - slave is ready to receive data from the master.
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_HIGH);

	// check if master is ready
	uint32_t timeout = SPI_TIMEOUT * size;
	while( ioport_get_pin_level(SPI_MASTER_READY_PIN) == IOPORT_PIN_LEVEL_LOW ) {
		if (!timeout--) {
			ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
			return STATUS_ERR_BUSY;
		}
	}

	spi_slave_receiving = 1;

	/* Waiting transfer done*/
	status_code_t ret = STATUS_OK;
	timeout = SPI_TIMEOUT * size;
	while(!(SPI_SLAVE_BASE->SPI_SR & SPI_SR_RXBUFF)) {
		if (!timeout--) {
			ret = STATUS_ERR_TIMEOUT;
			break;
		}
	}
	
	pdc_disable_transfer(spi_slave_pdc, PERIPH_PTCR_RXTDIS);

	// clear RTR 
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
	spi_slave_receiving = 0;

	return(ret);
}

status_code_t spi_slave_read_wait_nopdc(uint8_t *buf, uint32_t size)
{
	// if active transfer in progress
	if( spi_slave_receiving ) {
		return STATUS_ERR_BUSY;
	};
	
	// set RTR  - slave is ready to receive data from the master.	
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_HIGH);
	spi_slave_receiving = 1;

	status_code_t ret = STATUS_OK;
	for(uint32_t n = 0; n < size; n++) {
		uint32_t timeout = SPI_TIMEOUT;
		while (!(SPI_SLAVE_BASE->SPI_SR & SPI_SR_RDRF)) { 
			if (!timeout--) {
				ret = STATUS_ERR_TIMEOUT;
				break;
			}		
		}
		buf[n] = SPI_SLAVE_BASE->SPI_RDR;	
	}

	// clear RTR
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
	spi_slave_receiving = 0;

	return(ret);
}

/**
 * \brief Set SPI slave write.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
status_code_t spi_slave_write(void *buf, uint32_t size)
{
	// if active transfer in progress
	if( spi_slave_transmitting ) {
		return STATUS_ERR_BUSY;
	};
	
	spi_slave_pdc->PERIPH_TPR = (uint32_t)buf;
	spi_slave_pdc->PERIPH_TCR = size;
	spi_slave_pdc->PERIPH_TNPR = (uint32_t)NULL;
	spi_slave_pdc->PERIPH_TNCR = 0;
	
	// set RTS - slave is ready to send to master.
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_HIGH);
	spi_slave_transmitting = 1;
	
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(spi_slave_pdc, PERIPH_PTCR_TXTEN);

	/* Transfer done handler is in ISR */
	uint32_t spi_ier = SPI_IER_NSSR | SPI_IER_TXBUFE;
	//spi_ier = SPI_IER_TXBUFE;
	spi_enable_interrupt(SPI_SLAVE_BASE, spi_ier);

	return(STATUS_OK);
}

/**
 * \brief Set SPI slave read.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
status_code_t spi_slave_read(void *buf, uint32_t size)
{
	uint32_t spi_ier;

// how to start???
	if( spi_slave_receive_buffer_full ) {
		spi_slave_receive_buffer_full = 0;  // reset flag to start dma on next call
		return STATUS_OK;
	};

	// if active transfer in progress
	if( spi_slave_receiving ) { 
		return STATUS_ERR_BUSY;
	};
	
	spi_slave_pdc->PERIPH_RPR = (uint32_t)buf;
	spi_slave_pdc->PERIPH_RCR = size;
	spi_slave_pdc->PERIPH_RNPR = (uint32_t)NULL;
	spi_slave_pdc->PERIPH_RNCR = 0;
	
	// set RTR  - slave is ready to receive data from the master.
	ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_HIGH);	
	spi_slave_receiving = 1;

	spi_slave_receive_buffer_full = 0;
	
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(spi_slave_pdc, PERIPH_PTCR_TXTEN);

	/* Transfer done handler is in ISR */
	spi_ier = SPI_IER_NSSR | SPI_IER_RXBUFF;
	spi_enable_interrupt(SPI_SLAVE_BASE, spi_ier);

	return(STATUS_ERR_BUSY);
}

/**
 * \brief Interrupt handler for the SPI slave.
 */
void SPI_Handler(void)
{
	uint32_t status;
	
	status = spi_read_status(SPI_SLAVE_BASE);

	if(status & SPI_SR_NSSR) {
		if ( status & SPI_SR_RXBUFF ) {
			pdc_disable_transfer(spi_slave_pdc, PERIPH_PTCR_RXTDIS);
			// clear RTR
			spi_slave_receiving = 0;
			spi_slave_receive_buffer_full = 1;
			ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
		}
		if ( status & SPI_SR_TXBUFE ) {
			/* Disable the TX PDC transfer requests */
			pdc_disable_transfer(spi_slave_pdc, PERIPH_PTCR_TXTDIS);
			// clear RTS
			spi_slave_transmitting = 0;
			spi_slave_transmit_buffer_empty = 1;
			ioport_set_pin_level(SPI_SLAVE_READY_PIN, IOPORT_PIN_LEVEL_LOW);
		}
	}
}


/**
 * \brief Initialize SPI as master.
 */
void spi_master_initialize(void)
{
	
	//uint32_t i;
	//for (i = 0; i < COMM_BUFFER_SIZE; i++) {
	//	gs_uc_spi_m_tbuffer[i] = i;
	//}
	
	/* Get pointer to SPI master PDC register base */
	spi_master_pdc = spi_get_pdc_base(SPI_MASTER_BASE);

	/* Configure an SPI peripheral. */
#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);
#else
	/* Configure an SPI peripheral. */
	pmc_enable_periph_clk(SPI_ID);
	sysclk_enable_peripheral_clock(SPI_ID);
#endif

	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	spi_disable_loopback(SPI_MASTER_BASE);
	//spi_set_fixed_peripheral_select(SPI_MASTER_BASE);
	spi_disable_peripheral_select_decode(SPI_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_SEL);
	spi_configure_cs_behavior(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CS_RISE_NO_TX);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL, (sysclk_get_cpu_hz() / gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS, SPI_DLYBCT);
	
	spi_enable(SPI_MASTER_BASE);
	
	pdc_disable_transfer(spi_master_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	
	// slave ready is an input 
	ioport_set_pin_dir(SPI_SLAVE_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SPI_SLAVE_READY_PIN, IOPORT_MODE_OPEN_DRAIN);

	// master READY is an output
	ioport_set_pin_dir(SPI_MASTER_READY_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SPI_MASTER_READY_PIN, IOPORT_PIN_LEVEL_LOW);
	
	// slave ready is an input
	ioport_set_pin_dir(SPI_SLAVE_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SPI_SLAVE_READY_PIN, IOPORT_MODE_OPEN_DRAIN);

}

/**
 * \brief Set the specified SPI clock configuration.
 *
 * \param configuration  Index of the configuration to set.
 */
void spi_set_clock_configuration(uint32_t clock)
{
	gs_ul_spi_clock = clock;
	if(Verbose) printf("Setting SPI clock #%lu \r\n", (unsigned long)gs_ul_spi_clock);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL, (sysclk_get_cpu_hz() / gs_ul_spi_clock));
}

/**
 * \brief Perform SPI master transfer.
 *
 * \param pbuf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
void spi_master_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize)
{
	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)p_rbuf;
	pdc_spi_packet.ul_size = rsize;
	pdc_rx_init(spi_master_pdc, &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(spi_master_pdc, &pdc_spi_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(spi_master_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RXBUFF) == 0);
	
	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(spi_master_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
}

status_code_t spi_master_write(void *buf, uint32_t size)
{
	// check if slave is ready to receive
	if( ioport_get_pin_level(SPI_SLAVE_READY_PIN) == IOPORT_PIN_LEVEL_LOW ) {
		return STATUS_ERR_BUSY;
	}

	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)buf;
	pdc_spi_packet.ul_size = size;
	pdc_tx_init(spi_master_pdc, &pdc_spi_packet, NULL);

	// assert chip select
	//spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_SEL);
	
	/* Enable the TX PDC transfer requests */
	pdc_enable_transfer(spi_master_pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	//while((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TXBUFE) == 0);
	uint32_t timeout = SPI_TIMEOUT * size;
	while(!(SPI_MASTER_BASE->SPI_SR & SPI_SR_TXBUFE)) {
		if (!timeout--) {
			return STATUS_ERR_TIMEOUT;
		}
	}

	// Assert all lines; no peripheral is selected.
	//spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, NONE_CHIP_SELECT_ID);
	
	/* Disable the TX PDC transfer requests */
	pdc_disable_transfer(spi_master_pdc, PERIPH_PTCR_TXTDIS);
	
	return(STATUS_OK);
}

status_code_t spi_master_write_wait(void *buf, uint32_t size)
{
	
	// check if slave is ready to receive
	uint32_t timeout = SPI_TIMEOUT * size;
	while( ioport_get_pin_level(SPI_SLAVE_READY_PIN) == IOPORT_PIN_LEVEL_LOW ) {
		if (!timeout--) {
			return STATUS_ERR_TIMEOUT;
		}
	}

	return( spi_master_write(buf, size));

}
