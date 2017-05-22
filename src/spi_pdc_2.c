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


/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 40000000;

static uint32_t gs_ul_spi_tbuffer_size;
static uint32_t gs_ul_spi_rbuffer_size;

/* Pointer to UART PDC register base */
Pdc *g_p_spim_pdc, *g_p_spis_pdc;

static uint8_t *gs_uc_spi_m_tbuffer;
static uint8_t *gs_uc_spi_m_rbuffer;
static uint8_t *gs_uc_spi_s_tbuffer;
static uint8_t *gs_uc_spi_s_rbuffer;

static bool gs_uc_spi_drdy = 1;  // active low

/**
 * \brief Set SPI slave transfer.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
void spi_slave_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize)
{
	uint32_t spi_ier;
	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)p_rbuf;
	pdc_spi_packet.ul_size = rsize;
	pdc_rx_init(g_p_spis_pdc, &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(g_p_spis_pdc, &pdc_spi_packet, NULL);
	
	gs_uc_spi_s_tbuffer = (uint8_t *)p_tbuf;
	gs_uc_spi_s_rbuffer = (uint8_t *)p_rbuf;
	gs_ul_spi_tbuffer_size = tsize;
	gs_ul_spi_rbuffer_size = rsize;
	 
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spis_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Transfer done handler is in ISR */
	spi_ier = SPI_IER_NSSR | SPI_IER_RXBUFF;
	spi_enable_interrupt(SPI_SLAVE_BASE, spi_ier);
}

/**
 * \brief Set SPI slave write and wait.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
int spi_slave_write_wait(void *p_tbuf, uint32_t tsize)
{
	uint32_t spi_ier;
	
	//pdc_packet_t pdc_spi_packet;
	//pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	//pdc_spi_packet.ul_size = tsize;
	//pdc_tx_init(g_p_spis_pdc, &pdc_spi_packet, NULL);
	
	g_p_spis_pdc->PERIPH_TPR = (uint32_t)p_tbuf;
	g_p_spis_pdc->PERIPH_TCR = tsize;
	g_p_spis_pdc->PERIPH_TNPR = (uint32_t)NULL;
	g_p_spis_pdc->PERIPH_TNCR = 0;
	
	gs_uc_spi_s_tbuffer = (uint8_t *)p_tbuf;
	gs_ul_spi_tbuffer_size = tsize;
	
	gs_uc_spi_drdy = 0;  // set drdy flag, active low
	ioport_set_pin_level(SPI_DRDY_PIN, gs_uc_spi_drdy);
	
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spis_pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TXBUFE) == 0);
	uint32_t timeout = SPI_TIMEOUT * tsize;
	while(!(SPI_SLAVE_BASE->SPI_SR & SPI_SR_TXBUFE)) {
		if (!timeout--) {
			return ERR_TIMEOUT;
		}
	}
	
	gs_uc_spi_drdy = 1;  // clear drdy flag, active low
	ioport_set_pin_level(SPI_DRDY_PIN, gs_uc_spi_drdy);

	return(STATUS_OK);
}

/**
 * \brief Set SPI slave write.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer. 
 */
int spi_slave_write(void *p_tbuf, uint32_t tsize)
{
	uint32_t spi_ier;

	// wait for active transfer to finish
	uint32_t timeout = SPI_TIMEOUT * tsize;
	while( gs_uc_spi_drdy == 0 ) {
		if (!timeout--) {
			printf("spi_slave_write: timeout\r\n");
			return ERR_TIMEOUT;
		}
	};
	
	//pdc_packet_t pdc_spi_packet;
	//pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	//pdc_spi_packet.ul_size = tsize;
	//pdc_tx_init(g_p_spis_pdc, &pdc_spi_packet, NULL);
	
	g_p_spis_pdc->PERIPH_TPR = (uint32_t)p_tbuf;
	g_p_spis_pdc->PERIPH_TCR = tsize;
	g_p_spis_pdc->PERIPH_TNPR = (uint32_t)NULL;
	g_p_spis_pdc->PERIPH_TNCR = 0;
	
	gs_uc_spi_s_tbuffer = (uint8_t *)p_tbuf;
	gs_ul_spi_tbuffer_size = tsize;
	
	gs_uc_spi_drdy = 0;  // set drdy flag, active low
	ioport_set_pin_level(SPI_DRDY_PIN, gs_uc_spi_drdy);
	
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spis_pdc, PERIPH_PTCR_TXTEN);

	/* Transfer done handler is in ISR */
	//spi_ier = SPI_IER_NSSR | SPI_IER_TXBUFE;
	spi_ier = SPI_IER_TXBUFE;
	spi_enable_interrupt(SPI_SLAVE_BASE, spi_ier);

	return(STATUS_OK);
}

/**
 * \brief Interrupt handler for the SPI slave.
 */
void SPI_Handler(void)
{
	uint32_t status;
	
	status = spi_read_status(SPI_SLAVE_BASE);

//	if(status & SPI_SR_NSSR) {
		//if ( status & SPI_SR_RXBUFF ) {
		//	spi_slave_transfer(gs_uc_spi_s_tbuffer, gs_ul_spi_tbuffer_size, gs_uc_spi_s_rbuffer, gs_ul_spi_rbuffer_size);
		//}
		if ( status & SPI_SR_TXBUFE ) {
			/* Disable the TX PDC transfer requests */
			pdc_disable_transfer(g_p_spim_pdc, PERIPH_PTCR_TXTDIS);
			gs_uc_spi_drdy = 1;  // clear drdy flag, active low
			ioport_set_pin_level(SPI_DRDY_PIN, gs_uc_spi_drdy);
		}
//	}
}

/**
 * \brief Initialize SPI as slave.
 */
void spi_slave_initialize(void)
{
	/* Get pointer to SPI slave PDC register base */
	g_p_spis_pdc = spi_get_pdc_base(SPI_SLAVE_BASE);
	
	//uint32_t i;
	//for (i = 0; i < COMM_BUFFER_SIZE; i++) {
	//	gs_uc_spi_s_tbuffer[i] = i;
	//}
	
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
	spi_enable(SPI_SLAVE_BASE);
	
	pdc_disable_transfer(g_p_spis_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	
	//spi_slave_transfer(gs_uc_spi_s_tbuffer, gs_ul_spi_tbuffer_size, gs_uc_spi_s_rbuffer, gs_ul_spi_rbuffer_size);
			
	/* Configure SPI interrupts for slave only. */
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);

	// spi data ready pin
	ioport_set_pin_dir(SPI_DRDY_PIN, IOPORT_DIR_OUTPUT);
	gs_uc_spi_drdy = 1;  // clear drdy flag, active low
	ioport_set_pin_level(SPI_DRDY_PIN, gs_uc_spi_drdy);

	ioport_set_pin_dir(SPI_BUSY_PIN, IOPORT_DIR_INPUT);

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
	g_p_spim_pdc = spi_get_pdc_base(SPI_MASTER_BASE);

	/* Configure an SPI peripheral. */
	pmc_enable_periph_clk(SPI_ID);
	sysclk_enable_peripheral_clock(ID_SPI);

	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_SEL);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL, (sysclk_get_cpu_hz() / gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS, SPI_DLYBCT);
	spi_enable(SPI_MASTER_BASE);
	
	pdc_disable_transfer(g_p_spim_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	
	// spi data ready pin
	ioport_set_pin_dir(SPI_DRDY_PIN, IOPORT_DIR_OUTPUT);
	gs_uc_spi_drdy = 1;  // clear drdy flag, active low
	ioport_set_pin_level(SPI_DRDY_PIN, gs_uc_spi_drdy);

	ioport_set_pin_dir(SPI_BUSY_PIN, IOPORT_DIR_INPUT);

}

/**
 * \brief Set the specified SPI clock configuration.
 *
 * \param configuration  Index of the configuration to set.
 */
void spi_set_clock_configuration(uint32_t clock)
{
	gs_ul_spi_clock = clock;
	printf("Setting SPI clock #%lu \r\n", (unsigned long)gs_ul_spi_clock);
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
	pdc_rx_init(g_p_spim_pdc, &pdc_spi_packet, NULL);

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(g_p_spim_pdc, &pdc_spi_packet, NULL);

	gs_uc_spi_m_tbuffer = (uint8_t *)p_tbuf;
	gs_uc_spi_m_rbuffer = (uint8_t *)p_rbuf;
	gs_ul_spi_tbuffer_size = tsize;
	gs_ul_spi_rbuffer_size = rsize;

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_p_spim_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	while((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RXBUFF) == 0);
	
	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_p_spim_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
}

int spi_master_write(void *p_tbuf, uint32_t tsize)
{
	pdc_packet_t pdc_spi_packet;

	pdc_spi_packet.ul_addr = (uint32_t)p_tbuf;
	pdc_spi_packet.ul_size = tsize;
	pdc_tx_init(g_p_spim_pdc, &pdc_spi_packet, NULL);

	gs_uc_spi_m_tbuffer = (uint8_t *)p_tbuf;
	gs_ul_spi_tbuffer_size = tsize;

	/* Enable the TX PDC transfer requests */
	pdc_enable_transfer(g_p_spim_pdc, PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	//while((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TXBUFE) == 0);
	uint32_t timeout = SPI_TIMEOUT * tsize;
	while(!(SPI_MASTER_BASE->SPI_SR & SPI_SR_TXBUFE)) {
		if (!timeout--) {
			return ERR_TIMEOUT;
		}
	}
	
	/* Disable the TX PDC transfer requests */
	pdc_disable_transfer(g_p_spim_pdc, PERIPH_PTCR_TXTDIS);
	
	return(STATUS_OK);
}

