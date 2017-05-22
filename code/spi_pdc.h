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
 
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#ifndef _SPI_PDC_H
#define _SPI_PDC_H

#include <asf.h>

//#define SPI_Handler     FLEXCOM5_Handler
//#define SPI_IRQn        FLEXCOM5_IRQn

/** Spi Hw ID . */
#define SPI_ID          ID_SPI

/** SPI base address for SPI master mode*/
#define SPI_MASTER_BASE      SPI

/** SPI base address for SPI slave mode, (on different board) */
#define SPI_SLAVE_BASE       SPI

/* Chip select. */
#define SPI_CHIP_SEL 0

#define NONE_CHIP_SELECT_ID 0x0f

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 0

/* Delay before SPCK. */
#define SPI_DLYBS 0x0

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x0

typedef struct {
	uint8_t settings;
	uint8_t bytes_per_samp;
	uint8_t nblocks_per_adc_buffer;
	uint32_t total_nblocks;
	uint16_t nsamps;
	uint32_t fs;
} spi__t;


extern void spi_slave_initialize(void);
status_code_t spi_slave_write_wait(void *buf, uint32_t size);
status_code_t spi_slave_read_wait(void *buf, uint32_t size);
status_code_t spi_slave_read_wait_nopdc(uint8_t *buf, uint32_t size);
status_code_t spi_slave_write(void *buf, uint32_t size);
status_code_t spi_slave_read(void *buf, uint32_t size);

extern void spi_master_initialize(void);
extern void spi_set_clock_configuration(uint32_t clock);
status_code_t spi_master_write(void *buf, uint32_t size);
status_code_t spi_master_write_wait(void *buf, uint32_t size);

extern void spi_slave_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize);
extern void spi_master_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize);

#endif
