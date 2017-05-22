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

//#define SPI_Handler     SPI_Handler
//#define SPI_IRQn        SPI_IRQn

/** Spi Hw ID . */
#define SPI_ID          ID_SPI

/** SPI base address for SPI master mode*/
#define SPI_MASTER_BASE      SPI
/** SPI base address for SPI slave mode, (on different board) */
#define SPI_SLAVE_BASE       SPI

/* Chip select. */
#define SPI_CHIP_SEL 0

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 0

/* Delay before SPCK. */
#define SPI_DLYBS 0x0

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x0


extern void spi_master_initialize(void);
extern void spi_set_clock_configuration(uint32_t clock);
extern void spi_master_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize);
extern int spi_master_write(void *p_tbuf, uint32_t tsize);

extern void spi_slave_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf, uint32_t rsize);
extern void spi_slave_initialize(void);
extern int spi_slave_write(void *p_tbuf, uint32_t tsize);
extern int spi_slave_write_wait(void *p_tbuf, uint32_t tsize);

#endif
