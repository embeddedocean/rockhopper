/*
 * ak5552.h
 *
 * Created: 5/6/2016 11:21:35 AM
 *  Author: chris
 */ 

#ifndef AK5552_H_
#define AK5552_H_

//#define AK5552_BITS_PER_SLOT 32
#define AK5552_SLOTS_PER_FRAME 2

#define AK5552_DMA_BITS_PER_SAMPLE (32)
#define AK5552_DMA_BYTES_PER_SAMPLE (4)

#define AK5552_BITS_PER_SAMPLE (24)
#define AK5552_BYTES_PER_SAMPLE (3)

// If the header size is a multiple of the sample sise
// then the buffer will be completely filled by the header and data
// otherwise the buffer will have a few unused bytes at the end   
#define AK5552_HEADER_NBYTES (24)

//#define AK5552_SPI_BUFFER_NBLOCKS (6)
//#define AK5552_SPI_BUFFER_NBYTES (AK5552_SPI_BUFFER_NBLOCKS * SD_MMC_BLOCK_SIZE)

#define AK5552_BUFFER_NBLOCKS (48)
#define AK5552_BUFFER_NBYTES (AK5552_BUFFER_NBLOCKS * SD_MMC_BLOCK_SIZE)
#define AK5552_NUM_SAMPLES ((AK5552_BUFFER_NBYTES - AK5552_HEADER_NBYTES) / AK5552_BYTES_PER_SAMPLE)

#define AK5552_DMA_BUFFER_NBYTES (AK5552_NUM_SAMPLES * AK5552_DMA_BYTES_PER_SAMPLE)

// MCLK modes
#define	AK5552_EXT_MCLK  0              // MCLK supplied externally by some other clock source
#define	AK5552_INT_MCLK  1              // MCLK supplied by TC

// MCLK modes
#define	AK5552_MCLK_32FS  32
#define	AK5552_MCLK_64FS  64
#define	AK5552_MCLK_96FS  96
#define	AK5552_MCLK_128FS  128
#define	AK5552_MCLK_192FS  192
#define	AK5552_MCLK_256FS  256
#define	AK5552_MCLK_384FS  384
#define	AK5552_MCLK_512FS  512

typedef struct {
	uint8_t settings;
	uint8_t bytes_per_samp;
	uint8_t nblocks_per_adc_buffer;
	uint32_t total_nblocks;
	uint16_t nsamps;
	uint32_t fs;
} ak5552_t;

// nblocks per buffer is 3/4 the size of the dma buffer because we only need 24 of the 32 bits

extern int ak5552_init(uint32_t audio_mode,  uint32_t bits, uint8_t mclk_mode, uint32_t mclk_freq_div, uint32_t *fs);
extern int ak5553_config_external_mclk(uint32_t mclk_freq_div, uint32_t *fs);
extern int ak5553_config_internal_mclk(uint32_t mclk_freq_div, uint32_t *fs);

extern void ak5552_start_conversion(void);
extern void ak5552_stop_conversion(void);
extern uint16_t ak5552_read_dma(uint8_t *hdr, uint8_t *data);
extern uint8_t *ak5552_get_dma_buffer(void);
extern uint16_t ak5552_init_dma(void);
extern void ak5552_start_dma(void);
extern void ak5552_stop_dma(void);
extern void ak5552_update_header(uint8_t *hdr, uint8_t chksum);

extern void ak5552_get_date(uint8_t *cent, uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *week);
extern void ak5552_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec);

#endif /* AK5552_H_ */