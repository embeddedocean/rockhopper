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

#define AK5552_BUFFER_NBLOCKS (80)

//#define AK5552_DMA_BUFFER_NBYTES (AK5552_DMA_BUFFER_NBLOCKS*SD_MMC_BLOCK_SIZE)
//#define AK5552_DMA_BUFFER_NSAMPS (AK5552_DMA_BUFFER_NBYTES/AK5552_DMA_BYTES_PER_SAMPLE)

#define AK5552_MAX_DMA_BUFFER_NSAMPS (AK5552_BUFFER_NBLOCKS * SD_MMC_BLOCK_SIZE / AK5552_DMA_BYTES_PER_SAMPLE)

// nblocks per buffer is 3/4 the size of the dma buffer because we only need 24 of the 32 bits
//#define AK5552_BUFFER_NBLOCKS (AK5552_DMA_BUFFER_NBLOCKS * AK5552_BYTES_PER_SAMPLE / AK5552_DMA_BYTES_PER_SAMPLE)

#define AK5552_BITS_PER_SAMPLE (24)
#define AK5552_BYTES_PER_SAMPLE (3)

extern int ak5552_init(void);
extern void ak5552_start_convertion(uint32_t *mclk);
extern uint16_t ak5552_dma_read(uint32_t **buffer);
extern int ak5552_dma_init(uint16_t nsamps);
extern void ak5552_dma_start(void);
extern void ak5552_dma_stop(void);

extern void ak5552_get_date(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *week);
extern void ak5552_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec);

#endif /* AK5552_H_ */