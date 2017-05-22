/**
 *  AD7690.h
 */

#ifndef _LTC2380_H_
#define _LTC2380_H_

#include <asf.h>
#include <string.h>

#define LTC2380_DMA_BUFFER_SIZE (64*SD_MMC_BLOCK_SIZE/4)
//#define LTC2380_DMA_BUFFER_SIZE (128*SD_MMC_BLOCK_SIZE/4)

/** ssc adc sample size in bits */
#define LTC2380_SAMPLE_SIZE (24)

extern int ltc2380_init(uint32_t fs);
extern void ltc2380_dma_init(void);
extern void ltc2380_dma_start(void);
extern void ltc2380_dma_stop(void);
extern int ltc2380_dma_read(uint32_t **buffer);
extern void ltc2380_start_convertion(uint32_t fs);

#endif