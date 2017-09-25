/**
 *
 */


#ifndef _SD_CARD_H_
#define _SD_CARD_H_

#include <asf.h>
#include <string.h>

//#include "sd_mmc_mem.h"
#include "conf_sd_mmc.h"

#define SD_CARD_START_BLOCK 32

typedef struct {
	uint8_t slot;
	uint8_t state;
	uint8_t type;
	uint8_t version;
	uint32_t capacity;      // capacity in bytes
	uint32_t block_size;    // maximum block size
	uint32_t first_block;    // addr of first block
	uint32_t last_block;    // addr of last block
	//uint32_t write_start_addr;    // addr of current write block
	//uint32_t read_start_addr;    // addr of current read block
} sd_card_t;


extern int sd_card_init(sd_card_t *sd);
extern void sd_card_get_info(sd_card_t *sd);
extern int sd_card_write_raw(sd_card_t *sd, uint8_t *buffer, uint16_t nblocks, uint32_t addr);
extern int sd_card_read_raw(sd_card_t *sd, uint8_t *buffer, uint16_t nblocks, uint32_t addr);

//extern int sd_card_init_fat(void);
//extern FRESULT sd_card_open_file(char *filename);
//extern int sd_card_write_file(uint8_t *buffer, uint16_t nblocks);

#endif