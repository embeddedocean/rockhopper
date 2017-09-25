/**
 *
 */


#include <asf.h>
#include <string.h>

#include "sd_card.h"

/* IRQ priority for PIO (The lower the value, the greater the priority) */
#define IRQ_PRIOR_PIO    0

extern uint8_t Verbose;

/**
 * \brief Configure the Pushbuttons.
 *
 * Configure the PIO as inputs and generate corresponding interrupt when
 * pressed or released.
 */
int sd_card_init(sd_card_t *sd_card)
{
	uint8_t slot = 0;  
	
	//hsmci_init();
	sd_mmc_init();	

	sd_card->state = sd_mmc_check(slot);
	int count = 0;
	while (sd_card->state != SD_MMC_OK  && count < 4) {
		sd_card->state = sd_mmc_check(slot);
		delay_ms(1000);
		count++;
	}
	sd_card->slot = slot;
	sd_card->block_size = SD_MMC_BLOCK_SIZE;
	sd_card->first_block = SD_CARD_START_BLOCK;

	if(sd_card->state != SD_MMC_OK) {
		if(Verbose) printf("SD Card Initialization failed, 0x%x\r\n", sd_card->state);
	}
	
	return(sd_card->state);
}

void sd_card_get_info(sd_card_t *sd_card)
{
	if(sd_card->state != SD_MMC_OK) {
		if(Verbose) printf("SD card NOT OK:\r\n");
		return;	
	}
	 uint8_t slot = sd_card->slot;
	sd_card->type = sd_mmc_get_type(slot);
	sd_card->version = sd_mmc_get_version(slot);
	sd_card->capacity = sd_mmc_get_capacity(slot);  // capacity in KBytes
	sd_card->last_block = sd_card->capacity * (1024 / SD_MMC_BLOCK_SIZE);
	
	if(Verbose) {
		printf("SD card information:\r\n");
		switch(sd_card->type)
		{
			case CARD_TYPE_SD:
			printf("- Type: Normal SD card\r\n");
			break;
			case (CARD_TYPE_HC | CARD_TYPE_SD):
			printf("- Type: SD High Capacity card\r\n");
			break;
			default:
			printf("- Type: unknown, %d\r\n", sd_card->type);
		}
		printf("- Total size: %lu KB\r\n", sd_card->capacity);
		printf("- Version: %d\r\n", sd_card->version);
	}
}


int sd_card_write_raw(sd_card_t *sd_card, uint8_t *buffer, uint16_t nblocks, uint32_t addr)
{
	sd_mmc_err_t ret = 0;
	
	if(sd_card->state != SD_MMC_OK) {
		//printf("SD card NOT OK:\r\n");
		return(sd_card->state);
	}

	//sd_card.write_start_addr = sd_card.write_start_addr + nblocks;
	
//	ioport_set_pin_level(SD_CARD_WR_PIN, 1);

	ret = sd_mmc_init_write_blocks(sd_card->slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		if(Verbose) printf("sd_card_write: init_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_write_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		if(Verbose) printf("sd_card_write: start_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_write_blocks(sd_card->slot);
	if ( ret != SD_MMC_OK ) {
		if(Verbose) printf("sd_card_write: wait_end_of_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

//	ioport_set_pin_level(SD_CARD_WR_PIN, 0);
	
	return(SD_MMC_OK);
	
}


int sd_card_read_raw(sd_card_t *sd_card, uint8_t *buffer, uint16_t nblocks, uint32_t addr)
{
	sd_mmc_err_t ret = 0;
	
	if(sd_card->state != SD_MMC_OK) {
		if(Verbose) printf("SD card NOT OK:\r\n");
		return(sd_card->state);
	}

	//sd_card.read_start_addr = sd_card.read_start_addr + nblocks;

//	ioport_set_pin_level(SD_CARD_RD_PIN, 1);
	
	ret = sd_mmc_init_read_blocks(sd_card->slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		if(Verbose) printf("sd_card_read_raw: init_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_read_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		if(Verbose) printf("sd_card_read_raw: start_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_read_blocks(sd_card->slot);
	if ( ret != SD_MMC_OK ) {
		if(Verbose) printf("sd_card_read_raw: wait_end_of_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
//	ioport_set_pin_level(SD_CARD_RD_PIN, 0);

	return(SD_MMC_OK);
	
}


/*

#include <ff.h>
FATFS fat_fs;
FIL file_object;

int sd_card_init_fat(void)
{
	printf("Mount FAT on SD Card.\r\n");
	memset(&fat_fs, 0, sizeof(FATFS));
	FRESULT res = f_mount(LUN_ID_SD_MMC_0_MEM, &fat_fs);
	if (res != FR_OK) {
		printf("f_mount FAILED: res %d\r\n", res);
	}
	return(res);
}

FRESULT sd_card_open_file(char *filename)
{
	FRESULT res; 
	res = f_open(&file_object, (char const *)filename, FA_CREATE_ALWAYS | FA_WRITE);
	if( res != FR_OK ) {
		printf("f_open FAILED: res %d\r\n", res);
	}
	return(res);
}

int sd_card_write_file(uint8_t *buffer, uint16_t nblocks)
{
	UINT nbytes = nblocks * SD_MMC_BLOCK_SIZE;

	// write to file on SD card
	size_t nwrt = 0;
	FRESULT res = f_write(&file_object, buffer, nbytes, &nwrt);
	if( nwrt != nbytes ) {
		printf("Error writing to SD: %d, %d %d\r\n", res, nwrt, nblocks);
		return(0);
	}
	
	return(nwrt);
	
}
*/
