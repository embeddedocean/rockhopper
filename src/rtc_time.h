/*
 * rtc_time.h
 *
 * Created: 5/26/2017 9:39:16 AM
 *  Author: chris
 */ 

#ifndef _RTC_TIME_H_
#define _RTC_TIME_H_

typedef struct {
	uint8_t century;  
	uint8_t year;    // 0 - 99
	uint8_t month;   // 1 - 12
	uint8_t day;     // 1 - 31
	uint8_t hour;    // 0 - 23
	uint8_t minute;  // 0 - 59
	uint8_t second; // 0 - 59
} rtc_time_t;

extern void setup_rtc( rtc_time_t *t );
extern uint32_t rtc_time_to_epoch(rtc_time_t *t);
extern uint32_t time_to_epoch(uint8_t yr, uint8_t mn, uint8_t dy, uint8_t hr, uint8_t mi, uint8_t se);
extern void epoch_to_rtc_time(rtc_time_t *t, uint32_t epoch);


#endif /* DATE_TIME_H_ */