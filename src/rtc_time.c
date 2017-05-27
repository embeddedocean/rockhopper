/*
 * rtc_time.c
 *
 * Created: 5/26/2017 9:37:31 AM
 *  Author: chris
 */ 
#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "rtc_time.h"

/**
 * Calculate week from year, month, day.
 */
static uint32_t calculate_week(uint32_t ul_year, uint32_t ul_month, uint32_t ul_day)
{
	uint32_t ul_week;

	if (ul_month == 1 || ul_month == 2) {
		ul_month += 12;
		--ul_year;
	}

	ul_week = (ul_day + 2 * ul_month + 3 * (ul_month + 1) / 5 + ul_year +
			ul_year / 4 - ul_year / 100 + ul_year / 400) % 7;

	++ul_week;

	return ul_week;
}

void setup_rtc( rtc_time_t *dt )
{
	uint32_t year = (uint32_t)(dt->century*100 + dt->year);
	uint32_t month = (uint32_t)(dt->month);
	uint32_t day = (uint32_t)(dt->day);
	uint32_t hour = (uint32_t)(dt->hour);
	uint32_t minute = (uint32_t)(dt->minute);
	uint32_t sec = (uint32_t)(dt->second);
	
	uint32_t week = calculate_week(year, month, day);
	//uint32_t usec = 0;
	
	rtc_set_hour_mode(RTC, 0);
	rtc_set_time(RTC, hour, minute, sec);
	rtc_set_date(RTC, year, month, day, week);
	rtc_get_time(RTC, &hour, &minute, &sec);
	rtc_get_date(RTC, &year, &month, &day, &week);
	
	printf("RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, sec); 
}


//---------------------------------------------------------------------------------
// epoch calculator good for years 0-99 (representing 2000-2099)
// Found this on stackexchange

// offset is January 1, 2000 12:00:00 AM
#define RTC_EPOCH_OFFSET 946684800

static uint32_t DAYS[4][12] =
{
	{   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
	{ 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
	{ 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
	{1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
};


uint32_t rtc_time_to_epoch(rtc_time_t *t)
{
	uint32_t sec = (uint32_t)t->second;  // 0-59
	uint32_t minute = (uint32_t)t->minute;  // 0-59
	uint32_t hour   = (uint32_t)t->hour;    // 0-23
	uint32_t day    = (uint32_t)t->day-1;   // 0-30
	uint32_t month  = (uint32_t)t->month-1; // 0-11
	uint32_t year   = (uint32_t)t->year;    // 0-99
	return (((year/4*(365*4+1)+DAYS[year%4][month]+day)*24+hour)*60+minute)*60+sec + RTC_EPOCH_OFFSET;
}

uint32_t time_to_epoch(uint8_t yr, uint8_t mn, uint8_t dy, uint8_t hr, uint8_t mi, uint8_t se)
{
	uint32_t sec = (uint32_t)se;    // 0-59
	uint32_t minute = (uint32_t)mi;    // 0-59
	uint32_t hour   = (uint32_t)hr;    // 0-23
	uint32_t day    = (uint32_t)dy-1;  // 0-30
	uint32_t month  = (uint32_t)mn-1;  // 0-11
	uint32_t year   = (uint32_t)yr;    // 0-99
	return (((year/4*(365*4+1)+DAYS[year%4][month]+day)*24+hour)*60+minute)*60+sec + RTC_EPOCH_OFFSET;
}


void epoch_to_rtc_time(rtc_time_t *t, uint32_t epoch)
{
	epoch = epoch - RTC_EPOCH_OFFSET;
	
	t->second = (uint8_t)(epoch%60); 
	epoch /= 60;
	t->minute = (uint8_t)(epoch%60); 
	epoch /= 60;
	t->hour   = (uint8_t)(epoch%24); 
	epoch /= 24;

	uint32_t years = epoch/(365*4+1)*4;
	epoch %= 365*4+1;

	uint32_t year;
	for (year=3; year>0; year--)
	{
		if (epoch >= DAYS[year][0])
		break;
	}

	uint8_t month;
	for (month=11; month>0; month--)
	{
		if (epoch >= DAYS[year][month])
		break;
	}

	t->century  = 20; // always 20th century
	t->year  = (uint8_t)(years + year);
	t->month = (uint8_t)(month + 1);
	t->day   = (uint8_t)(epoch-DAYS[year][month] + 1);
}

