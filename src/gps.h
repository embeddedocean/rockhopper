/*
 * gps_com.h
 *
 * Created: 5/26/2017 3:09:22 PM
 *  Author: chris
 */ 


#ifndef GPS_COM_H_
#define GPS_COM_H_

#include "rtc_time.h"

typedef struct {
	double lat;
	double lon;
} gps_t;

//extern int gps_parse_zda(uint8_t *str, int &hour, int *min, int *sec, int *day, int *month, int *year);
extern int gps_parse_zda(uint8_t *str, rtc_time_t *rtc);
extern int gps_parse_gga(uint8_t *str);
extern int gps_gets(char *str, int max_bytes, int timeout);

#endif /* GPS_COM_H_ */