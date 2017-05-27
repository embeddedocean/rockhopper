/*
 * gps.c
 *
 * Created: 5/24/2017 2:24:15 PM
 *  Author: chris
 */ 
/*
$GPGGA,210204.000,4737.7930,N,12218.6641,W,2,09,1.1,141.0,M,-18.4,M,,0000*62
$GPGLL,4737.7930,N,12218.6641,W,210204.000,A,D*4C
$GPGSA,A,3,15,28,17,24,06,13,19,30,12,,,,1.8,1.1,1.5*3D
$GPGSV,3,1,12,17,76,079,40,19,72,194,42,28,40,081,38,24,35,306,43*7C
$GPGSV,3,2,12,46,33,147,33,15,25,260,35,13,23,219,36,06,20,158,29*75
$GPGSV,3,3,12,01,20,043,25,30,13,143,32,12,13,277,29,11,05,038,27*7C
$GPRMC,210204.000,A,4737.7930,N,12218.6641,W,000.0,052.9,260517,,,D*72
$GPVTG,052.9,T,,M,000.0,N,000.0,K,D*06

$PSTI,00,1,514,-0.9,30,0*26
*/

#include "asf.h"
#include "board.h"
#include <string.h>
#include "gps.h"

/*
  ZDA - Data and Time

  $GPZDA,hhmmss.ss,dd,mm,yyyy,xx,yy*CC
  $GPZDA,201530.00,04,07,2002,00,00*60

  where:
    hhmmss    HrMinSec(UTC)
    dd,mm,yyy Day,Month,Year
    xx        local zone hours -13..13
    yy        local zone minutes 0..59
    *CC       checksum
*/
//int gps_parse_zda(uint8_t *str, int &hour, int *min, int *sec, int *day, int *month, int *year)
int gps_parse_zda(uint8_t *str, rtc_time_t *rtc)
{
	int n = 0;
	float t, s;
	int h, m, d, mn, y;
	if(strncmp((char *)str, "$GPZDA", 6) == 0) {
		n = sscanf((char *)str, "$GPZDA,%f,%d,%d,%d", &t, &d, &mn, &y);
		h = (int)(t * 0.0001);
		t -= (float)h * 10000.0;
		m = (int)(t * 0.01);
		s = t - (float)m * 100.0;
		rtc->century = 20;
		rtc->year = (uint8_t)(y - 2000);
		rtc->month = (uint8_t)mn;
		rtc->day = (uint8_t)d;
		rtc->hour = (uint8_t)h;
		rtc->minute = (uint8_t)m;
		rtc->second = (uint8_t)s;
		//fprintf(stdout, "GPZDA %d/%d/%d %d:%d:%d\r\n", d, mn, y, h, m, (int)s);
	}
	return(n);
}

/*
 $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

 Where:
 GGA          Global Positioning System Fix Data
 123519       Fix taken at 12:35:19 UTC
 4807.038,N   Latitude 48 deg 07.038' N
 01131.000,E  Longitude 11 deg 31.000' E
 1            Fix quality: 0 = invalid
 1 = GPS fix (SPS)
 2 = DGPS fix
 3 = PPS fix
 4 = Real Time Kinematic
 5 = Float RTK
 6 = estimated (dead reckoning) (2.3 feature)
 7 = Manual input mode
 8 = Simulation mode
 08           Number of satellites being tracked
 0.9          Horizontal dilution of position
 545.4,M      Altitude, Meters, above mean sea level
 46.9,M       Height of geoid (mean sea level) above WGS84
 ellipsoid
 (empty field) time in seconds since last DGPS update
 (empty field) DGPS station ID number
 *47          the checksum data, always begins with *
*/
int gps_parse_gga(uint8_t *str)
{
	int n = 0;
	float t, s;
	float lat, lon;
	int h, m;
	char ns, ew;
	if(strncmp((char *)str, "$GPGGA", 6) == 0) {
		n = sscanf(str, "$GPGGA,%f,%f,%c,%f,%c", &t, &lat, &ns, &lon, &ew);
		h = (int)(t * 0.0001);
		t -= (float)h * 10000.0;
		m = (int)(t * 0.01);
		s = t - (float)m * 100.0;
		float degs = (float)((int)(lat * 0.01));
		float mins = lat - degs*100.0;
		lat = degs + mins/60.0;
		degs = (float)((int)(lon * 0.01));
		mins = lon - degs*100.0;
		lon = degs + mins/60.0;
						
		//fprintf(stdout, "GPGGA %d:%d:%d, %f %c, %f %c\r\n", h, m, s, lat, ns, lon, ew);
	}
	return(n);
}


/*
int gps_gets(char *str, int max_bytes, int timeout)
{
	uint8_t c;
	int n = 0;
	int go = 1;
	int count = 0;
	int timeout_ms = timeout*1000;
	while( go ) {
		//fprintf(stdout, "%x\r\n", uart_get_status(GPS_UART));
		if(uart_is_rx_ready(GPS_UART)) {
			uart_read(GPS_UART, &c);
			//fprintf(stdout, "%c", c);
			if(c == '*') { // newline
				str[n] = 0;
				go = 0;
			} else {
				str[n] = (char)c;
			}
			//if(c == '$') { // new message
			//	n = 0;
			//}
			n++;
			if(n == max_bytes) go = 0;
		} else {
			delay_ms(1);
			count++;
		}
		if(count >= timeout_ms) go = 0;
	}	
	return(n);
}
*/
