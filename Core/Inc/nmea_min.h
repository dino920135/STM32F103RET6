#ifndef NMEA_MIN_H
#define NMEA_MIN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Parse $G?GGA line. Returns true on success and fills time fields. */
bool nmea_parse_gga_time(const char *line, int *out_h, int *out_m, int *out_s, int *out_us);

/* Parse $G?RMC line. Returns true on success and fills time and date fields. */
bool nmea_parse_rmc_time_date(const char *line, int *out_h, int *out_m, int *out_s, int *out_us, int *out_dd, int *out_mm, int *out_yy);

/* Calculate day of week from date (1=Sunday, 2=Monday, ..., 7=Saturday) */
int calculate_day_of_week(int dd, int mm, int yy);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_MIN_H */
