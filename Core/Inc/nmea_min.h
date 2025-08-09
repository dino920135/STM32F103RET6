#ifndef NMEA_MIN_H
#define NMEA_MIN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Parse $G?GGA line. Returns true on success and fills time fields. */
bool nmea_parse_gga_time(const char *line, int *out_h, int *out_m, int *out_s, int *out_us);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_MIN_H */
