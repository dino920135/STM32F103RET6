#include "nmea_min.h"
#include <ctype.h>
#include <string.h>

static int parse_int(const char *s, int len)
{
  int v = 0;
  for (int i = 0; i < len; ++i) {
    if (!isdigit((unsigned char)s[i])) return -1;
    v = v * 10 + (s[i] - '0');
  }
  return v;
}

/* Extract hhmmss(.sss) from field 1 of GGA */
bool nmea_parse_gga_time(const char *line, int *out_h, int *out_m, int *out_s, int *out_us)
{
  if (!line || strncmp(line+3, "GGA", 3) != 0) return false; /* "$G?GGA" */
  /* Find first comma */
  const char *p = strchr(line, ',');
  if (!p) return false; /* field 1 start */
  const char *f = p + 1;
  /* field format: hhmmss[.sss] */
  if (strlen(f) < 6) return false;
  int hh = parse_int(f+0, 2);
  int mm = parse_int(f+2, 2);
  int ss = parse_int(f+4, 2);
  int us = 0;
  const char *dot = strchr(f, '.');
  if (dot) {
    /* parse milliseconds up to 3 digits, convert to microseconds */
    int ms_len = 0;
    const char *q = dot + 1;
    while (isdigit((unsigned char)q[ms_len]) && ms_len < 3) ms_len++;
    int ms = parse_int(q, ms_len);
    if (ms < 0) ms = 0;
    us = ms * 1000;
  }
  if (hh < 0 || mm < 0 || ss < 0) return false;
  if (out_h) *out_h = hh;
  if (out_m) *out_m = mm;
  if (out_s) *out_s = ss;
  if (out_us) *out_us = us;
  return true;
}

/* Parse RMC message for time and date */
bool nmea_parse_rmc_time_date(const char *line, int *out_h, int *out_m, int *out_s, int *out_us, int *out_dd, int *out_mm, int *out_yy)
{
  if (!line || strncmp(line+3, "RMC", 3) != 0) return false; /* "$G?RMC" */
  
  /* Find field boundaries without modifying input */
  const char *field_starts[12];
  int field_lens[12];
  int field_count = 0;
  const char *p = line;
  
  /* Split by commas */
  while (*p && field_count < 12) {
    field_starts[field_count] = p;
    const char *comma = strchr(p, ',');
    if (!comma) {
      field_lens[field_count] = strlen(p);
      field_count++;
      break;
    }
    field_lens[field_count] = comma - p;
    field_count++;
    p = comma + 1;
  }
  
  if (field_count < 10) return false; /* Need at least 10 fields for RMC */
  
  /* Field 1: Time (hhmmss.sss) */
  if (field_lens[1] < 6) return false;
  const char *time_field = field_starts[1];
  
  int hh = parse_int(time_field+0, 2);
  int mm = parse_int(time_field+2, 2);
  int ss = parse_int(time_field+4, 2);
  int us = 0;
  
  /* Find decimal point in time field */
  for (int i = 6; i < field_lens[1]; i++) {
    if (time_field[i] == '.') {
      int ms_len = 0;
      const char *q = time_field + i + 1;
      while (ms_len < 3 && (i + 1 + ms_len) < field_lens[1] && isdigit((unsigned char)q[ms_len])) {
        ms_len++;
      }
      int ms = parse_int(q, ms_len);
      if (ms < 0) ms = 0;
      us = ms * 1000;
      break;
    }
  }
  
  /* Field 9: Date (ddmmyy) */
  if (field_lens[9] < 6) return false;
  const char *date_field = field_starts[9];
  
  int dd = parse_int(date_field+0, 2);
  int mm_date = parse_int(date_field+2, 2);
  int yy = parse_int(date_field+4, 2);
  
  /* Convert 2-digit year to 4-digit (assume 2000-2099) */
  if (yy >= 0 && yy < 100) yy += 2000;
  
  if (hh < 0 || mm < 0 || ss < 0 || dd < 0 || mm_date < 0 || yy < 0) return false;
  
  if (out_h) *out_h = hh;
  if (out_m) *out_m = mm;
  if (out_s) *out_s = ss;
  if (out_us) *out_us = us;
  if (out_dd) *out_dd = dd;
  if (out_mm) *out_mm = mm_date;
  if (out_yy) *out_yy = yy;
  
  return true;
}

/* Calculate day of week from date using Zeller's congruence */
int calculate_day_of_week(int dd, int mm, int yy)
{
  if (mm < 3) {
    mm += 12;
    yy--;
  }
  
  int century = yy / 100;
  int year_of_century = yy % 100;
  
  /* Zeller's congruence formula */
  int day_of_week = (dd + (13 * (mm + 1)) / 5 + year_of_century + year_of_century / 4 + century / 4 - 2 * century) % 7;
  
  /* Convert to Sunday=1, Monday=2, ..., Saturday=7 */
  if (day_of_week <= 0) day_of_week += 7;
  
  return day_of_week;
}
