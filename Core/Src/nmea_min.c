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
