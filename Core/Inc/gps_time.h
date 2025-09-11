#ifndef GPS_TIME_H
#define GPS_TIME_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "core_cm3.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PPS-only time disciplining with GPS time (GPST = UTC + 18s). */

typedef struct {
  /* GPS seconds of week aligned to last PPS. -1 means unknown/not locked yet */
  int64_t utc_seconds_of_week;
  /* DWT cycle count at last PPS edge */
  uint32_t pps_cycles;
  /* Estimated cycles per second between PPS pulses */
  uint32_t cycles_per_second;
  /* Number of PPS pulses observed */
  uint32_t pps_count;
  /* DWT availability */
  bool dwt_available;
  /* Timestamp correlation for NMEA timing */
  uint32_t last_pps_cycles;        /* DWT cycles at most recent PPS */
  uint32_t nmea_arrival_cycles;    /* DWT cycles when NMEA arrived */
  int64_t pending_utc_seconds;     /* Pending UTC time from NMEA */
  bool nmea_pending;               /* True if NMEA time is pending correlation */
  /* Gradual compensation for PPS-NMEA timing difference */
  uint32_t pps_nmea_delay_cycles;  /* Estimated delay between PPS and NMEA */
  uint32_t delay_estimate_count;   /* Number of samples for delay estimation */
  bool delay_compensation_enabled; /* Enable gradual compensation */
} GpsTime;

/* Initialize DWT and state. Returns true if DWT is usable. */
bool gps_time_init(GpsTime *gt);

/* Call from PPS ISR (EXTI callback or TIM input capture ISR). */
void gps_time_on_pps_isr(GpsTime *gt);

/* Set the absolute GPS seconds of week to be applied at the NEXT PPS edge. */
void gps_time_set_utc_on_next_pps(GpsTime *gt, int64_t gps_seconds_of_week);

/* Set GPS time with timestamp correlation - determines immediate vs next PPS application */
void gps_time_set_utc_with_correlation(GpsTime *gt, int64_t gps_seconds_of_week);

/* Enable/disable gradual delay compensation */
void gps_time_enable_delay_compensation(GpsTime *gt, bool enable);

/* Get estimated PPS-NMEA delay in microseconds */
uint32_t gps_time_get_pps_nmea_delay_us(const GpsTime *gt);

/* Query current time: returns GPS seconds of week and microseconds (if locked).
   If not locked, returns false and outputs coarse time based on internal tick. */
bool gps_time_now(const GpsTime *gt, int64_t *out_sec, uint32_t *out_usec);

/* State of time synchronization */
typedef enum {
  GPS_TIME_NO_DWT = 0,     /* DWT not available; coarse time only */
  GPS_TIME_NO_PPS,         /* No PPS seen yet */
  GPS_TIME_UTC_PENDING,    /* PPS seen; UTC armed for next PPS */
  GPS_TIME_PPS_ONLY,       /* PPS running; UTC not yet set */
  GPS_TIME_UTC_LOCKED      /* PPS running; UTC aligned */
} GpsTimeState;

GpsTimeState gps_time_get_state(const GpsTime *gt);
const char* gps_time_state_str(GpsTimeState s);

#ifdef __cplusplus
}
#endif

#endif /* GPS_TIME_H */
