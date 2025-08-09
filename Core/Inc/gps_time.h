#ifndef GPS_TIME_H
#define GPS_TIME_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "core_cm3.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PPS-only time disciplining. NMEA integration will be added later. */

typedef struct {
  /* UTC seconds aligned to last PPS. -1 means unknown/not locked yet */
  int64_t utc_seconds;
  /* DWT cycle count at last PPS edge */
  uint32_t pps_cycles;
  /* Estimated cycles per second between PPS pulses */
  uint32_t cycles_per_second;
  /* Number of PPS pulses observed */
  uint32_t pps_count;
  /* DWT availability */
  bool dwt_available;
} GpsTime;

/* Initialize DWT and state. Returns true if DWT is usable. */
bool gps_time_init(GpsTime *gt);

/* Call from PPS ISR (EXTI callback or TIM input capture ISR). */
void gps_time_on_pps_isr(GpsTime *gt);

/* Set the absolute UTC seconds to be applied at the NEXT PPS edge. */
void gps_time_set_utc_on_next_pps(GpsTime *gt, int64_t utc_seconds);

/* Query current time: returns seconds and microseconds since epoch (if locked).
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
