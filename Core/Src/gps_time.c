#include "gps_time.h"

static bool dwt_try_enable(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t a = DWT->CYCCNT;
  for (volatile int i = 0; i < 256; ++i) { __NOP(); }
  uint32_t b = DWT->CYCCNT;
  return (b != a);
}

bool gps_time_init(GpsTime *gt)
{
  if (!gt) return false;
  gt->utc_seconds_of_week = -1;
  gt->pps_cycles = 0;
  gt->cycles_per_second = SystemCoreClock; /* initial guess */
  gt->pps_count = 0;
  gt->dwt_available = dwt_try_enable();
  /* Initialize timestamp correlation variables */
  gt->last_pps_cycles = 0;
  gt->nmea_arrival_cycles = 0;
  gt->pending_utc_seconds = -1;
  gt->nmea_pending = false;
  /* Initialize delay compensation */
  gt->pps_nmea_delay_cycles = 0;
  gt->delay_estimate_count = 0;
  gt->delay_compensation_enabled = false;
  return gt->dwt_available;
}

/* Internal: pending GPS time to latch on next PPS */
static volatile int64_t s_pending_utc = -1;

void gps_time_set_utc_on_next_pps(GpsTime *gt, int64_t gps_seconds_of_week)
{
  (void)gt;
  s_pending_utc = gps_seconds_of_week;
}

/* Set GPS time with timestamp correlation - NMEA always refers to current second */
void gps_time_set_utc_with_correlation(GpsTime *gt, int64_t gps_seconds_of_week)
{
  if (!gt || !gt->dwt_available) return;
  
  uint32_t now = DWT->CYCCNT;
  gt->nmea_arrival_cycles = now;
  
  if (gt->pps_count > 0) {
    /* Update delay estimation if compensation is enabled */
    if (gt->delay_compensation_enabled) {
      uint32_t cycles_since_pps = now - gt->last_pps_cycles;
      
      /* Update running average of PPS-NMEA delay */
      if (gt->delay_estimate_count == 0) {
        gt->pps_nmea_delay_cycles = cycles_since_pps;
      } else {
        /* Gradual adaptation: new_avg = (old_avg * 7 + new_sample) / 8 */
        gt->pps_nmea_delay_cycles = (gt->pps_nmea_delay_cycles * 7 + cycles_since_pps) / 8;
      }
      gt->delay_estimate_count++;
      
      /* Apply time with delay compensation */
      /* If NMEA arrived close to expected delay, apply immediately */
      uint32_t expected_delay = gt->pps_nmea_delay_cycles;
      uint32_t delay_diff = (cycles_since_pps > expected_delay) ? 
                           (cycles_since_pps - expected_delay) : 
                           (expected_delay - cycles_since_pps);
      
      /* If within 10% of expected delay, apply immediately */
      if (delay_diff < (expected_delay / 10)) {
        gt->utc_seconds_of_week = gps_seconds_of_week;
        gt->nmea_pending = false;
        gt->pending_utc_seconds = -1;
      } else {
        /* Delay is unexpected - arm for next PPS for safety */
        gt->pending_utc_seconds = gps_seconds_of_week;
        gt->nmea_pending = true;
      }
    } else {
      /* No compensation - apply immediately to current second */
      gt->utc_seconds_of_week = gps_seconds_of_week;
      gt->nmea_pending = false;
      gt->pending_utc_seconds = -1;
    }
  } else {
    /* No PPS yet - arm for next PPS */
    gt->pending_utc_seconds = gps_seconds_of_week;
    gt->nmea_pending = true;
  }
}

/* Enable/disable gradual delay compensation */
void gps_time_enable_delay_compensation(GpsTime *gt, bool enable)
{
  if (!gt) return;
  gt->delay_compensation_enabled = enable;
  if (!enable) {
    /* Reset delay estimation when disabled */
    gt->pps_nmea_delay_cycles = 0;
    gt->delay_estimate_count = 0;
  }
}

/* Get estimated PPS-NMEA delay in microseconds */
uint32_t gps_time_get_pps_nmea_delay_us(const GpsTime *gt)
{
  if (!gt || !gt->dwt_available || gt->cycles_per_second == 0) return 0;
  return (uint32_t)((uint64_t)gt->pps_nmea_delay_cycles * 1000000ULL / (uint64_t)gt->cycles_per_second);
}

void gps_time_on_pps_isr(GpsTime *gt)
{
  if (!gt) return;
  uint32_t now = DWT->CYCCNT;

  if (gt->pps_count > 0) {
    uint32_t delta = now - gt->pps_cycles;
    gt->cycles_per_second = delta; /* update estimate */
  }
  gt->pps_cycles = now;
  gt->last_pps_cycles = now; /* Store PPS timestamp for correlation */
  gt->pps_count++;

  /* Handle legacy pending UTC (for backward compatibility) */
  if (s_pending_utc >= 0) {
    gt->utc_seconds_of_week = s_pending_utc;
    s_pending_utc = -1;
  }
  /* Handle new correlation-based pending time */
  else if (gt->nmea_pending && gt->pending_utc_seconds >= 0) {
    gt->utc_seconds_of_week = gt->pending_utc_seconds;
    gt->nmea_pending = false;
    gt->pending_utc_seconds = -1;
  }
  /* Normal time advancement */
  else if (gt->utc_seconds_of_week >= 0) {
    gt->utc_seconds_of_week += 1; /* advance one second on each PPS */
    /* Wrap around at end of week (604800 seconds = 7 days) */
    if (gt->utc_seconds_of_week >= 604800) {
      gt->utc_seconds_of_week = 0;
    }
  }
}

bool gps_time_now(const GpsTime *gt, int64_t *out_sec, uint32_t *out_usec)
{
  if (!gt || !gt->dwt_available) {
    if (out_sec) *out_sec = (int64_t)HAL_GetTick() / 1000;
    if (out_usec) *out_usec = (HAL_GetTick() % 1000) * 1000u;
    return false;
  }
  uint32_t now = DWT->CYCCNT;
  uint32_t sub = now - gt->pps_cycles;

  if (gt->utc_seconds_of_week < 0) {
    if (out_sec) *out_sec = (int64_t)HAL_GetTick() / 1000;
    if (out_usec) *out_usec = (HAL_GetTick() % 1000) * 1000u;
    return false;
  }
  /* Convert sub-second cycles to microseconds using cycles_per_second */
  uint32_t usec = (uint32_t)((uint64_t)sub * 1000000ULL / (uint64_t)(gt->cycles_per_second ? gt->cycles_per_second : SystemCoreClock));
  if (usec >= 1000000u) usec = 999999u; /* clamp */
  if (out_sec) *out_sec = gt->utc_seconds_of_week;
  if (out_usec) *out_usec = usec;
  return true;
}

GpsTimeState gps_time_get_state(const GpsTime *gt)
{
  if (!gt) return GPS_TIME_NO_DWT;
  if (!gt->dwt_available) return GPS_TIME_NO_DWT;
  if (gt->pps_count == 0) return GPS_TIME_NO_PPS;
  if (s_pending_utc >= 0) return GPS_TIME_UTC_PENDING;
  if (gt->utc_seconds_of_week < 0) return GPS_TIME_PPS_ONLY;
  return GPS_TIME_UTC_LOCKED;
}

const char* gps_time_state_str(GpsTimeState s)
{
  switch (s) {
    case GPS_TIME_NO_DWT: return "NO_DWT";
    case GPS_TIME_NO_PPS: return "NO_PPS";
    case GPS_TIME_UTC_PENDING: return "UTC_PENDING";
    case GPS_TIME_PPS_ONLY: return "PPS_ONLY";
    case GPS_TIME_UTC_LOCKED: return "UTC_LOCKED";
    default: return "UNKNOWN";
  }
}
