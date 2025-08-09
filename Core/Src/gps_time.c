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
  gt->utc_seconds = -1;
  gt->pps_cycles = 0;
  gt->cycles_per_second = SystemCoreClock; /* initial guess */
  gt->pps_count = 0;
  gt->dwt_available = dwt_try_enable();
  return gt->dwt_available;
}

/* Internal: pending UTC to latch on next PPS */
static volatile int64_t s_pending_utc = -1;

void gps_time_set_utc_on_next_pps(GpsTime *gt, int64_t utc_seconds)
{
  (void)gt;
  s_pending_utc = utc_seconds;
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
  gt->pps_count++;

  if (s_pending_utc >= 0) {
    gt->utc_seconds = s_pending_utc;
    s_pending_utc = -1;
  } else if (gt->utc_seconds >= 0) {
    gt->utc_seconds += 1; /* advance one second on each PPS */
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

  if (gt->utc_seconds < 0) {
    if (out_sec) *out_sec = (int64_t)HAL_GetTick() / 1000;
    if (out_usec) *out_usec = (HAL_GetTick() % 1000) * 1000u;
    return false;
  }
  /* Convert sub-second cycles to microseconds using cycles_per_second */
  uint32_t usec = (uint32_t)((uint64_t)sub * 1000000ULL / (uint64_t)(gt->cycles_per_second ? gt->cycles_per_second : SystemCoreClock));
  if (usec >= 1000000u) usec = 999999u; /* clamp */
  if (out_sec) *out_sec = gt->utc_seconds;
  if (out_usec) *out_usec = usec;
  return true;
}

GpsTimeState gps_time_get_state(const GpsTime *gt)
{
  if (!gt) return GPS_TIME_NO_DWT;
  if (!gt->dwt_available) return GPS_TIME_NO_DWT;
  if (gt->pps_count == 0) return GPS_TIME_NO_PPS;
  if (s_pending_utc >= 0) return GPS_TIME_UTC_PENDING;
  if (gt->utc_seconds < 0) return GPS_TIME_PPS_ONLY;
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
