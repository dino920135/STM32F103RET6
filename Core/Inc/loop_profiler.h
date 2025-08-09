#ifndef LOOP_PROFILER_H
#define LOOP_PROFILER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "core_cm3.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
  LoopProfiler: measures per-iteration execution time using the DWT cycle counter.
  - Uses DWT->CYCCNT when available (microsecond-level precision)
  - Falls back to HAL_GetTick() (millisecond resolution) if DWT not available
*/

typedef struct {
  uint32_t target_period_us;     /* e.g., 10000 for 100 Hz */

  /* Stats accumulated in cycles when dwt_available==true, otherwise in microseconds */
  uint32_t min_val;
  uint32_t max_val;
  uint64_t sum_val;
  uint32_t sample_count;

  /* Last measurement (same unit as above) */
  uint32_t last_val;

  /* Timing points */
  uint32_t start_cycles;
  uint32_t start_ms;

  bool dwt_available;
} LoopProfiler;

/* Initialize the profiler; returns true if DWT is available */
bool loop_profiler_init(LoopProfiler *profiler, uint32_t target_period_us);

/* Mark the start of the measured section */
static inline void loop_profiler_begin(LoopProfiler *profiler)
{
  if (profiler->dwt_available) {
    profiler->start_cycles = DWT->CYCCNT;
  } else {
    profiler->start_ms = HAL_GetTick();
  }
}

/* Mark the end of the measured section and update stats */
void loop_profiler_end(LoopProfiler *profiler);

/* Convert a cycle count to microseconds */
static inline uint32_t loop_profiler_cycles_to_us(uint32_t cycles)
{
  return (uint32_t)((uint64_t)cycles * 1000000ULL / SystemCoreClock);
}

/* Get stats (all in microseconds). Returns sample_count */
uint32_t loop_profiler_get_stats_us(const LoopProfiler *profiler,
                                    uint32_t *out_min_us,
                                    uint32_t *out_avg_us,
                                    uint32_t *out_max_us,
                                    uint32_t *out_last_us,
                                    uint32_t *out_slack_us,
                                    float *out_load_percent);

/* Reset accumulated stats (keeps configuration) */
void loop_profiler_reset(LoopProfiler *profiler);

#ifdef __cplusplus
}
#endif

#endif /* LOOP_PROFILER_H */
