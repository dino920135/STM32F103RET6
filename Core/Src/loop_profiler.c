#include "loop_profiler.h"

static bool dwt_try_enable(void)
{
  /* Enable DWT/ITM block */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Reset counter */
  DWT->CYCCNT = 0;
  /* Enable cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* Simple sanity check: counter should increment */
  uint32_t a = DWT->CYCCNT;
  for (volatile int i = 0; i < 1024; ++i) { __NOP(); }
  uint32_t b = DWT->CYCCNT;
  return (b != a);
}

bool loop_profiler_init(LoopProfiler *profiler, uint32_t target_period_us)
{
  if (profiler == NULL) return false;
  profiler->target_period_us = target_period_us;
  profiler->min_val = 0xFFFFFFFFu;
  profiler->max_val = 0u;
  profiler->sum_val = 0u;
  profiler->sample_count = 0u;
  profiler->last_val = 0u;
  profiler->start_cycles = 0u;
  profiler->start_ms = 0u;

  profiler->dwt_available = dwt_try_enable();
  return profiler->dwt_available;
}

void loop_profiler_end(LoopProfiler *profiler)
{
  uint32_t val;
  if (profiler->dwt_available) {
    uint32_t end_cycles = DWT->CYCCNT;
    val = end_cycles - profiler->start_cycles; /* cycles */
  } else {
    uint32_t end_ms = HAL_GetTick();
    uint32_t dt_ms = end_ms - profiler->start_ms;
    val = dt_ms * 1000u; /* convert ms to us for consistency */
  }
  profiler->last_val = val;
  if (val < profiler->min_val) profiler->min_val = val;
  if (val > profiler->max_val) profiler->max_val = val;
  profiler->sum_val += val;
  profiler->sample_count += 1u;
}

uint32_t loop_profiler_get_stats_us(const LoopProfiler *profiler,
                                    uint32_t *out_min_us,
                                    uint32_t *out_avg_us,
                                    uint32_t *out_max_us,
                                    uint32_t *out_last_us,
                                    uint32_t *out_slack_us,
                                    float *out_load_percent)
{
  uint32_t min_us, avg_us, max_us, last_us;

  if (profiler->sample_count == 0u) {
    min_us = avg_us = max_us = last_us = 0u;
  } else {
    if (profiler->dwt_available) {
      min_us  = loop_profiler_cycles_to_us(profiler->min_val);
      avg_us  = loop_profiler_cycles_to_us((uint32_t)(profiler->sum_val / profiler->sample_count));
      max_us  = loop_profiler_cycles_to_us(profiler->max_val);
      last_us = loop_profiler_cycles_to_us(profiler->last_val);
    } else {
      /* Values already stored in microseconds */
      min_us  = profiler->min_val;
      avg_us  = (uint32_t)(profiler->sum_val / profiler->sample_count);
      max_us  = profiler->max_val;
      last_us = profiler->last_val;
    }
  }

  uint32_t slack_us = 0u;
  float load = 0.0f;
  if (profiler->target_period_us != 0u) {
    slack_us = (max_us < profiler->target_period_us) ? (profiler->target_period_us - max_us) : 0u;
    load = (float)max_us * 100.0f / (float)profiler->target_period_us;
  }

  if (out_min_us)       *out_min_us = min_us;
  if (out_avg_us)       *out_avg_us = avg_us;
  if (out_max_us)       *out_max_us = max_us;
  if (out_last_us)      *out_last_us = last_us;
  if (out_slack_us)     *out_slack_us = slack_us;
  if (out_load_percent) *out_load_percent = load;

  return profiler->sample_count;
}

void loop_profiler_reset(LoopProfiler *profiler)
{
  profiler->min_val = 0xFFFFFFFFu;
  profiler->max_val = 0u;
  profiler->sum_val = 0u;
  profiler->sample_count = 0u;
  profiler->last_val = 0u;
}
