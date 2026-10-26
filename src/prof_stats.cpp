#include "prof_stats.h"

#include <stdio.h>
#include <string.h>

void PeriodStat::clear()
{
  memset(this, 0, sizeof(*this));
}

void PeriodStat::add(uint32_t us)
{
  uint32_t bin = us / prof::bin_us;
  if (bin >= prof::bins)
    bin = prof::bins - 1;
  if (hist[bin] < UINT16_MAX)
    hist[bin]++;
  n++;
  sum_us += us;
  if (us > max_us)
    max_us = us;
}

uint32_t PeriodStat::quantile(float q) const
{
  if (n == 0)
    return 0;
  uint32_t total = 0;
  for (uint16_t i = 0; i < prof::bins; i++)
    total += hist[i];
  // Rank of the sample at quantile q, 1-based, at least 1.
  uint32_t rank = (uint32_t)(q * (float)total + 0.999f);
  if (rank < 1)
    rank = 1;
  uint32_t seen = 0;
  for (uint16_t i = 0; i < prof::bins; i++)
  {
    seen += hist[i];
    if (seen >= rank)
      return (uint32_t)(i + 1) * prof::bin_us;
  }
  return (uint32_t)prof::bins * prof::bin_us;
}

void SectionStat::clear()
{
  n = sum_us = max_us = 0;
}

void SectionStat::add(uint32_t us)
{
  n++;
  sum_us += us;
  if (us > max_us)
    max_us = us;
}

namespace
{
  size_t finish(int written, size_t cap)
  {
    return (written < 0 || (size_t)written >= cap) ? 0 : (size_t)written;
  }
}

size_t formatPeriod(char *buf, size_t cap, uint32_t t_ms, const char *name, const PeriodStat &s)
{
  return finish(snprintf(buf, cap, "P,%lu,period,%s,%lu,%lu,%lu,%lu,%lu", (unsigned long)t_ms, name,
                         (unsigned long)s.n, (unsigned long)s.sum_us, (unsigned long)s.max_us,
                         (unsigned long)s.quantile(0.5f), (unsigned long)s.quantile(0.99f)),
                cap);
}

size_t formatSection(char *buf, size_t cap, uint32_t t_ms, const char *name, const SectionStat &s)
{
  return finish(snprintf(buf, cap, "P,%lu,section,%s,%lu,%lu,%lu", (unsigned long)t_ms, name,
                         (unsigned long)s.n, (unsigned long)s.sum_us, (unsigned long)s.max_us),
                cap);
}

size_t formatCounter(char *buf, size_t cap, uint32_t t_ms, const char *name, uint32_t value)
{
  return finish(snprintf(buf, cap, "P,%lu,counter,%s,%lu", (unsigned long)t_ms, name, (unsigned long)value), cap);
}
