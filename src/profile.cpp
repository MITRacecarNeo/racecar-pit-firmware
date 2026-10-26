#include "profile.h"

#ifdef RCN_PROFILE

#include <Arduino.h>
#include <IntervalTimer.h>

#include "prof_stats.h"

namespace prof
{
  namespace
  {
    const char *const period_names[period_count] = {
        "drive_loop", "tx_slot", "rx_loop", "imu_loop", "power_loop", "rc_loop", "dot_loop", "led_loop", "wdt_refresh"};
    const char *const section_names[section_count] = {
        "drive_body", "imu_read", "imu_bus_wait", "power_read", "power_bus_wait",
        "led_show", "tx_send", "rx_frame", "cmd_to_pwm", "irq_late"};
    const char *const counter_names[counter_count] = {
        "tx_frames", "info_chunks", "rx_frames", "rx_crc_fail", "rx_short", "imu_distinct", "power_reads"};

    // Each stat has one writer; the reporter copies and clears it with
    // interrupts masked, a few microseconds per stat.
    PeriodStat periods[period_count];
    uint32_t period_last[period_count];
    SectionStat sections[section_count];
    uint32_t counters[counter_count];

    volatile uint32_t cmd_accepted_at = 0;
    volatile uint32_t cmd_seq = 0;
    uint32_t cmd_seq_used = 0;

    constexpr uint32_t irq_period_us = 100;
    IntervalTimer irq_timer;
    uint32_t irq_last = 0;

    uint32_t cyclesToUs(uint32_t cycles)
    {
      return cycles / (F_CPU_ACTUAL / 1000000UL);
    }

    void onIrqTimer()
    {
      const uint32_t t = ARM_DWT_CYCCNT;
      if (irq_last != 0)
      {
        const uint32_t dt_us = cyclesToUs(t - irq_last);
        sections[irq_late].add(dt_us > irq_period_us ? dt_us - irq_period_us : 0);
      }
      irq_last = t;
    }

    uint32_t window_start_ms = 0;
    uint32_t lines_dropped = 0;

    // Never block loop() on USB: with no reader on the port the buffer fills
    // and a plain println would wait.
    void emit(const char *line, size_t len)
    {
      if (len == 0)
        return;
      if ((size_t)Serial.availableForWrite() < len + 2)
      {
        lines_dropped++;
        return;
      }
      Serial.write(line, len);
      Serial.write("\r\n", 2);
    }
  }

  void begin()
  {
    for (auto &p : periods)
      p.clear();
    for (auto &s : sections)
      s.clear();
    irq_timer.priority(0);
    irq_timer.begin(onIrqTimer, irq_period_us);
    window_start_ms = millis();
  }

  uint32_t now()
  {
    return ARM_DWT_CYCCNT;
  }

  void period(Period id)
  {
    const uint32_t t = ARM_DWT_CYCCNT;
    noInterrupts();
    if (period_last[id] != 0)
      periods[id].add(cyclesToUs(t - period_last[id]));
    period_last[id] = t;
    interrupts();
  }

  void section(Section id, uint32_t start)
  {
    const uint32_t us = cyclesToUs(ARM_DWT_CYCCNT - start);
    noInterrupts();
    sections[id].add(us);
    interrupts();
  }

  void count(Counter id)
  {
    noInterrupts();
    counters[id]++;
    interrupts();
  }

  void commandAccepted()
  {
    cmd_accepted_at = ARM_DWT_CYCCNT;
    cmd_seq = cmd_seq + 1;
  }

  void commandUsed()
  {
    const uint32_t seq = cmd_seq;
    if (seq == cmd_seq_used)
      return;
    cmd_seq_used = seq;
    section(cmd_to_pwm, cmd_accepted_at);
  }

  void report()
  {
    const uint32_t t_ms = millis();
    if (t_ms - window_start_ms < 1000)
      return;
    window_start_ms = t_ms;

    static PeriodStat p;
    char line[96];
    for (uint8_t i = 0; i < period_count; i++)
    {
      noInterrupts();
      p = periods[i];
      periods[i].clear();
      interrupts();
      emit(line, formatPeriod(line, sizeof(line), t_ms, period_names[i], p));
    }
    for (uint8_t i = 0; i < section_count; i++)
    {
      noInterrupts();
      const SectionStat s = sections[i];
      sections[i].clear();
      interrupts();
      emit(line, formatSection(line, sizeof(line), t_ms, section_names[i], s));
    }
    for (uint8_t i = 0; i < counter_count; i++)
    {
      noInterrupts();
      const uint32_t v = counters[i];
      counters[i] = 0;
      interrupts();
      emit(line, formatCounter(line, sizeof(line), t_ms, counter_names[i], v));
    }
    const uint32_t dropped = lines_dropped;
    lines_dropped = 0;
    emit(line, formatCounter(line, sizeof(line), t_ms, "report_lines_dropped", dropped));
  }
}

#endif
