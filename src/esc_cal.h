#pragma once
#include <stddef.h>
#include <stdint.h>

// ESC throttle calibration steps for the calibration build (RCN_ESC_CAL).
// Arduino-free, unit tested under tests/.
//
// The steps follow the ESC's own sequence: neutral, full forward, full
// reverse, each held while the operator presses SET. Commands arrive as single
// characters over USB serial from scripts/esc_cal.py:
//   'n' neutral, 'f' full forward, 'r' full reverse, '?' status only.
// Full forward is accepted only from neutral and full reverse only from full
// forward, so a stray character cannot jump to the far end. Any character
// counts as the link alive; a step away from neutral returns to neutral when
// the link has been quiet for link_timeout_ms.

enum class EscCalStep : uint8_t
{
  Neutral = 0,
  FullForward = 1,
  FullReverse = 2,
  Done = 3, // neutral again after full reverse
};

struct EscCal
{
  EscCalStep step = EscCalStep::Neutral;
  uint32_t last_rx_ms = 0;
  bool aborted = false; // the last step away from neutral timed out
};

// Applies one command character at now_ms; false for an unknown character or
// an out-of-order step (the step is unchanged).
bool escCalApply(EscCal &cal, char c, uint32_t now_ms);

// Returns to neutral and sets aborted when a step away from neutral has had
// no character for timeout_ms.
void escCalTick(EscCal &cal, uint32_t now_ms, uint32_t timeout_ms);

uint16_t escCalPwm(const EscCal &cal, uint16_t pwm_min, uint16_t pwm_neutral, uint16_t pwm_max);

const char *escCalStepName(EscCalStep step);

// Status reply, "esc_cal step=<name> pwm=<us> aborted=<0|1>\n"; returns the
// length written, or 0 if it does not fit.
size_t escCalStatus(const EscCal &cal, uint16_t pwm, char *buf, size_t cap);

// LEDs lit from each end of the strip for a held PWM: 1 at pwm_min up to
// `half` at pwm_max, so full reverse still shows.
int escCalLedHalf(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max, int half);
