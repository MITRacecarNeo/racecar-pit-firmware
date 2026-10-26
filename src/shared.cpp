#include "shared.h"

Seqlock<DriveCommand> drive_cmd;

Threads::Mutex CmdMutex;
bool dotmatrix_cmd[wire::dot_matrix_bytes] = {false};
uint8_t led_cmd[cfg::RGB_LED_WS2812B::num_leds * 3] = {0};
uint8_t system_state = 0;
volatile uint32_t lastRxTime = 0;

bool piCommandFresh(uint32_t timeout_ms)
{
  const uint32_t last = lastRxTime;
  return last != 0 && (micros() - last) < timeout_ms * 1000UL;
}

Seqlock<RcChannels> rc_channels;

Threads::Mutex PowerMutex;
float power_amps = 0.0f;
float power_volts = 0.0f;

Threads::Mutex ImuMutex;
float imuVals[9] = {0};

Threads::Mutex I2CMutex;

volatile bool imu_ok = false;
volatile bool power_ok = false;

volatile bool manual_override = false;
volatile bool rc_driving = false;
volatile float measured_speed_mps = 0.0f;
