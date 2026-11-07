#include "link.h"

#include <Arduino.h>
#include <IntervalTimer.h>
#include <TeensyThreads.h>

#include "config.h"
#include "command_framer.h"
#include "crc16.h"
#include "info.h"
#include "info_buffers.h"
#include "packets.h"
#include "profile.h"
#include "watchdog.h"
#include "shared.h"
#include "speed_store.h"

namespace
{
  IntervalTimer telemetry_timer;
  TXPacket pkt = {};

  // Last values that read cleanly, used when a write is in progress.
  RcChannels rc_last = {};
  ImuSample imu_last = {};
  PowerSample power_last = {0.0f, 0.0f};

  InfoBuffers<info::payload_max_bytes> info_buffers;
  const InfoBuffers<info::payload_max_bytes>::Payload *info_sending = nullptr;
  uint16_t info_next = 0;
  uint8_t info_packet[info_packet_max_bytes];

  void fillTelemetry()
  {
    rc_channels.tryRead(rc_last);
    imu_sample.tryRead(imu_last);
    power_sample.tryRead(power_last);
    memcpy(pkt.RxReceiver, rc_last.us, sizeof(pkt.RxReceiver));
    memcpy(pkt.IMU, imu_last.v, sizeof(pkt.IMU));
    pkt.VoltAndCurr[0] = (uint16_t)(power_last.volts * 1000.0f);
    // Amps packed as mA so sub-amp resolution survives the uint16.
    pkt.VoltAndCurr[1] = (uint16_t)(power_last.amps < 0.0f ? 0.0f : power_last.amps * 1000.0f);
    pkt.Encoder = measured_speed_mps;
    const StallWord stall = unpackStallWord(stall_word);
    pkt.EKF[0] = (float)(uint8_t)stall.state;
    pkt.EKF[1] = stall.seconds_left;
    pkt.EKF[2] = (float)stall.trips;
    pkt.Timestamp = (int32_t)micros();
    // Checksum the exact bytes going on the wire, magic included.
    pkt.Checksum = crc16_ccitt((const uint8_t *)&pkt, sizeof(TXPacket) - 2);
  }

  // system_state is one byte, so it reads whole without CmdMutex.
  bool infoRequested()
  {
    return (system_state & cfg::SYS_STATE::info_request) && piCommandFresh(cfg::DISP::cmd_timeout_ms);
  }

  // One info chunk per telemetry frame while the Pi requests it. Each cycle
  // sends one payload start to end.
  void sendInfoChunk()
  {
    if (!infoRequested())
    {
      info_next = 0;
      return;
    }
    if (info_next == 0)
      info_sending = info_buffers.take();
    if (info_sending == nullptr)
      return;
    const size_t n = buildInfoPacket(info_sending->data, info_sending->len, info_sending->revision,
                                     info_next, info_packet, sizeof(info_packet));
    if (n > 0)
    {
      Serial1.write(info_packet, n);
      PROF_COUNT(info_chunks);
    }
    info_next = (uint16_t)((info_next + 1) % infoChunkCount(info_sending->len));
  }

  void telemetryTick()
  {
#ifdef RCN_WDT_TEST
    if (wdt::test_hang == 't')
      return;
#endif
    wdt::beat(wdt::telemetry);
    PROF_PERIOD(tx_slot);
    PROF_START(t_tx);
    fillTelemetry();
    Serial1.write((const uint8_t *)&pkt, sizeof(pkt));
    PROF_SECTION(tx_send, t_tx);
    PROF_COUNT(tx_frames);
    sendInfoChunk();
  }
}

void startTelemetry()
{
  initTxHeader(pkt.header);
  telemetry_timer.priority(cfg::UART::tx_irq_priority);
  telemetry_timer.begin(telemetryTick, 1000000UL / cfg::UART::tx_freq);
}

void InfoBuilder()
{
  size_t last_len = 0;
  uint16_t last_crc = 0;
  uint8_t revision = 0;
  bool built = false;
  uint32_t last_build_ms = 0;

  while (true)
  {
    wdt::beat(wdt::info_builder);
    WDT_TEST_HANG('b');
    // Rebuilt while the Pi requests it; the revision changes only with the
    // content.
    if (infoRequested() && (!built || millis() - last_build_ms >= cfg::UART::info_rebuild_ms))
    {
      auto *p = info_buffers.beginBuild();
      if (p != nullptr)
      {
        const InfoStatus status = {imu_ok, power_ok, wdt::lastResetCause(), wdt::resetFlags(), wdt::armed(),
                                   imu_ok ? imu_regs : Lsm9ds1Registers{}, readShared(dip_summary),
                                   speedConstantsInfo()};
        p->len = writeFirmwareInfo(status, p->data, sizeof(p->data));
        const uint16_t crc = crc16_ccitt((const uint8_t *)p->data, p->len);
        if (built && (p->len != last_len || crc != last_crc))
          revision++;
        p->revision = revision;
        last_len = p->len;
        last_crc = crc;
        built = true;
        last_build_ms = millis();
        info_buffers.publish();
      }
    }
    threads.delay(cfg::TEENSY_THREADS::info_tick_ms);
  }
}

namespace
{
  void applyCommand(const RxPacket &rx)
  {
    drive_cmd.write({rx.ServoAng, rx.MotorSpeed});
    CmdMutex.lock();
    memcpy(dotmatrix_cmd, rx.DotMatrix, sizeof(dotmatrix_cmd));
    memcpy(led_cmd, rx.LED, sizeof(led_cmd));
    system_state = rx.SystemState;
    CmdMutex.unlock();
  }
}

void UARTRx()
{
  static CommandFramer framer;
  uint32_t frame_start_ms = 0;

  while (true)
  {
    PROF_PERIOD(rx_loop);
    wdt::beat(wdt::command_rx);
    WDT_TEST_HANG('r');
    // Take only what has arrived; a frame spans several turns of this thread.
    while (Serial1.available() > 0)
    {
      const bool was_in_frame = framer.inFrame();
      const bool complete = framer.push((uint8_t)Serial1.read());
      if (!was_in_frame && framer.inFrame())
        frame_start_ms = millis();
      if (!complete)
        continue;

      PROF_START(t_rx);
      if (cfg::UART::verify_rx_crc && !framer.crcOk())
      {
        PROF_COUNT(rx_crc_fail);
        continue;
      }
      applyCommand(framer.frame());
      lastRxTime = micros();
      PROF_SECTION(rx_frame, t_rx);
      PROF_COUNT(rx_frames);
      PROF_CMD_ACCEPTED();
    }

    if (framer.inFrame() && millis() - frame_start_ms > cfg::UART::rx_read_timeout_ms)
    {
      framer.reset();
      PROF_COUNT(rx_short);
    }
    lastRxTime = expireRxTime(lastRxTime, micros(), cfg::DISP::rx_expire_us);
    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}
