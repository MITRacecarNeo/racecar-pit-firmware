#include "link.h"

#include <Arduino.h>
#include <TeensyThreads.h>

#include "config.h"
#include "crc16.h"
#include "info.h"
#include "packets.h"
#include "profile.h"
#include "shared.h"

namespace
{
  void fillTelemetry(TXPacket &pkt)
  {
    RxMutex.lock();
    memcpy(pkt.RxReceiver, channelVals, sizeof(pkt.RxReceiver));
    RxMutex.unlock();

    ImuMutex.lock();
    memcpy(pkt.IMU, imuVals, sizeof(pkt.IMU));
    ImuMutex.unlock();

    PowerMutex.lock();
    const float amps = power_amps;
    const float volts = power_volts;
    PowerMutex.unlock();

    pkt.VoltAndCurr[0] = (uint16_t)(volts * 1000.0f);
    // Amps packed as mA so sub-amp resolution survives the uint16.
    pkt.VoltAndCurr[1] = (uint16_t)(amps < 0.0f ? 0.0f : amps * 1000.0f);
    pkt.Encoder = measured_speed_mps;
    pkt.Timestamp = (int32_t)micros();
    // Checksum the exact bytes going on the wire, magic included.
    pkt.Checksum = crc16_ccitt((const uint8_t *)&pkt, sizeof(TXPacket) - 2);
  }

  bool infoRequested()
  {
    CmdMutex.lock();
    const uint8_t state = system_state;
    CmdMutex.unlock();
    return (state & cfg::SYS_STATE::info_request) && piCommandFresh(cfg::DISP::cmd_timeout_ms);
  }

  // Sends the info payload one chunk per call, cycling while the Pi keeps
  // the request bit set. The payload is rebuilt at the start of each cycle;
  // the revision changes only when its content does.
  class InfoSender
  {
  public:
    void step()
    {
      if (next_ == 0)
        rebuild();
      const size_t n = buildInfoPacket(payload_, len_, revision_, next_, packet_, sizeof(packet_));
      if (n > 0)
      {
        Serial1.write(packet_, n);
        PROF_COUNT(info_chunks);
      }
      next_ = (uint16_t)((next_ + 1) % infoChunkCount(len_));
    }

    void reset() { next_ = 0; }

  private:
    void rebuild()
    {
      const InfoStatus status = {imu_ok, power_ok};
      const size_t n = writeFirmwareInfo(status, payload_, sizeof(payload_));
      const uint16_t crc = crc16_ccitt((const uint8_t *)payload_, n);
      if (built_ && (n != len_ || crc != crc_))
        revision_++;
      len_ = n;
      crc_ = crc;
      built_ = true;
    }

    char payload_[info::payload_max_bytes];
    uint8_t packet_[info_packet_max_bytes];
    size_t len_ = 0;
    uint16_t crc_ = 0;
    uint8_t revision_ = 0;
    uint16_t next_ = 0;
    bool built_ = false;
  };

  InfoSender info_sender;
}

void UARTTx()
{
  static TXPacket pkt = {};
  initTxHeader(pkt.header);

  const uint32_t interval = 1000000UL / cfg::UART::tx_freq;
  uint32_t next = micros();

  while (true)
  {
    const uint32_t now = micros();
    if ((int32_t)(now - next) >= 0)
    {
      PROF_PERIOD(tx_slot);
      PROF_START(t_tx);
      fillTelemetry(pkt);
      Serial1.write((const uint8_t *)&pkt, sizeof(pkt));
      PROF_SECTION(tx_send, t_tx);
      PROF_COUNT(tx_frames);

      if (infoRequested())
        info_sender.step();
      else
        info_sender.reset();

      // Hold the average rate; after a stall longer than one slot, restart
      // from now rather than sending a burst.
      next += interval;
      if ((int32_t)(now - next) >= 0)
        next = now + interval;
    }
    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}

namespace
{
  void applyCommand(const RxPacket &rx)
  {
    CmdMutex.lock();
    cmd_setpoint[0] = rx.ServoAng;
    cmd_setpoint[1] = rx.MotorSpeed;
    memcpy(dotmatrix_cmd, rx.DotMatrix, sizeof(dotmatrix_cmd));
    memcpy(led_cmd, rx.LED, sizeof(led_cmd));
    system_state = rx.SystemState;
    CmdMutex.unlock();
  }
}

void UARTRx()
{
  static RxPacket rx;
  uint8_t window[4] = {0, 0, 0, 0};

  // magic is little-endian on the Teensy, so its bytes match the wire.
  const uint32_t magic = wire::command_magic;
  const uint16_t magic_crc = crc16_ccitt((const uint8_t *)&magic, sizeof(magic));

  while (true)
  {
    PROF_PERIOD(rx_loop);
    while (Serial1.available())
    {
      // Slide a 4-byte window over the stream until the magic word lands.
      memmove(window, window + 1, sizeof(window) - 1);
      window[3] = Serial1.read();

      uint32_t seen;
      memcpy(&seen, window, sizeof(seen));
      if (seen != magic)
        continue;

      PROF_START(t_rx);
      if (Serial1.readBytes((char *)&rx, sizeof(rx)) != sizeof(rx))
      {
        PROF_COUNT(rx_short);
        continue;
      }

      const uint16_t crc = crc16_ccitt((const uint8_t *)&rx, sizeof(RxPacket) - 2, magic_crc);
      if (cfg::UART::verify_rx_crc && crc != rx.Checksum)
      {
        PROF_COUNT(rx_crc_fail);
        continue;
      }

      applyCommand(rx);
      lastRxTime = micros();
      PROF_SECTION(rx_frame, t_rx);
      PROF_COUNT(rx_frames);
      PROF_CMD_ACCEPTED();
    }
    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}
