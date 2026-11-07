#include "speed_store.h"

#include <Arduino.h>
#include <avr/eeprom.h>

#include "config.h"
#include "seqlock.h"
#include "shared.h"
#include "watchdog.h"

namespace
{
  constexpr uint32_t block_addr = 0;
  const SpeedConstants &defaults = cfg::PID_CTRL::speed_defaults;

  SpeedConstants buffers[2] = {cfg::PID_CTRL::speed_defaults, cfg::PID_CTRL::speed_defaults};
  volatile uint8_t active = 0;
  ConstantsSession session;
  Seqlock<ConstantsInfo> info_snapshot;

  void publish()
  {
    const uint8_t next = active ^ 1;
    buffers[next] = session.live;
    active = next; // one byte store; the drive tick sees the old or the new set
    info_snapshot.write(constantsInfo(session, defaults));
  }

  void readBlock(uint8_t *block)
  {
    eeprom_read_block(block, (const void *)block_addr, car_constants::block_bytes);
  }

  void writeLine(const char *s, size_t n)
  {
    // Blocking is fine here: these are answers to a person or a script.
    Serial.write(s, n);
  }
}

void loadSpeedConstants()
{
  uint8_t block[car_constants::block_bytes];
  readBlock(block);
  session = openSession(block, defaults);
  publish();
}

const SpeedConstants &speedConstants()
{
  return buffers[active];
}

ConstantsInfo speedConstantsInfo()
{
  return readShared(info_snapshot);
}

void serviceSpeedConstants()
{
  static char line[160];
  static size_t len = 0;
  static bool overflow = false;

  while (Serial.available() > 0)
  {
    const char c = (char)Serial.read();
    if (c == '\r')
      continue;
    if (c != '\n')
    {
      if (len < sizeof(line) - 1)
        line[len++] = c;
      else
        overflow = true;
      continue;
    }
    line[len] = '\0';
    const bool skip = len == 0;
    const bool too_long = overflow;
    len = 0;
    overflow = false;
    if (skip)
      continue;

    static char out[1024];
    if (too_long)
    {
      static const char msg[] = "err line too long\n";
      writeLine(msg, sizeof(msg) - 1);
      continue;
    }

    const bool stopped = fabsf(measured_speed_mps) < 0.01f && !rc_driving;
    uint32_t ymd = 0;
    const ConstantsAction action = constantsCommand(session, defaults, line, stopped, ymd, out, sizeof(out));
    size_t n = strlen(out);

    if (action == ConstantsAction::save || action == ConstantsAction::erase)
    {
      uint8_t block[car_constants::block_bytes];
      if (action == ConstantsAction::save)
        encodeBlock(session.live, ymd, block);
      else
        memset(block, 0xFF, sizeof(block));
      // Only bytes that differ are written; a full flash sector is erased
      // first, which holds interrupts off for tens of ms.
      wdt::flashWrite(true);
      eeprom_write_block(block, (void *)block_addr, sizeof(block));
      readBlock(block);
      wdt::flashWrite(false);
      n = action == ConstantsAction::save ? finishSave(session, block, out, sizeof(out))
                                          : finishErase(session, block, out, sizeof(out));
      info_snapshot.write(constantsInfo(session, defaults));
    }
    else if (action == ConstantsAction::apply)
      publish();

    writeLine(out, n);
  }
}
