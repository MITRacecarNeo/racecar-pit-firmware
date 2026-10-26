#include "sensors.h"

#include <Arduino.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <INA226.h>
#include <PPMReader.h>
#include <TeensyThreads.h>
#include <Wire.h>

#include "config.h"
#include "profile.h"
#include "watchdog.h"
#include "shared.h"

namespace
{
  // Pause between begin() attempts on a missing sensor, with the bus released
  // so the other sensor keeps running; well inside the watchdog timeout.
  constexpr uint32_t sensor_retry_ms = 20;

  using Lsm = Adafruit_LSM9DS1;

  // One register over Wire; the caller holds I2CMutex.
  uint8_t readRegister(uint8_t address, uint8_t reg)
  {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
      return 0;
    if (Wire.requestFrom(address, (uint8_t)1) != 1)
      return 0;
    return (uint8_t)Wire.read();
  }

  constexpr Lsm::lsm9ds1AccelRange_t accelRange(uint8_t g)
  {
    return g == 2 ? Lsm::LSM9DS1_ACCELRANGE_2G
           : g == 4 ? Lsm::LSM9DS1_ACCELRANGE_4G
           : g == 16 ? Lsm::LSM9DS1_ACCELRANGE_16G
                     : Lsm::LSM9DS1_ACCELRANGE_8G;
  }

  constexpr Lsm::lsm9ds1AccelDataRate_t accelRate(uint16_t hz)
  {
    return hz == 10 ? Lsm::LSM9DS1_ACCELDATARATE_10HZ
           : hz == 50 ? Lsm::LSM9DS1_ACCELDATARATE_50HZ
           : hz == 238 ? Lsm::LSM9DS1_ACCELDATARATE_238HZ
           : hz == 476 ? Lsm::LSM9DS1_ACCELDATARATE_476HZ
           : hz == 952 ? Lsm::LSM9DS1_ACCELDATARATE_952HZ
                       : Lsm::LSM9DS1_ACCELDATARATE_119HZ;
  }

  constexpr Lsm::lsm9ds1MagGain_t magGain(uint8_t gauss)
  {
    return gauss == 4 ? Lsm::LSM9DS1_MAGGAIN_4GAUSS
           : gauss == 12 ? Lsm::LSM9DS1_MAGGAIN_12GAUSS
           : gauss == 16 ? Lsm::LSM9DS1_MAGGAIN_16GAUSS
                         : Lsm::LSM9DS1_MAGGAIN_8GAUSS;
  }

  constexpr Lsm::lsm9ds1GyroScale_t gyroScale(uint16_t dps)
  {
    return dps == 245 ? Lsm::LSM9DS1_GYROSCALE_245DPS
           : dps == 2000 ? Lsm::LSM9DS1_GYROSCALE_2000DPS
                         : Lsm::LSM9DS1_GYROSCALE_500DPS;
  }
}

void ReadCurrAndVolt()
{
  static INA226 ina(cfg::INA226_SENSOR::i2c_addy);

  while (true)
  {
    I2CMutex.lock();
    const bool found = ina.begin();
    if (found)
    {
      ina.setAverage(cfg::INA226_SENSOR::avg_set);
      ina.setMaxCurrentShunt(cfg::INA226_SENSOR::max_curr_expected, cfg::INA226_SENSOR::shunt_resistance);
    }
    I2CMutex.unlock();
    if (found)
      break;
    wdt::beat(wdt::power);
    threads.delay(sensor_retry_ms);
  }
  power_ok = true;

  while (true)
  {
    wdt::beat(wdt::power);
    WDT_TEST_HANG('p');
    PROF_PERIOD(power_loop);
    PROF_START(t_wait);
    I2CMutex.lock();
    PROF_SECTION(power_bus_wait, t_wait);
    PROF_START(t_read);
    const float amps = ina.getCurrent();
    const float volts = ina.getBusVoltage();
    PROF_SECTION(power_read, t_read);
    I2CMutex.unlock();
    PROF_COUNT(power_reads);

    power_sample.write({amps, volts});

    threads.delay(cfg::INA226_SENSOR::poll_ms);
  }
}

void ReadIMUStates()
{
  static Adafruit_LSM9DS1 lsm;

  while (true)
  {
    I2CMutex.lock();
    const bool found = lsm.begin();
    if (found)
    {
      lsm.setupAccel(accelRange(cfg::LSM9DS1_IMU::accel_range_g), accelRate(cfg::LSM9DS1_IMU::accel_rate_hz));
      lsm.setupMag(magGain(cfg::LSM9DS1_IMU::mag_range_gauss));
      lsm.setupGyro(gyroScale(cfg::LSM9DS1_IMU::gyro_range_dps));
      // Adafruit BusIO's begin() calls Wire.begin(), which resets the bus to
      // 100 kHz; apply the configured clock again.
      Wire.setClock(cfg::I2C::clock_hz);
      imu_regs.ctrl1_g = readRegister(LSM9DS1_ADDRESS_ACCELGYRO, 0x10);
      imu_regs.ctrl6_xl = readRegister(LSM9DS1_ADDRESS_ACCELGYRO, 0x20);
      imu_regs.ctrl1_m = readRegister(LSM9DS1_ADDRESS_MAG, 0x20);
      imu_regs.ctrl2_m = readRegister(LSM9DS1_ADDRESS_MAG, 0x21);
      imu_regs.ctrl3_m = readRegister(LSM9DS1_ADDRESS_MAG, 0x22);
      imu_regs.read = true;
    }
    I2CMutex.unlock();
    if (found)
      break;
    wdt::beat(wdt::imu);
    threads.delay(sensor_retry_ms);
  }
  imu_ok = true;

  uint32_t next_us = micros();
  while (true)
  {
    wdt::beat(wdt::imu);
    WDT_TEST_HANG('i');
    while ((int32_t)(micros() - next_us) < 0)
      threads.yield();
    next_us += cfg::LSM9DS1_IMU::read_period_us;
    if ((int32_t)(micros() - next_us) >= 0)
      next_us = micros() + cfg::LSM9DS1_IMU::read_period_us;

    PROF_PERIOD(imu_loop);
    sensors_event_t a, m, g;
    PROF_START(t_wait);
    I2CMutex.lock();
    PROF_SECTION(imu_bus_wait, t_wait);
    PROF_START(t_read);
    // One transfer each. getEvent(a, m, g, temp) also reads the temperature
    // and reads the magnetometer twice.
    lsm.getAccel().getEvent(&a);
    lsm.getGyro().getEvent(&g);
    lsm.getMag().getEvent(&m);
    PROF_SECTION(imu_read, t_read);
    I2CMutex.unlock();
#ifdef RCN_PROFILE
    static float last_sample[3] = {0, 0, 0};
    if (a.acceleration.x != last_sample[0] || a.acceleration.z != last_sample[1] || g.gyro.z != last_sample[2])
    {
      PROF_COUNT(imu_distinct);
      last_sample[0] = a.acceleration.x;
      last_sample[1] = a.acceleration.z;
      last_sample[2] = g.gyro.z;
    }
#endif

    imu_sample.write({{a.acceleration.x, a.acceleration.y, a.acceleration.z,
                       g.gyro.x, g.gyro.y, g.gyro.z,
                       m.magnetic.x, m.magnetic.y, m.magnetic.z}});
  }
}

void RxReceive()
{
  PPMReader ppm(cfg::FLYSKY_IA6B_RX::ppm_pin, cfg::FLYSKY_IA6B_RX::channel_count);

  while (true)
  {
    wdt::beat(wdt::rc_rx);
    WDT_TEST_HANG('c');
    PROF_PERIOD(rc_loop);
    uint16_t values[cfg::FLYSKY_IA6B_RX::channel_count];
    for (uint8_t i = 0; i < cfg::FLYSKY_IA6B_RX::channel_count; i++)
      values[i] = ppm.latestValidChannelValue(i + 1, 0);

    rc_channels.beginWrite();
    memcpy(rc_channels.data().us, values, sizeof(values));
    rc_channels.endWrite();

    threads.delay(cfg::TEENSY_THREADS::tick_ms);
  }
}
