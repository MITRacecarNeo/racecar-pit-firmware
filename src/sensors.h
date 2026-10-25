#pragma once

// Sensor threads; started from setup() after Wire.begin().
void ReadCurrAndVolt(); // INA226 bus voltage and current
void ReadIMUStates();   // LSM9DS1 accel, gyro, mag
void RxReceive();       // FlySky PPM channels
