#pragma once

#include "car_constants.h"

// Per-car speed constants on the Teensy: loaded from the EEPROM at boot
// (built-in fleet values when the block is missing or invalid), changed and
// saved with "cal ..." lines on USB serial (src/car_constants.h).

// From setup(), before the drive tick starts.
void loadSpeedConstants();

// The set in use. Safe from the drive tick: changes go to a second buffer
// that is swapped in whole.
const SpeedConstants &speedConstants();

// From loop(): reads USB serial lines and answers them.
void serviceSpeedConstants();

// Snapshot for the info reply; safe from any thread.
ConstantsInfo speedConstantsInfo();
