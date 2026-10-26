#pragma once

// Hall pins and their interrupts; call once from setup().
void initHallEncoder();

// Starts the drive loop: a 1 kHz timer interrupt doing RC-over-Pi
// arbitration, the speed loop, and servo and ESC PWM. Sole writer of
// measured_speed_mps, manual_override and rc_driving.
void startDriveControl();

// Prints the speed-loop trace when cfg::DEBUG::serial_plot is set; call from
// loop().
void printDriveTrace();
