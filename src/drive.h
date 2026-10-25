#pragma once

// Hall pins and their interrupts; call once from setup().
void initHallEncoder();

// Actuation thread, 1 kHz: RC-over-Pi arbitration, the speed loop, servo and
// ESC PWM. Sole writer of measured_speed_mps, manual_override and rc_driving.
void DriveControl();
