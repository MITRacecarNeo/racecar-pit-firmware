#pragma once

// Starts the telemetry timer interrupt: one frame every 1/tx_freq, plus one
// info chunk while the Pi requests it. Call from setup() after Serial1.begin().
void startTelemetry();

// Thread: builds the info payload while the Pi requests it.
void InfoBuilder();

// Thread: command frames into the shared Pi command state.
void UARTRx();
