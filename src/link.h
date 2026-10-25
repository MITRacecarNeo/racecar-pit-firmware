#pragma once

// Pi UART link threads; started from setup() after Serial1.begin().
void UARTTx(); // telemetry every slot, plus one info chunk while the Pi requests it
void UARTRx(); // command frames into the shared Pi command state
