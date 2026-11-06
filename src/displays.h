#pragma once

// LED strip setup; call once from setup().
void initLedStrip();

// Dot matrix thread: LOAD, OFF, blank, the Pi bitmap or stall words
// (src/display_state.h).
void DotMatrix();

// LED strip frame: loading waves, battery meter, Pi pixels, driver-start
// rainbow or dark (src/display_state.h). Called from loop(), not its own thread.
void updateLedStrip();
