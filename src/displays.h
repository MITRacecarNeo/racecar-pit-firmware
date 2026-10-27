#pragma once

// LED strip setup; call once from setup().
void initLedStrip();

// Dot matrix thread: boot splash, then the Pi bitmap or the drive-mode glyph.
void DotMatrix();

// LED strip frame: loading sweep, Pi pixels, or the idle charge bar. Called
// from loop(), not its own thread.
void updateLedStrip();
