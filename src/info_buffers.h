#pragma once
#include <stddef.h>
#include <stdint.h>

// Two info payload buffers handed from the builder thread to the telemetry
// interrupt. The interrupt takes the published buffer at the start of each
// chunk cycle and sends from it until the cycle ends; the builder writes the
// other buffer, and only once the interrupt has taken the last one, so a
// buffer is never written while chunks are sent from it. One writer each;
// Arduino-free and host tested.
template <size_t N>
class InfoBuffers
{
public:
    struct Payload
    {
        char data[N];
        size_t len;
        uint8_t revision;
    };

    // Builder: the buffer to fill next, or nullptr while the interrupt has
    // not taken the last published one.
    Payload *beginBuild()
    {
        if (has_published_ && taken_ != published_)
            return nullptr;
        return &buf_[1 - published_];
    }

    // Builder: makes the buffer from beginBuild() the one the next cycle sends.
    void publish()
    {
        fence();
        published_ = (uint8_t)(1 - published_);
        has_published_ = true;
    }

    // Interrupt: at the start of a chunk cycle; nullptr before the first
    // publish.
    const Payload *take()
    {
        if (!has_published_)
            return nullptr;
        taken_ = published_;
        fence();
        return &buf_[taken_];
    }

private:
    static void fence() { __atomic_signal_fence(__ATOMIC_SEQ_CST); }

    Payload buf_[2] = {};
    volatile uint8_t published_ = 0;
    volatile uint8_t taken_ = 0;
    volatile bool has_published_ = false;
};
