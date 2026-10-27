#pragma once
#include <stdint.h>

// Single-writer value shared with interrupt handlers, guarded by a sequence
// counter instead of a mutex (an interrupt cannot wait on a lock). The count
// is odd while a write is in progress. Arduino-free and host tested.
//
// One core: a reader in an interrupt that sees an even count reads a value
// that stays still, since the writer cannot run until the interrupt returns.
// A reader in a thread retries if a write happened during its copy.
template <typename T>
class Seqlock
{
public:
    void write(const T &value)
    {
        beginWrite();
        data_ = value;
        endWrite();
    }

    void beginWrite()
    {
        seq_ = seq_ + 1;
        fence();
    }

    void endWrite()
    {
        fence();
        seq_ = seq_ + 1;
    }

    // Writable in place between beginWrite() and endWrite().
    T &data() { return data_; }

    // One attempt; false, with `out` unchanged, while a write is in progress
    // or when one landed during the copy.
    bool tryRead(T &out) const
    {
        const uint32_t start = seq_;
        fence();
        if (start & 1u)
            return false;
        const T copy = data_;
        fence();
        if (seq_ != start)
            return false;
        out = copy;
        return true;
    }

    uint32_t sequence() const { return seq_; }

private:
    static void fence() { __atomic_signal_fence(__ATOMIC_SEQ_CST); }

    volatile uint32_t seq_ = 0;
    T data_{};
};
