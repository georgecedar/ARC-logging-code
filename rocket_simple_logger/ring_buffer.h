#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "config.h"
#include "data_types.h"

class RingBuffer {
public:
    void push(const DataPoint& sample) {
        buffer_[head_] = sample;
        head_ = (head_ + 1) % RING_BUFFER_SIZE;
        if (count_ < RING_BUFFER_SIZE) count_++;
    }

    int count() const { return count_; }

    const DataPoint& at(int chronoIndex) const {
        int start = (count_ < RING_BUFFER_SIZE) ? 0 : head_;
        int idx = (start + chronoIndex) % RING_BUFFER_SIZE;
        return buffer_[idx];
    }

    int drain(void (*writeFn)(const DataPoint&)) {
        int n = count_;
        for (int i = 0; i < n; i++) {
            writeFn(at(i));
        }
        return n;
    }

    void clear() {
        head_ = 0;
        count_ = 0;
    }

private:
    DataPoint buffer_[RING_BUFFER_SIZE];
    int head_ = 0;
    int count_ = 0;
};

#endif
