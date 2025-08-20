// src/codec/util/ring_buffer.hpp
#pragma once
#include <vector>
#include <cstddef>
#include <atomic>
#include <type_traits>

namespace pcc::util {

template <class T>
class ring_buffer {
public:
    explicit ring_buffer(std::size_t capacity)
    : buf_(capacity + 1) 
    , head_(0), tail_(0) {}

    ring_buffer(const ring_buffer&) = delete;
    ring_buffer& operator=(const ring_buffer&) = delete;

    bool push(const T& v) {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        const std::size_t n = next(h);
        if (n == tail_.load(std::memory_order_acquire)) {
            return false; 
        }
        buf_[h] = v;
        head_.store(n, std::memory_order_release);
        return true;
    }

    bool push(T&& v) {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        const std::size_t n = next(h);
        if (n == tail_.load(std::memory_order_acquire)) {
            return false; 
        }
        buf_[h] = std::move(v);
        head_.store(n, std::memory_order_release);
        return true;
    }

    bool pop(T& out) {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) {
            return false; 
        }
        out = std::move(buf_[t]);
        tail_.store(next(t), std::memory_order_release);
        return true;
    }

    std::size_t size() const {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        if (h >= t) return h - t;
        return capacity() + 1 - (t - h);
    }

    std::size_t capacity() const { return buf_.size() - 1; }
    bool empty() const { return size() == 0; }
    bool full() const { return size() == capacity(); }

private:
    std::size_t next(std::size_t i) const noexcept {
        ++i;
        if (i == buf_.size()) i = 0;
        return i;
    }

    std::vector<T> buf_;
    std::atomic<std::size_t> head_;
    std::atomic<std::size_t> tail_;
};

} 
