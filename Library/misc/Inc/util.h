//******************************************************************************
// @addtogroup MISC
// @file       misc_util.h
// @brief      汎用的に使う関数郡
//******************************************************************************
#pragma once

#include "stdint.h"

namespace misc {

#undef PI
constexpr float PI = 3.14159265358979323846f;

template <typename T>
inline T min(T a, T b) {
    return a < b ? a : b;
}

template <typename T>
inline T min(T a, T b, T c, T d) {
    return min(min(a, b), min(c, d));
}

template <typename T>
T min(T* array, uint16_t len) {
    T min = array[0];
    for (uint16_t i = 0; i < len; ++i) {
        if (array[i] < min) {
            min = array[i];
        }
    }
    return min;
}

template <typename T>
inline T max(T a, T b) {
    return a > b ? a : b;
}

template <typename T>
inline T max(T a, T b, T c, T d) {
    return max(max(a, b), max(c, d));
}

template <typename T>
T max(T* array, uint16_t len) {
    T max = array[0];
    for (uint16_t i = 0; i < len; ++i) {
        if (array[i] > max) {
            max = array[i];
        }
    }
    return max;
}

template <typename T>
T average(T* array, uint16_t len) {
    T sum = 0;
    for (uint16_t i = 0; i < len; ++i) {
        sum += (array[i] / len);
    }
    return sum;
}

template <typename T>
T abs(T a) {
    return a < 0 ? -a : a;
}

template <typename T>
struct Point {
    T x;
    T y;
};

// 優先度無しで、ヒープ領域を使用しないキュー
template <typename T, uint16_t SIZE>
class Queue {
   private:
    T data[SIZE];
    uint16_t head;
    uint16_t tail;

   public:
    Queue() : head(0), tail(0) {}

    void push(T value) {
        data[tail] = value;
        tail = (tail + 1) % SIZE;
    }

    T front() {
        return data[head];
    }

    void pop() {
        head = (head + 1) % SIZE;
    }

    bool empty() {
        return head == tail;
    }
};

}  // namespace misc
