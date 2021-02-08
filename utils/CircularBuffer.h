#ifndef UTILS_CIRCULARBUFFER_H_
#define UTILS_CIRCULARBUFFER_H_

#include <cstddef>
#include <cstring>
#include <algorithm>
#include <mutex>
#include <condition_variable>

template<typename T>
class CircularBuffer {
public:
    CircularBuffer(std::size_t capacity) :
        capacity(capacity),
        available(0)
    {
        buffer = new T[capacity];
        read_pos = write_pos = buffer;
    }

    virtual ~CircularBuffer()
    {
        delete[] buffer;
    }

    void read(T *into, std::size_t count)
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [=]{ return available >= count; });
        read_write(&read_pos, into, count);
        available -= count;
    }

    void write(const T *from, std::size_t count)
    {
        std::lock_guard<std::mutex> lock(mutex);
        read_write(&write_pos, const_cast<T *>(from), count);
        available += count;
        cv.notify_one();
    }

private:
    void read_write(T **ppos, T *buf, std::size_t count)
    {
        while (count > 0) {
            T * const pos = *ppos;
            auto to_end = capacity - (pos - buffer);
            auto n = std::min(count, to_end);

            if (ppos == &write_pos)
                std::memcpy(pos, buf, n * sizeof(T));
            else
                memcpy(buf, pos, n * sizeof(T));
            count -= n;
            buf += n;
            *ppos += n;
            if (pos == (buffer + capacity))
                *ppos = buffer;
        }
    }

    std::size_t capacity;
    std::size_t available;
    T *buffer;
    T *read_pos;
    T *write_pos;
    std::mutex mutex;
    std::condition_variable cv;
};

#endif /* UTILS_CIRCULARBUFFER_H_ */
