/*
 * Copyright (C) 2020 Medusalix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <atomic>
#include <string>
#include <stdexcept>
#include <thread>
#include <future>
#include <chrono>
#include <pulse/pulseaudio.h>
#include "../utils/CircularBuffer.h"

class Bytes;

/*
 * Bidirectional audio stream
 * Provides async input and sync output functionality
 */
class AudioStream
{
public:
    AudioStream(
        std::string name,
        uint32_t sampleRate,
        uint8_t channels,
        std::chrono::milliseconds latency
    );
    virtual ~AudioStream();

    CircularBuffer<uint8_t>& getBuffer();

private:
    void mainLoop(std::promise<bool> ready);

    std::string name;
    pa_sample_spec ss;
    uint32_t latencyUs;
    std::thread thread;
    std::atomic<bool> threadQuit;
    pa_mainloop *paMainloop;
    CircularBuffer<uint8_t> buf;
};

class AudioException : public std::runtime_error
{
public:
    AudioException(std::string message);
    AudioException(std::string message, int error);
};
