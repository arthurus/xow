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

#include "audio.h"
#include "../utils/bytes.h"

#include <thread>
#include <pulse/simple.h>
#include <pulse/error.h>

// From the users' perspective
#define STREAM_NAME_SOURCE "output"

AudioStream::AudioStream(SamplesRead samplesRead) :
    state(STATE_STOPPED), samplesRead(samplesRead) {}

AudioStream::~AudioStream()
{
    stop();
}

void AudioStream::start(
    std::string name,
    uint32_t sampleRate,
    uint8_t channels,
    size_t sampleCount
) {
    if (state != STATE_STOPPED)
    {
        throw AudioException("Stream is currently running");
    }

    int error = 0;
    pa_sample_spec config = {};

    config.format = PA_SAMPLE_S16LE;
    config.rate = sampleRate;
    config.channels = channels;

    pa_conn = pa_simple_new(
        nullptr,
        name.c_str(),
        PA_STREAM_RECORD,
        "XboxOutput.monitor",
        STREAM_NAME_SOURCE,
        &config,
        nullptr,
        nullptr,
        &error
    );

    if (!pa_conn)
    {
        throw AudioException("Error creating source", error);
    }

    state = STATE_RUNNING;

    std::thread(&AudioStream::read, this, sampleCount).detach();
}

void AudioStream::stop()
{
    state = STATE_STOPPING;
}

void AudioStream::read(size_t sampleCount)
{
    int error = 0;
    Bytes samples(sampleCount);

    while (state == STATE_RUNNING)
    {
        if (pa_simple_read(
            pa_conn,
            samples.raw(),
            samples.size(),
            &error
        ) < 0) {
            throw AudioException("Error reading from source", error);
        }

        if (state == STATE_RUNNING)
        {
            samplesRead(samples);
        }
    }

    pa_simple_free(pa_conn);

    state = STATE_STOPPED;
}

AudioException::AudioException(std::string message) :
    std::runtime_error(message) {}

AudioException::AudioException(std::string message, int error) :
    std::runtime_error(message + ": " + pa_strerror(error)) {}
