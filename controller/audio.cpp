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
#include "../utils/log.h"

#include <cstring>

AudioStream::AudioStream(
    std::string name,
    uint32_t sampleRate,
    uint8_t channels,
    std::chrono::milliseconds latency
) :
    name(name),
    ss { PA_SAMPLE_S16LE, sampleRate, channels },
    latencyUs(latency.count() * 1000),
    threadQuit(false),
    buf(pa_usec_to_bytes(latencyUs, &ss) * 4)
{
    paMainloop = pa_mainloop_new();

    std::promise<bool> ready;
    auto future_ready = ready.get_future();
    thread = std::thread(&AudioStream::mainLoop, this, std::move(ready));
    future_ready.get();
}

AudioStream::~AudioStream()
{
    threadQuit = true;
    pa_mainloop_wakeup(paMainloop);
    thread.join();
    pa_mainloop_free(paMainloop);
}

CircularBuffer<uint8_t>& AudioStream::getBuffer()
{
    return buf;
}

static void pa_state_cb(pa_context *c, void *userdata)
{
    int *status = (int *)userdata;
    switch (pa_context_get_state(c)) {
    // These are just here for reference
    case PA_CONTEXT_UNCONNECTED:
    case PA_CONTEXT_CONNECTING:
    case PA_CONTEXT_AUTHORIZING:
    case PA_CONTEXT_SETTING_NAME:
    default:
        break;
    case PA_CONTEXT_FAILED:
    case PA_CONTEXT_TERMINATED:
        *status = 2;
        break;
    case PA_CONTEXT_READY:
        *status = 1;
        break;
    }
}

static void stream_state_cb(pa_stream *s, void *userdata)
{
    int *state = (int *)userdata;

    switch (pa_stream_get_state(s)) {
    case PA_STREAM_READY:
        *state = 1;
        break;
    case PA_STREAM_FAILED:
    case PA_STREAM_TERMINATED:
        *state = 2;
        break;
    case PA_STREAM_UNCONNECTED:
    case PA_STREAM_CREATING:
        break;
    }
}

static void stream_read_cb(pa_stream *stream, size_t length, void *userdata)
{
    auto buf = static_cast<CircularBuffer<uint8_t> *>(userdata);
    const void *data;

    int ret = pa_stream_peek(stream, &data, &length);
    if (ret < 0)
        throw AudioException("Error reading from stream", ret);

    if (length > 0) {
        if (data != nullptr)
            buf->write(static_cast<const uint8_t *>(data), length);
        pa_stream_drop(stream);
    }
}

static void get_sink_info_cb(pa_context *ctx, const pa_sink_info *i, int eol, void *userdata)
{
    auto out = (const char **)userdata;

    if (eol)
    {
        if (*out == nullptr)
            *out = (const char *)1;
        return;
    }

    *out = strdup(i->monitor_source_name);
}

void AudioStream::mainLoop(std::promise<bool> ready)
{
    pa_mainloop_api *api = pa_mainloop_get_api(paMainloop);
    pa_context *ctx = pa_context_new(api, name.c_str());
    pa_stream *stream = nullptr;

    try
    {
        int ret = pa_context_connect(ctx, nullptr, PA_CONTEXT_NOFLAGS, nullptr);
        if (ret < 0)
            throw AudioException("Failed to connect context", ret);

        int status = 0;
        pa_context_set_state_callback(ctx, pa_state_cb, &status);
        while (status == 0)
            pa_mainloop_iterate(paMainloop, 1, nullptr);

        if (status == 2)
            throw AudioException("Failed to connect context");

        const char *default_sink_monitor_name = nullptr;
        auto op = pa_context_get_sink_info_by_name(ctx, "@DEFAULT_SINK@", get_sink_info_cb, &default_sink_monitor_name);
        while (default_sink_monitor_name == nullptr)
            pa_mainloop_iterate(paMainloop, 1, nullptr);
        pa_operation_unref(op);

        if (default_sink_monitor_name == (void *)1)
            throw AudioException("Failed to get default sink monitor");

        Log::debug("Capturing from %s", default_sink_monitor_name);

        stream = pa_stream_new(ctx, "Xbox controller output", &ss, nullptr);
        if (!stream)
            throw AudioException("Failed to create stream");

        int stream_state = 0;
        pa_stream_set_state_callback(stream, stream_state_cb, &stream_state);
        pa_stream_set_read_callback(stream, stream_read_cb, &buf);

        pa_buffer_attr bufattr = {(uint32_t)-1};
        bufattr.fragsize = pa_usec_to_bytes(latencyUs, &ss);

        ret = pa_stream_connect_record(stream, default_sink_monitor_name, &bufattr, PA_STREAM_ADJUST_LATENCY);
        if (ret < 0)
            throw AudioException("Failed to connect stream", ret);

        while (stream_state == 0)
            pa_mainloop_iterate(paMainloop, 1, nullptr);

        if (stream_state == 2)
            throw AudioException("Failed to connect stream");

        free((void *)default_sink_monitor_name);
    }
    catch (...)
    {
        ready.set_exception(std::current_exception());
        goto out;
    }

    ready.set_value(true);

    while (threadQuit == false)
        pa_mainloop_iterate(paMainloop, 1, nullptr);

    pa_stream_disconnect(stream);
    pa_context_disconnect(ctx);

    Log::info("Audio stopped");
out:
    if (stream)
        pa_stream_unref(stream);
    if (ctx)
        pa_context_unref(ctx);
}

AudioException::AudioException(std::string message) :
    std::runtime_error(message) {}

AudioException::AudioException(std::string message, int error) :
    std::runtime_error(message + ": " + pa_strerror(error)) {}
