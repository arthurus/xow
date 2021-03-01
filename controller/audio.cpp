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

// From the users' perspective
#define STREAM_NAME_OUT "output"
#define SINK_NAME "XboxOutput"
#define SINK_MODULE_NAME "module-null-sink"
#define SINK_MODULE_ARGS "sink_name=" SINK_NAME " sink_properties=device.description=Xbox_controller_sink"

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

static void get_module_info_cb(pa_context *ctx, const pa_module_info *i, int is_last, void *userdata)
{
    int *module_idx = (int *)userdata;

    if (is_last)
    {
        if (*module_idx == -2)
            *module_idx = -1;
        return;
    }

    if (strcmp(i->name, SINK_MODULE_NAME) == 0 &&
        strcmp(i->argument, SINK_MODULE_ARGS) == 0)
            *module_idx = i->index;
}

static void load_module_cb(pa_context *ctx, uint32_t idx, void *userdata)
{
    int *module_idx = (int *)userdata;

    *module_idx = idx;
}

static void get_sink_input_info_list_cb(pa_context *ctx, const pa_sink_input_info *i, int eol, void *userdata)
{
    int *sink_idx = (int *)userdata;

    if (eol)
    {
        *sink_idx = -1;
        return;
    }

    if (i->sink != (uint32_t)*sink_idx)
    {
        Log::debug("Moving stream #%d '%s'", i->index, i->name);
        auto op = pa_context_move_sink_input_by_name(ctx, i->index, SINK_NAME, nullptr, nullptr);
        pa_operation_unref(op);
    }
}

static void get_sink_info_cb(pa_context *ctx, const pa_sink_info *i, int eol, void *userdata)
{
    int *sink_idx = (int *)userdata;

    if (eol)
    {
        if (*sink_idx == -2)
            *sink_idx = -1;
        return;
    }

    *sink_idx = i->index;
}

static void context_status_cb(pa_context *ctx, int success, void *userdata)
{
    int *status = (int *)userdata;

    *status = success ? 1 : -1;
}

void AudioStream::mainLoop(std::promise<bool> ready)
{
    pa_mainloop_api *api = pa_mainloop_get_api(paMainloop);
    pa_context *ctx = pa_context_new(api, name.c_str());
    pa_stream *stream = nullptr;
    int module_idx = -2;
    int sink_idx = -2;
    int status;

    try
    {
        int ret = pa_context_connect(ctx, nullptr, PA_CONTEXT_NOFLAGS, nullptr);
        if (ret < 0)
            throw AudioException("Failed to connect context", ret);

        status = 0;
        pa_context_set_state_callback(ctx, pa_state_cb, &status);
        while (status == 0)
            pa_mainloop_iterate(paMainloop, 1, nullptr);

        if (status == 2)
            throw AudioException("Failed to connect context");

        auto op = pa_context_get_module_info_list(ctx, get_module_info_cb, &module_idx);
        while (module_idx == -2)
            pa_mainloop_iterate(paMainloop, 1, nullptr);
        pa_operation_unref(op);

        if (module_idx == -1)
        {
            module_idx = -2;
            op = pa_context_load_module(ctx, SINK_MODULE_NAME, SINK_MODULE_ARGS, load_module_cb, &module_idx);
            while (module_idx == -2)
                pa_mainloop_iterate(paMainloop, 1, nullptr);
            pa_operation_unref(op);

            if (module_idx == -1)
                throw AudioException("Failed to load module");
        }

        op = pa_context_get_sink_info_by_name(ctx, SINK_NAME, get_sink_info_cb, &sink_idx);
        while (sink_idx == -2)
            pa_mainloop_iterate(paMainloop, 1, nullptr);
        pa_operation_unref(op);

        if (sink_idx == -1)
            throw AudioException("Failed to get sink index");

        status = 0;
        op = pa_context_set_default_sink(ctx, SINK_NAME, context_status_cb, &status);
        while (status == 0)
            pa_mainloop_iterate(paMainloop, 1, nullptr);
        pa_operation_unref(op);

        if (status != 1)
            throw AudioException("Failed to set default sink");

        status = sink_idx;
        op = pa_context_get_sink_input_info_list(ctx, get_sink_input_info_list_cb, &status);
        while (status == sink_idx)
            pa_mainloop_iterate(paMainloop, 1, nullptr);
        pa_operation_unref(op);

        stream = pa_stream_new(ctx, STREAM_NAME_OUT, &ss, nullptr);
        if (!stream)
            throw AudioException("Failed to create stream");

        int stream_state = 0;
        pa_stream_set_state_callback(stream, stream_state_cb, &stream_state);
        pa_stream_set_read_callback(stream, stream_read_cb, &buf);

        pa_buffer_attr bufattr = {(uint32_t)-1};
        bufattr.fragsize = pa_usec_to_bytes(latencyUs, &ss);

        ret = pa_stream_connect_record(stream, SINK_NAME".monitor", &bufattr, PA_STREAM_ADJUST_LATENCY);
        if (ret < 0)
            throw AudioException("Failed to connect stream", ret);

        while (stream_state == 0)
            pa_mainloop_iterate(paMainloop, 1, nullptr);

        if (stream_state == 2)
            throw AudioException("Failed to connect stream");
    }
    catch (...)
    {
        ready.set_exception(std::current_exception());
        goto out;
    }

    ready.set_value(true);

    while (threadQuit == false)
        pa_mainloop_iterate(paMainloop, 1, nullptr);

    status = 0;
    pa_operation_unref(pa_context_unload_module(ctx, module_idx, context_status_cb, &status));
    while (status == 0)
        pa_mainloop_iterate(paMainloop, 1, nullptr);

    if (status != 1)
        throw AudioException("Failed to unload module");

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
