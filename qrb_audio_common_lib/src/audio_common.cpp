// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_audio_common_lib/audio_common.hpp"

#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <random>
#include <stdexcept>

#include "stdio.h"

namespace qrb
{
namespace audio_common_lib
{

pa_threaded_mainloop * CommonAudioStream::pulse_mainloop_ = nullptr;
pa_mainloop_api * CommonAudioStream::pulse_mainloop_api_ = nullptr;
pa_context * CommonAudioStream::pulse_context_ = nullptr;
pa_time_event * CommonAudioStream::pulse_context_time_event_ = nullptr;
pa_context_state_t CommonAudioStream::pulse_context_state_ = PA_CONTEXT_UNCONNECTED;
std::set<CommonAudioStream *> CommonAudioStream::timestamp_streams_;
std::unordered_map<uint32_t, CommonAudioStream *> CommonAudioStream::stream_list_;

uint32_t CommonAudioStream::add_stream(CommonAudioStream * stream)
{
  uint32_t handle = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dist(1, UINT32_MAX);

  for (;;) {
    handle = dist(gen);
    if (stream_list_.find(handle) == stream_list_.end())
      break;
  }

  stream_list_[handle] = stream;
  return handle;
}

void CommonAudioStream::delete_stream(CommonAudioStream * stream)
{
  for (auto it = stream_list_.begin(); it != stream_list_.end();) {
    if (it->second == stream) {
      stream_list_.erase(it->first);
      break;
    } else {
      ++it;
    }
  }
}

uint32_t CommonAudioStream::get_common_stream_handle(CommonAudioStream * stream)
{
  uint32_t handle = 0;

  for (auto it = stream_list_.begin(); it != stream_list_.end();) {
    if (it->second == stream) {
      handle = it->first;
      break;
    } else {
      ++it;
    }
  }

  return handle;
}

CommonAudioStream * CommonAudioStream::get_stream(uint32_t handle)
{
  auto it = stream_list_.find(handle);
  if (it != stream_list_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}

void CommonAudioStream::pulse_stream_update_timing_callback(pa_stream * stream,
    int success,
    void * userdata)
{
  pa_usec_t l;
  pa_usec_t usec;
  int negative = 0;

  CommonAudioStream * current_stream = static_cast<CommonAudioStream *>(userdata);

  uint32_t stream_handle = get_common_stream_handle(current_stream);

  if (!success || pa_stream_get_time(stream, &usec) < 0 ||
      pa_stream_get_latency(stream, &l, &negative) < 0) {
    printf(("Failed to get latency: %s"),
        pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
    return;
  }

  Stream_Event_Data stream_event_data = {
    .usec = usec - current_stream->start_usec,
  };

  current_stream->event_cb(StreamEvent::StreamTimestamp, stream_event_data, (void *)stream_handle);
}

void CommonAudioStream::pulse_context_time_event_callback(pa_mainloop_api * m,
    pa_time_event * e,
    const struct timeval * t,
    void * userdata)
{
  for (CommonAudioStream * stream : CommonAudioStream::timestamp_streams_) {
    if (stream && stream->get_stream_state() == PA_STREAM_READY) {
      pa_operation * o;
      if (!(o = pa_stream_update_timing_info(stream->get_stream_handle(),
                CommonAudioStream::pulse_stream_update_timing_callback, stream)))
        printf(("pa_stream_update_timing_info() failed: %s"),
            pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
      else
        pa_operation_unref(o);
    }
  }

  pa_context_rttime_restart(
      CommonAudioStream::pulse_context_, e, pa_rtclock_now() + TIME_EVENT_USEC);
}

int CommonAudioStream::register_stream_timestamp_event(CommonAudioStream * stream_handle)
{
  CommonAudioStream::timestamp_streams_.insert(stream_handle);

  if (CommonAudioStream::pulse_context_time_event_ == nullptr) {
    CommonAudioStream::pulse_context_time_event_ =
        pa_context_rttime_new(CommonAudioStream::pulse_context_, pa_rtclock_now() + TIME_EVENT_USEC,
            CommonAudioStream::pulse_context_time_event_callback, nullptr);
    if (!CommonAudioStream::pulse_context_time_event_) {
      printf("pa_context_rttime_new failed\n");
      return -1;
    } else {
      printf("init context time event succeed\n");
    }
  }

  return 0;
}

void CommonAudioStream::deregister_stream_timestamp_event(CommonAudioStream * stream_handle)
{
  CommonAudioStream::timestamp_streams_.erase(stream_handle);

  if ((CommonAudioStream::timestamp_streams_.size() == 0) &&
      CommonAudioStream::pulse_context_time_event_) {
    CommonAudioStream::pulse_mainloop_api_->time_free(CommonAudioStream::pulse_context_time_event_);
    CommonAudioStream::pulse_context_time_event_ = nullptr;
  }
}

void CommonAudioStream::pulse_context_state_callback(pa_context * context, void * userdata)
{
  assert(context);

  CommonAudioStream::pulse_context_state_ = pa_context_get_state(context);
  printf(
      "CommonAudioStream::pulse_context_state_callback:CommonAudioStream::"
      "pulse_context_state_ = %d\n",
      CommonAudioStream::pulse_context_state_);
  switch (CommonAudioStream::pulse_context_state_) {
    case PA_CONTEXT_CONNECTING:
    case PA_CONTEXT_AUTHORIZING:
    case PA_CONTEXT_SETTING_NAME:
      break;

    case PA_CONTEXT_READY:
      break;

    case PA_CONTEXT_TERMINATED:
      break;

    case PA_CONTEXT_FAILED:
      printf("pulse context state PA_CONTEXT_FAILED\n");
      pa_context_unref(pulse_context_);
      pulse_context_ = nullptr;
      init_pulse_env();
      break;
    default:
      printf("pulse context state update error: %s\n", pa_strerror(pa_context_errno(context)));
  }
}

void CommonAudioStream::pulse_stream_state_callback(pa_stream * stream, void * userdata)
{
  assert(stream);

  CommonAudioStream * current_stream = static_cast<CommonAudioStream *>(userdata);
  uint32_t stream_handle = get_common_stream_handle(current_stream);

  current_stream->set_stream_state(pa_stream_get_state(stream));
  printf("CommonAudioStream::pulse_stream_state_callback enter status = %d\n",
      current_stream->get_stream_state());

  Stream_Event_Data dummy_data;

  switch (current_stream->get_stream_state()) {
    case PA_STREAM_CREATING:
      break;
    case PA_STREAM_TERMINATED:
      if (current_stream->repeat_count == 0)
        current_stream->event_cb(StreamEvent::StreamStoped, dummy_data, (void *)stream_handle);
      break;
    case PA_STREAM_READY:
      break;
    case PA_STREAM_FAILED:
      current_stream->event_cb(StreamEvent::StreamAbort, dummy_data, (void *)stream_handle);
      break;
    default:
      printf("%s:pulse stream state update failed\n", __func__);
  }
  pa_threaded_mainloop_signal(CommonAudioStream::pulse_mainloop_, 0);
}

void CommonAudioStream::stream_underflow_callback(pa_stream * stream, void * userdata)
{
  assert(stream);
  printf("Stream underrun.\n");
}

void CommonAudioStream::stream_overflow_callback(pa_stream * stream, void * userdata)
{
  assert(stream);
  printf("Stream overrun.\n");
}

pa_threaded_mainloop * CommonAudioStream::init_pulse_env()
{
  bool first_init = false;

  if (CommonAudioStream::pulse_mainloop_ != nullptr && pulse_context_ != nullptr)
    return CommonAudioStream::pulse_mainloop_;

  if (CommonAudioStream::pulse_mainloop_ == nullptr) {
    first_init = true;
    CommonAudioStream::pulse_mainloop_ = pa_threaded_mainloop_new();
    if (!CommonAudioStream::pulse_mainloop_) {
      printf("CommonAudioStream::pulse_mainloop_ fail\n");
      return nullptr;
    }

    if (pa_threaded_mainloop_start(CommonAudioStream::pulse_mainloop_) < 0) {
      printf("pa_threaded_mainloop_start() failed.\n");
      pa_threaded_mainloop_free(pulse_mainloop_);
      CommonAudioStream::pulse_mainloop_ = nullptr;
      return nullptr;
    }
  }

  if (CommonAudioStream::pulse_mainloop_api_ == nullptr)
    CommonAudioStream::pulse_mainloop_api_ =
        pa_threaded_mainloop_get_api(CommonAudioStream::pulse_mainloop_);

  if (CommonAudioStream::pulse_context_ == nullptr)
    CommonAudioStream::pulse_context_ =
        pa_context_new(CommonAudioStream::pulse_mainloop_api_, "ros_context");

  if (CommonAudioStream::pulse_context_ == nullptr) {
    printf("pulseaudio context init fail\n");
    return nullptr;
  }

  pa_context_set_state_callback(
      CommonAudioStream::pulse_context_, CommonAudioStream::pulse_context_state_callback, nullptr);

  if (pa_context_connect(CommonAudioStream::pulse_context_, nullptr, PA_CONTEXT_NOFLAGS, nullptr) <
      0) {
    printf("connect pulseaudio server fail %s\n",
        pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
  }

  if (first_init)
    atexit(clean_pulse_mainloop);

  return CommonAudioStream::pulse_mainloop_;
}

void CommonAudioStream::clean_pulse_mainloop()
{
  if (pulse_context_)
    pa_context_unref(pulse_context_);

  if (pulse_context_time_event_) {
    if (pulse_mainloop_api_)
      pulse_mainloop_api_->time_free(pulse_context_time_event_);
  }

  if (pulse_mainloop_) {
    pa_threaded_mainloop_stop(pulse_mainloop_);
    pa_threaded_mainloop_free(pulse_mainloop_);
  };
}

void CommonAudioStream::check_context_ready()
{
  for (;;) {
    if (CommonAudioStream::pulse_context_state_ == PA_CONTEXT_READY)
      break;
    usleep(5000);
  }
}

CommonAudioStream::CommonAudioStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec)
  : m_file_path(filepath), m_sample_spec(sample_spec)
{
  if (CommonAudioStream::pulse_context_ == nullptr) {
    CommonAudioStream::init_pulse_env();
    CommonAudioStream::check_context_ready();
  }
}

CommonAudioStream::~CommonAudioStream()
{
  printf("delete Stream handle %p", this);
  switch (stream_state_) {
    case PA_STREAM_READY:
      stop_stream();
    case PA_STREAM_TERMINATED:
      close_stream();
      break;
    default:
      break;
  }

  if (snd_file) {
    sf_close(snd_file);
    snd_file = nullptr;
  }
}

uint32_t CommonAudioStream::audio_stream_open(const audio_stream_info & stream_info,
    stream_event_callback_func event_callback)
{
  uint32_t stream_handle = 0;
  CommonAudioStream * stream = nullptr;
  std::shared_ptr<pa_sample_spec> stream_sample_spec = std::make_shared<pa_sample_spec>();

  stream_sample_spec->rate = stream_info.rate;
  stream_sample_spec->channels = stream_info.channels;

  switch (stream_info.format) {
    case 8:
      stream_sample_spec->format = PA_SAMPLE_U8;
      break;
    case 16:
      stream_sample_spec->format = PA_SAMPLE_S16NE;
      break;
    case 24:
      stream_sample_spec->format = PA_SAMPLE_S24NE;
      break;
    case 32:
      stream_sample_spec->format = PA_SAMPLE_S32NE;
      break;
  }

  if (stream_info.file_path.empty() && (!pa_sample_spec_valid(stream_sample_spec.get()))) {
    printf("Stream sample spec is invalid\n");
    return stream_handle;
  }

  try {
    if (stream_info.type == StreamPlayback)
      stream = new PlaybackStream(string(stream_info.file_path), stream_sample_spec);
    else if (stream_info.type == StreamCapture)
      stream = new CaptureStream(string(stream_info.file_path), stream_sample_spec);
    else
      stream = nullptr;
  } catch (const std::runtime_error & e) {
    printf("create stream error:%s\n", e.what());
    stream = nullptr;
  }

  if (!stream)
    return stream_handle;
  printf("create stream succeed\n");

  if (stream_info.volume && stream_info.volume > STREAM_VOL_MIN &&
      stream_info.volume <= STREAM_VOL_MAX)
    stream->mvolume = stream_info.volume;
  else {
    printf("Wrong volume");
    return stream_handle;
  }

  stream->repeat_count = stream_info.repeat;

  stream->event_cb = event_callback;
  stream->pcm_mode_ = stream_info.pcm_mode;

  if (stream_info.need_timestamp) {
    CommonAudioStream::register_stream_timestamp_event(stream);
  }

  return add_stream(stream);
}

void CommonAudioStream::init_pulse_stream()
{
  stream_handle_ = pa_stream_new(CommonAudioStream::pulse_context_,
      std::to_string(intptr_t(this)).c_str(), m_sample_spec.get(), nullptr);
  if (stream_handle_ == nullptr) {
    printf("create stream:%s\n", pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
    throw std::runtime_error("create pulseaudio stream return nullptr");
  }

  pa_stream_set_state_callback(
      stream_handle_, CommonAudioStream::pulse_stream_state_callback, this);

  pa_stream_set_underflow_callback(
      stream_handle_, CommonAudioStream::stream_underflow_callback, nullptr);
  pa_stream_set_overflow_callback(
      stream_handle_, CommonAudioStream::stream_overflow_callback, nullptr);
}

int CommonAudioStream::pause_stream(bool pause)
{
  if (stream_state_ != PA_STREAM_READY) {
    printf("stream state(%d) error, pause fail\n", stream_state_);
    return -1;
  }

  pa_operation * op_pause = pa_stream_cork(stream_handle_, pause, nullptr, nullptr);
  if (op_pause) {
    pa_operation_unref(op_pause);
  }

  return 0;
}

int CommonAudioStream::mute_stream(bool mute)
{
  uint32_t stream_index;
  pa_operation * op;

  stream_index = pa_stream_get_index(stream_handle_);
  op = pa_context_set_sink_input_mute(
      CommonAudioStream::pulse_context_, stream_index, mute, nullptr, nullptr);
  if (!op) {
    printf("%s:failed to set stream(%d) to mute(%d)", stream_handle_, mute);
    pa_operation_unref(op);
  }

  return 0;
}

int CommonAudioStream::stop_stream()
{
  printf("stream state(%d), stop_stream enter\n", stream_state_);

  if (stream_state_ == PA_STREAM_TERMINATED) {
    printf("stream state(%d) already in stop\n", stream_state_);
    return 0;
  }

  if (stream_state_ != PA_STREAM_READY) {
    printf("stream state(%d) error, Stop fail\n", stream_state_);
    return -EPERM;
  }

  CommonAudioStream::deregister_stream_timestamp_event(this);

  if (pa_stream_disconnect(stream_handle_) < 0) {  // after disconnect state will be
                                                   // PA_STREAM_TERMINATED(4)
    printf("Disconnect Stream fail\n");
  }

  pa_threaded_mainloop_lock(CommonAudioStream::pulse_mainloop_);
  while (stream_state_ != PA_STREAM_TERMINATED) {
    pa_threaded_mainloop_wait(CommonAudioStream::pulse_mainloop_);
  }
  pa_threaded_mainloop_unlock(CommonAudioStream::pulse_mainloop_);
  printf("stop_stream complete, stream_state_ = %d\n", stream_state_);
  return 0;
}

void CommonAudioStream::internal_stopstream()
{
  std::thread t(&CommonAudioStream::stop_stream, this);
  t.detach();
}

int CommonAudioStream::close_stream()
{
  printf(
      "stream state(%d), stream_handle_(%p) close_stream enter\n", stream_state_, stream_handle_);

  if ((stream_state_ != PA_STREAM_TERMINATED) && (stream_state_ != PA_STREAM_FAILED)) {
    printf("stream state(%d) error, Close fail\n", stream_state_);
    return -EPERM;
  }
  if (stream_handle_) {
    pa_stream_unref(stream_handle_);
    stream_handle_ = nullptr;
  }

  delete_stream(this);

  if (stream_handle_ == nullptr)
    return 0;
  return -EPERM;
}

}  // namespace audio_common_lib
}  // namespace qrb
