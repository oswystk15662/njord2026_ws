#include "zed2i_driver/ground_video_streamer.hpp"

#include <cuda_runtime_api.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

// This path does not use the libnvjpeg CUDA library. On the target Jetson
// Orin Nano Super image (Driver 595.78, CUDA 13.2, L4T 39.2), nvjpegCreate()/
// nvjpegCreateSimple() return NVJPEG_STATUS_EXECUTION_FAILED (status 6) for
// every backend (0-5), confirmed on-device with a minimal repro
// (/tmp/nvjpeg_repro.cu on the Jetson). The device has no /dev/nvhost-nvjpg*
// node at all (`ls /dev/nvhost-nvjpg*` -> no such file), so the hardware
// NVJPEG engine is not exposed to userspace on this image/kernel
// (6.8.12-1021-tegra); this is a platform/image issue, not an application
// bug, and every further nvjpeg call on the resulting handle just propagates
// failure (observed as NVJPEG_STATUS_INVALID_PARAMETER on a later run; an
// earlier run instead hit SIGILL inside libnvjpeg on the same bad handle --
// both are consistent with "the handle nvjpegCreate returned is not valid").
// Encoding is instead delegated to the GStreamer `nvjpegenc` element
// (gst-plugins-good, package libgstnvjpeg.so, confirmed present via
// `gst-inspect-1.0 nvjpegenc` on-device), which drives the Tegra NVJPG
// hardware block directly and is unaffected by the broken libnvjpeg entry
// points. Its sink pad only accepts I420/NV12 (4:2:0), which happens to be
// exactly the subsampling `rtpjpegpay` (RFC 2435) requires -- payloading a
// 4:4:4 JPEG produces "Invalid component" and drops every frame -- so the
// pipeline below (I420 all the way to nvjpegenc) satisfies that constraint
// by construction. This file therefore only uses CUDA for the cheap GPU-side
// resize + device-to-host copy of the downscaled BGR frame; the appsrc feeds
// raw BGR video into videoconvert/nvvidconv/nvjpegenc, which does the actual
// JPEG encoding.

namespace zed2i_driver
{
namespace
{

void check_cuda(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

__global__ void resize_bgra_to_bgri(
  const std::uint8_t * source, std::size_t source_pitch, int source_width, int source_height,
  std::uint8_t * destination, int destination_width, int destination_height)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= destination_width || y >= destination_height) {
    return;
  }
  const int source_x = x * source_width / destination_width;
  const int source_y = y * source_height / destination_height;
  const auto * input = source + static_cast<std::size_t>(source_y) * source_pitch + source_x * 4;
  auto * output = destination + (static_cast<std::size_t>(y) * destination_width + x) * 3;
  output[0] = input[0];
  output[1] = input[1];
  output[2] = input[2];
}

}  // namespace

class GroundVideoStreamer::Impl
{
public:
  explicit Impl(const GroundVideoConfig & config)
  : config_(config), period_(std::chrono::duration<double>(1.0 / config.fps))
  {
    validate();
    gst_init(nullptr, nullptr);
    create_pipeline();
    try {
      check_cuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreateWithFlags");
      slots_.resize(static_cast<std::size_t>(config_.max_pending_frames + 1));
      for (auto & slot : slots_) {
        check_cuda(cudaMalloc(&slot.bgra, max_bgra_bytes()), "cudaMalloc BGRA slot");
        check_cuda(cudaMalloc(&slot.bgri, max_bgri_bytes()), "cudaMalloc BGR slot");
        check_cuda(cudaMallocHost(&slot.host, max_bgri_bytes()), "cudaMallocHost BGR slot");
        check_cuda(cudaEventCreateWithFlags(&slot.ready, cudaEventDisableTiming), "cudaEventCreateWithFlags");
      }
      worker_ = std::thread(&Impl::run, this);
    } catch (...) {
      release_cuda();
      release_pipeline();
      throw;
    }
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    pending_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    release_pipeline();
    release_cuda();
  }

  void submit(const std::uint8_t * bgra_device, std::size_t bgra_pitch, int width, int height)
  {
    if (bgra_device == nullptr || width != config_.source_width || height != config_.source_height) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || failed_ || (last_selected_ != std::chrono::steady_clock::time_point{} &&
      now - last_selected_ < period_))
    {
      return;
    }
    auto slot = std::find_if(slots_.begin(), slots_.end(), [](const Slot & candidate) {
      return candidate.state == State::kFree;
    });
    if (slot == slots_.end()) {
      // A pending slot has not entered nvJPEG yet.  The private CUDA stream
      // serializes the replacement D2D copy after the old one, so replacing it
      // is safe and keeps the queue latest-wins.
      slot = std::find_if(slots_.begin(), slots_.end(), [](const Slot & candidate) {
        return candidate.state == State::kPending;
      });
      if (slot == slots_.end()) {
        ++dropped_no_slot_;
        return;
      }
      ++dropped_pending_;
    }
    try {
      check_cuda(cudaMemcpy2DAsync(
          slot->bgra, static_cast<std::size_t>(width) * 4, bgra_device, bgra_pitch,
          static_cast<std::size_t>(width) * 4, static_cast<std::size_t>(height),
          cudaMemcpyDeviceToDevice, stream_), "cudaMemcpy2DAsync stream slot");
      check_cuda(cudaEventRecord(slot->ready, stream_), "cudaEventRecord stream slot");
    } catch (const std::exception &) {
      failed_ = true;
      return;
    }
    slot->source_width = width;
    slot->source_height = height;
    slot->capture_time = now;
    slot->state = State::kPending;
    last_selected_ = now;
    ++selected_;
    pending_.notify_one();
  }

private:
  enum class State {kFree, kPending, kEncoding, kInFlight};
  struct Slot
  {
    std::uint8_t * bgra{nullptr};
    std::uint8_t * bgri{nullptr};
    std::uint8_t * host{nullptr};
    cudaEvent_t ready{nullptr};
    int source_width{};
    int source_height{};
    std::chrono::steady_clock::time_point capture_time{};
    State state{State::kFree};
  };
  struct BufferLease
  {
    Impl * owner;
    std::size_t slot_index;
  };

  void validate() const
  {
    if (config_.host.empty() || config_.source_width <= 0 || config_.source_height <= 0 ||
      config_.width <= 0 || config_.height <= 0 || config_.fps <= 0.0 ||
      config_.jpeg_quality < 1 || config_.jpeg_quality > 100 || config_.port < 1 ||
      config_.port > 65535 || config_.max_pending_frames < 1 || config_.max_pending_frames > 3 ||
      config_.mtu < 256 || config_.mtu > 65507)
    {
      throw std::invalid_argument("invalid ground-video configuration");
    }
  }

  std::size_t max_bgra_bytes() const
  {
    return static_cast<std::size_t>(config_.source_width) * config_.source_height * 4;
  }
  std::size_t max_bgri_bytes() const
  {
    return static_cast<std::size_t>(config_.width) * config_.height * 3;
  }

  void create_pipeline()
  {
    GError * error = nullptr;
    // appsrc feeds packed BGR raw video (from the GPU resize kernel, copied to
    // pinned host memory).  videoconvert produces I420, nvvidconv copies it
    // into an NVMM buffer, and nvjpegenc drives the Tegra hardware JPEG
    // encoder on that NVMM buffer -- see the note above libnvjpeg is not used
    // here because it does not work on this device.
    const std::string description = "appsrc name=source is-live=true format=time block=false "
      "do-timestamp=false ! videoconvert ! video/x-raw,format=I420 ! nvvidconv ! "
      "video/x-raw(memory:NVMM),format=I420 ! nvjpegenc quality=" +
      std::to_string(config_.jpeg_quality) + " ! jpegparse ! rtpjpegpay mtu=" +
      std::to_string(config_.mtu) + " ! udpsink host=" + config_.host + " port=" +
      std::to_string(config_.port) + " sync=false async=false";
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error != nullptr || pipeline_ == nullptr) {
      const std::string message = error == nullptr ? "unknown GStreamer error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("could not create ground-video pipeline: " + message);
    }
    appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
    if (appsrc_ == nullptr) {
      throw std::runtime_error("could not find ground-video appsrc element");
    }
    // Caps MUST be set before the pipeline transitions to PLAYING: appsrc
    // negotiates its source pad caps as part of that state change, and if
    // they are still unset at that point the negotiation can fail silently
    // (no error, but zero frames ever reach the sink) or lock in caps that
    // downstream elements reject. Setting them first guarantees appsrc
    // advertises fixed caps from the very first buffer.
    int fps_numerator = 0;
    int fps_denominator = 1;
    gst_util_double_to_fraction(config_.fps, &fps_numerator, &fps_denominator);
    GstCaps * caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT, config_.width,
      "height", G_TYPE_INT, config_.height, "framerate", GST_TYPE_FRACTION, fps_numerator,
      fps_denominator, nullptr);
    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("could not start ground-video pipeline");
    }
  }

  static void release_wrapped_buffer(gpointer data)
  {
    auto * lease = static_cast<BufferLease *>(data);
    auto * owner = lease->owner;
    std::lock_guard<std::mutex> lock(owner->mutex_);
    owner->slots_[lease->slot_index].state = State::kFree;
    delete lease;
    owner->pending_.notify_one();
  }

  void run()
  {
    while (true) {
      std::size_t index = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        pending_.wait(lock, [this] {
          return stopping_ || std::any_of(slots_.begin(), slots_.end(), [](const Slot & slot) {
            return slot.state == State::kPending;
          });
        });
        if (stopping_) {
          return;
        }
        auto slot = std::find_if(slots_.begin(), slots_.end(), [](const Slot & candidate) {
          return candidate.state == State::kPending;
        });
        index = static_cast<std::size_t>(std::distance(slots_.begin(), slot));
        slot->state = State::kEncoding;
      }
      try {
        encode(index);
      } catch (const std::exception &) {
        std::lock_guard<std::mutex> lock(mutex_);
        slots_[index].state = State::kFree;
        failed_ = true;
      }
    }
  }

  void encode(std::size_t index)
  {
    auto & slot = slots_[index];
    check_cuda(cudaEventSynchronize(slot.ready), "cudaEventSynchronize stream slot");
    const dim3 threads(16, 16);
    const dim3 blocks(
      static_cast<unsigned int>((config_.width + threads.x - 1) / threads.x),
      static_cast<unsigned int>((config_.height + threads.y - 1) / threads.y));
    resize_bgra_to_bgri<<<blocks, threads, 0, stream_>>>(
      slot.bgra, static_cast<std::size_t>(slot.source_width) * 4, slot.source_width, slot.source_height,
      slot.bgri, config_.width, config_.height);
    check_cuda(cudaGetLastError(), "resize_bgra_to_bgri");
    // JPEG encoding itself happens downstream in the GStreamer pipeline
    // (videoconvert -> nvvidconv -> nvjpegenc), because libnvjpeg is broken
    // on this device (see note above). This D2H copy of the packed BGR
    // frame is the sole device-to-host transfer in the streaming path.
    const std::size_t bytes = max_bgri_bytes();
    check_cuda(cudaMemcpyAsync(slot.host, slot.bgri, bytes, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync BGR to host");
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize BGR copy");
    auto * lease = new BufferLease{this, index};
    GstBuffer * buffer = gst_buffer_new_wrapped_full(
      GST_MEMORY_FLAG_READONLY, slot.host, bytes, 0, bytes, lease, release_wrapped_buffer);
    GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(slot.capture_time - started_).count());
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
    GST_BUFFER_DURATION(buffer) = static_cast<GstClockTime>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period_).count());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot.state = State::kInFlight;
      ++encoded_;
    }
    if (gst_app_src_push_buffer(appsrc_, buffer) != GST_FLOW_OK) {
      // push_buffer consumes the GstBuffer, so its release callback frees the slot.
      std::lock_guard<std::mutex> lock(mutex_);
      ++network_errors_;
    }
  }

  void release_pipeline()
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsrc_ != nullptr) {
      gst_object_unref(appsrc_);
      appsrc_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  void release_cuda()
  {
    for (auto & slot : slots_) {
      if (slot.ready != nullptr) {cudaEventDestroy(slot.ready);}
      if (slot.host != nullptr) {cudaFreeHost(slot.host);}
      if (slot.bgri != nullptr) {cudaFree(slot.bgri);}
      if (slot.bgra != nullptr) {cudaFree(slot.bgra);}
    }
    if (stream_ != nullptr) {cudaStreamDestroy(stream_);}
  }

  GroundVideoConfig config_;
  std::chrono::duration<double> period_;
  const std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_selected_{};
  std::mutex mutex_;
  std::condition_variable pending_;
  bool stopping_{false};
  bool failed_{false};
  std::thread worker_;
  std::vector<Slot> slots_;
  cudaStream_t stream_{nullptr};
  GstElement * pipeline_{nullptr};
  GstAppSrc * appsrc_{nullptr};
  std::uint64_t selected_{0};
  std::uint64_t encoded_{0};
  std::uint64_t dropped_no_slot_{0};
  std::uint64_t dropped_pending_{0};
  std::uint64_t network_errors_{0};
};

GroundVideoStreamer::GroundVideoStreamer(const GroundVideoConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

GroundVideoStreamer::~GroundVideoStreamer() = default;

void GroundVideoStreamer::submit(
  const std::uint8_t * bgra_device, std::size_t bgra_pitch, int width, int height)
{
  impl_->submit(bgra_device, bgra_pitch, width, height);
}

}  // namespace zed2i_driver
