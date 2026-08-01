#include "zed2i_driver/ground_video_streamer.hpp"

#include <cuda_runtime_api.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <nvjpeg.h>

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

void check_nvjpeg(nvjpegStatus_t result, const char * operation)
{
  if (result != NVJPEG_STATUS_SUCCESS) {
    throw std::runtime_error(std::string(operation) + " failed (nvJPEG status " +
      std::to_string(static_cast<int>(result)) + ")");
  }
}

__global__ void resize_bgra_to_bgri(
  const std::uint8_t * source, std::size_t source_pitch, int source_width, int source_height,
  std::uint8_t * destination, int destination_width, int destination_height,
  bool fov_ellipse_enable, float ellipse_cx, float ellipse_cy, float ellipse_a, float ellipse_b)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= destination_width || y >= destination_height) {
    return;
  }
  const int source_x = x * source_width / destination_width;
  const int source_y = y * source_height / destination_height;
  auto * output = destination + (static_cast<std::size_t>(y) * destination_width + x) * 3;
  if (fov_ellipse_enable) {
    const float dx = (static_cast<float>(source_x) - ellipse_cx) / ellipse_a;
    const float dy = (static_cast<float>(source_y) - ellipse_cy) / ellipse_b;
    if (dx * dx + dy * dy > 1.0F) {
      output[0] = 0;
      output[1] = 0;
      output[2] = 0;
      return;
    }
  }
  const auto * input = source + static_cast<std::size_t>(source_y) * source_pitch + source_x * 4;
  output[0] = input[0];
  output[1] = input[1];
  output[2] = input[2];
}

}  // namespace

class GroundVideoStreamer::Impl
{
public:
  explicit Impl(const GroundVideoConfig & config)
  : config_(config), period_(std::chrono::duration<double>(1.0 / config.fps)),
    ellipse_cx_(static_cast<float>(config.fov_ellipse_cx_ratio * config.source_width)),
    ellipse_cy_(static_cast<float>(config.fov_ellipse_cy_ratio * config.source_height)),
    ellipse_a_(static_cast<float>(std::max(config.fov_ellipse_a_ratio * config.source_width, 1e-6))),
    ellipse_b_(static_cast<float>(std::max(config.fov_ellipse_b_ratio * config.source_height, 1e-6)))
  {
    validate();
    gst_init(nullptr, nullptr);
    create_pipeline();
    try {
      check_cuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreateWithFlags");
      check_nvjpeg(nvjpegCreateSimple(&encoder_), "nvjpegCreateSimple");
      check_nvjpeg(nvjpegEncoderStateCreate(encoder_, &encoder_state_, stream_), "nvjpegEncoderStateCreate");
      check_nvjpeg(nvjpegEncoderParamsCreate(encoder_, &encoder_params_, stream_), "nvjpegEncoderParamsCreate");
      check_nvjpeg(nvjpegEncoderParamsSetQuality(encoder_params_, config_.jpeg_quality, stream_),
        "nvjpegEncoderParamsSetQuality");
      check_nvjpeg(nvjpegEncoderParamsSetOptimizedHuffman(encoder_params_, 1, stream_),
        "nvjpegEncoderParamsSetOptimizedHuffman");
      slots_.resize(static_cast<std::size_t>(config_.max_pending_frames + 1));
      for (auto & slot : slots_) {
        check_cuda(cudaMalloc(&slot.bgra, max_bgra_bytes()), "cudaMalloc BGRA slot");
        check_cuda(cudaMalloc(&slot.bgri, max_bgri_bytes()), "cudaMalloc BGR slot");
        check_cuda(cudaMallocHost(&slot.host, max_jpeg_bytes()), "cudaMallocHost JPEG slot");
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
  std::size_t max_jpeg_bytes() const
  {
    return max_bgri_bytes() + 65536;
  }

  void create_pipeline()
  {
    GError * error = nullptr;
    const std::string description = "appsrc name=source is-live=true format=time block=false "
      "do-timestamp=false ! jpegparse ! rtpjpegpay mtu=" + std::to_string(config_.mtu) +
      " ! udpsink host=" + config_.host + " port=" + std::to_string(config_.port) +
      " sync=false async=false";
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error != nullptr || pipeline_ == nullptr) {
      const std::string message = error == nullptr ? "unknown GStreamer error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("could not create ground-video pipeline: " + message);
    }
    appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
    if (appsrc_ == nullptr || gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("could not start ground-video pipeline");
    }
    GstCaps * caps = gst_caps_new_empty_simple("image/jpeg");
    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);
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
      slot.bgri, config_.width, config_.height,
      config_.fov_ellipse_enable, ellipse_cx_, ellipse_cy_, ellipse_a_, ellipse_b_);
    check_cuda(cudaGetLastError(), "resize_bgra_to_bgri");
    nvjpegImage_t image{};
    image.channel[0] = slot.bgri;
    image.pitch[0] = static_cast<unsigned int>(config_.width * 3);
    check_nvjpeg(nvjpegEncodeImage(
        encoder_, encoder_state_, encoder_params_, &image, NVJPEG_INPUT_BGRI,
        config_.width, config_.height, stream_), "nvjpegEncodeImage");
    std::size_t bytes = 0;
    check_nvjpeg(nvjpegEncodeRetrieveBitstream(encoder_, encoder_state_, nullptr, &bytes, stream_),
      "nvjpegEncodeRetrieveBitstream size");
    if (bytes > max_jpeg_bytes()) {
      std::lock_guard<std::mutex> lock(mutex_);
      slots_[index].state = State::kFree;
      ++dropped_oversize_;
      return;
    }
    // nvJPEG writes the encoded bitstream straight to this pinned host buffer.
    // This is the sole device-to-host transfer in the streaming path.
    check_nvjpeg(nvjpegEncodeRetrieveBitstream(encoder_, encoder_state_, slot.host, &bytes, stream_),
      "nvjpegEncodeRetrieveBitstream data");
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize JPEG");
    auto * lease = new BufferLease{this, index};
    GstBuffer * buffer = gst_buffer_new_wrapped_full(
      GST_MEMORY_FLAG_READONLY, slot.host, max_jpeg_bytes(), 0, bytes, lease, release_wrapped_buffer);
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
    if (encoder_params_ != nullptr) {nvjpegEncoderParamsDestroy(encoder_params_);}
    if (encoder_state_ != nullptr) {nvjpegEncoderStateDestroy(encoder_state_);}
    if (encoder_ != nullptr) {nvjpegDestroy(encoder_);}
    if (stream_ != nullptr) {cudaStreamDestroy(stream_);}
  }

  GroundVideoConfig config_;
  std::chrono::duration<double> period_;
  float ellipse_cx_{0.0F};
  float ellipse_cy_{0.0F};
  float ellipse_a_{0.0F};
  float ellipse_b_{0.0F};
  const std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_selected_{};
  std::mutex mutex_;
  std::condition_variable pending_;
  bool stopping_{false};
  bool failed_{false};
  std::thread worker_;
  std::vector<Slot> slots_;
  cudaStream_t stream_{nullptr};
  nvjpegHandle_t encoder_{nullptr};
  nvjpegEncoderState_t encoder_state_{nullptr};
  nvjpegEncoderParams_t encoder_params_{nullptr};
  GstElement * pipeline_{nullptr};
  GstAppSrc * appsrc_{nullptr};
  std::uint64_t selected_{0};
  std::uint64_t encoded_{0};
  std::uint64_t dropped_no_slot_{0};
  std::uint64_t dropped_pending_{0};
  std::uint64_t dropped_oversize_{0};
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
