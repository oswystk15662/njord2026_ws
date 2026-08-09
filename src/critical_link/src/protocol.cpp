#include "critical_link/protocol.hpp"

#include <algorithm>
#include <stdexcept>

namespace critical_link
{
namespace
{

void append_u16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value));
  out.push_back(static_cast<uint8_t>(value >> 8U));
}

void append_u32(std::vector<uint8_t> & out, uint32_t value)
{
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<uint8_t>(value >> shift));
  }
}

void append_u64(std::vector<uint8_t> & out, uint64_t value)
{
  for (unsigned int shift = 0; shift < 64; shift += 8) {
    out.push_back(static_cast<uint8_t>(value >> shift));
  }
}

uint16_t read_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t read_u32(const uint8_t * data)
{
  uint32_t value = 0;
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    value |= static_cast<uint32_t>(*data++) << shift;
  }
  return value;
}

uint64_t read_u64(const uint8_t * data)
{
  uint64_t value = 0;
  for (unsigned int shift = 0; shift < 64; shift += 8) {
    value |= static_cast<uint64_t>(*data++) << shift;
  }
  return value;
}

bool valid_stream(uint8_t value)
{
  return value >= static_cast<uint8_t>(StreamId::kJoy) &&
         value <= static_cast<uint8_t>(StreamId::kLinkProbe);
}

}  // namespace

uint32_t crc32_ieee(const uint8_t * data, size_t size)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

std::vector<uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() > kMaxPayloadSize) {
    throw std::length_error("critical-link payload exceeds ESP-NOW-safe limit");
  }

  std::vector<uint8_t> out;
  out.reserve(kHeaderSize + frame.payload.size() + kCrcSize);
  out.insert(out.end(), kFrameMagic.begin(), kFrameMagic.end());
  out.push_back(kProtocolVersion);
  out.push_back(static_cast<uint8_t>(frame.stream));
  append_u16(out, frame.flags);
  append_u64(out, frame.session_id);
  append_u32(out, frame.sequence);
  append_u64(out, frame.source_monotonic_ms);
  append_u16(out, static_cast<uint16_t>(frame.payload.size()));
  out.insert(out.end(), frame.payload.begin(), frame.payload.end());
  append_u32(out, crc32_ieee(out.data(), out.size()));
  return out;
}

std::optional<Frame> decode_frame(const uint8_t * data, size_t size)
{
  if (data == nullptr || size < kHeaderSize + kCrcSize || size > kMaxFrameSize) {
    return std::nullopt;
  }
  if (!std::equal(kFrameMagic.begin(), kFrameMagic.end(), data) ||
    data[4] != kProtocolVersion || !valid_stream(data[5]))
  {
    return std::nullopt;
  }

  const size_t payload_size = read_u16(data + 28);
  const size_t expected_size = kHeaderSize + payload_size + kCrcSize;
  if (payload_size > kMaxPayloadSize || expected_size != size) {
    return std::nullopt;
  }
  if (read_u32(data + size - kCrcSize) != crc32_ieee(data, size - kCrcSize)) {
    return std::nullopt;
  }

  Frame frame;
  frame.stream = static_cast<StreamId>(data[5]);
  frame.flags = read_u16(data + 6);
  frame.session_id = read_u64(data + 8);
  frame.sequence = read_u32(data + 16);
  frame.source_monotonic_ms = read_u64(data + 20);
  frame.payload.assign(data + kHeaderSize, data + kHeaderSize + payload_size);
  return frame;
}

bool sequence_is_newer(uint32_t candidate, uint32_t reference)
{
  return static_cast<int32_t>(candidate - reference) > 0;
}

std::vector<Frame> FrameStreamDecoder::push(const uint8_t * data, size_t size)
{
  if (data != nullptr && size > 0) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<Frame> frames;
  while (true) {
    const auto magic = std::search(
      buffer_.begin(), buffer_.end(), kFrameMagic.begin(), kFrameMagic.end());
    if (magic == buffer_.end()) {
      const size_t keep = std::min(buffer_.size(), kFrameMagic.size() - 1U);
      buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(keep));
      break;
    }
    buffer_.erase(buffer_.begin(), magic);
    if (buffer_.size() < kHeaderSize) {
      break;
    }
    const size_t payload_size = read_u16(buffer_.data() + 28);
    if (payload_size > kMaxPayloadSize) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const size_t frame_size = kHeaderSize + payload_size + kCrcSize;
    if (buffer_.size() < frame_size) {
      break;
    }
    const auto frame = decode_frame(buffer_.data(), frame_size);
    if (frame) {
      frames.push_back(*frame);
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    } else {
      buffer_.erase(buffer_.begin());
    }
  }
  return frames;
}

void FrameStreamDecoder::clear()
{
  buffer_.clear();
}

size_t SequenceGate::stream_index(StreamId stream)
{
  const auto index = static_cast<size_t>(stream);
  return index < 4 ? index : 0;
}

bool SequenceGate::accept(uint64_t session_id, StreamId stream, uint32_t sequence)
{
  if (retired_sessions_.count(session_id) != 0U) {
    return false;
  }
  if (!active_session_ || *active_session_ != session_id) {
    if (active_session_) {
      retired_sessions_.insert(*active_session_);
    }
    active_session_ = session_id;
    streams_ = {};
  }

  auto & state = streams_[stream_index(stream)];
  if (!state.initialized || sequence_is_newer(sequence, state.sequence)) {
    state.initialized = true;
    state.sequence = sequence;
    return true;
  }
  return false;
}

}  // namespace critical_link
