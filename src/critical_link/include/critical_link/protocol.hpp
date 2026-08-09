#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace critical_link
{

constexpr std::array<uint8_t, 4> kFrameMagic{{'N', 'C', 'L', '1'}};
constexpr uint8_t kProtocolVersion = 2;
constexpr size_t kHeaderSize = 34;
constexpr size_t kCrcSize = 4;
constexpr size_t kMaxPayloadSize = 200;
constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;

// These flags express operator intent only.  The vessel-side policy decides
// which configured source may act on that intent; no priority travels on the
// wire.
constexpr uint16_t kFlagControlActive = 1U << 0;
constexpr uint16_t kFlagTakeoverRequest = 1U << 1;

enum class StreamId : uint8_t
{
  kJoy = 1,
  kGroundHeartbeat = 2,
  kLinkProbe = 3,
};

struct Frame
{
  StreamId stream{StreamId::kJoy};
  uint16_t flags{0};
  uint32_t source_id{0};
  uint64_t session_id{0};
  uint32_t sequence{0};
  uint64_t source_monotonic_ms{0};
  std::vector<uint8_t> payload;
};

uint32_t crc32_ieee(const uint8_t * data, size_t size);
std::vector<uint8_t> encode_frame(const Frame & frame);
std::optional<Frame> decode_frame(const uint8_t * data, size_t size);
bool sequence_is_newer(uint32_t candidate, uint32_t reference);

class FrameStreamDecoder
{
public:
  std::vector<Frame> push(const uint8_t * data, size_t size);
  void clear();

private:
  std::vector<uint8_t> buffer_;
};

class SequenceGate
{
public:
  bool accept(uint32_t source_id, uint64_t session_id, StreamId stream, uint32_t sequence);
  std::optional<uint64_t> active_session(uint32_t source_id) const;

private:
  struct StreamState
  {
    bool initialized{false};
    uint32_t sequence{0};
  };

  static size_t stream_index(StreamId stream);

  struct SourceState
  {
    std::optional<uint64_t> active_session;
    std::unordered_set<uint64_t> retired_sessions;
    std::array<StreamState, 4> streams{};
  };

  std::unordered_map<uint32_t, SourceState> sources_;
};

}  // namespace critical_link
