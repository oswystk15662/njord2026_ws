#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "critical_link/protocol.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace critical_link
{

// Policy is entirely vessel-owned.  A sender identifies itself with source_id,
// but is never allowed to advertise its own priority in a frame.
struct SourceConfig
{
  uint32_t id{0};
  std::string name;
  uint32_t priority{0};
};

struct SourceArbiterConfig
{
  uint64_t command_timeout_ms{250};
  uint64_t heartbeat_timeout_ms{1500};
  uint64_t neutral_before_handover_ms{50};
  uint64_t takeover_request_timeout_ms{500};
};

// Parse the receiver parameter format: "id|name|priority".
std::optional<SourceConfig> parse_source_spec(const std::string & text);

struct SourceStatus
{
  SourceConfig config;
  bool command_fresh{false};
  bool heartbeat_fresh{false};
  bool control_active{false};
  bool selected{false};
  uint64_t last_command_ms{0};
  uint64_t last_heartbeat_ms{0};
};

struct ArbitrationDecision
{
  sensor_msgs::msg::Joy joy;
  std::optional<uint32_t> selected_source;
  bool ground_station_alive{false};
};

class SourceArbiter
{
public:
  SourceArbiter(std::vector<SourceConfig> sources, SourceArbiterConfig config);
  ~SourceArbiter();

  bool known_source(uint32_t source_id) const;
  void accept(const Frame & frame, const std::optional<sensor_msgs::msg::Joy> & joy,
    uint64_t receive_ms);
  ArbitrationDecision tick(uint64_t now_ms);
  std::vector<SourceStatus> statuses(uint64_t now_ms) const;

private:
  struct SourceState;

  bool command_fresh(const SourceState & source, uint64_t now_ms) const;
  bool heartbeat_fresh(const SourceState & source, uint64_t now_ms) const;
  bool eligible(const SourceState & source, uint64_t now_ms) const;
  const SourceState * best_eligible(uint64_t now_ms) const;
  const SourceState * find_source(uint32_t source_id) const;
  SourceState * find_source(uint32_t source_id);
  void begin_handover(std::optional<uint32_t> target, uint64_t now_ms);

  std::vector<SourceState> sources_;
  SourceArbiterConfig config_;
  std::optional<uint32_t> active_source_;
  std::optional<uint32_t> pending_source_;
  bool handover_pending_{false};
  uint64_t neutral_until_ms_{0};
};

}  // namespace critical_link
