#include "critical_link/source_arbiter.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace critical_link
{
namespace
{

std::vector<std::string> split(const std::string & text)
{
  std::vector<std::string> fields;
  size_t begin = 0;
  while (true) {
    const size_t delimiter = text.find('|', begin);
    fields.push_back(text.substr(begin, delimiter == std::string::npos ? delimiter : delimiter - begin));
    if (delimiter == std::string::npos) {
      return fields;
    }
    begin = delimiter + 1U;
  }
}

std::optional<uint32_t> parse_u32(const std::string & text)
{
  if (text.empty()) {
    return std::nullopt;
  }
  size_t consumed = 0;
  try {
    const auto value = std::stoull(text, &consumed, 10);
    if (consumed != text.size() || value > std::numeric_limits<uint32_t>::max()) {
      return std::nullopt;
    }
    return static_cast<uint32_t>(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace

struct SourceArbiter::SourceState
{
  SourceConfig config;
  std::optional<sensor_msgs::msg::Joy> joy;
  uint64_t last_command_ms{0};
  uint64_t last_heartbeat_ms{0};
  uint64_t takeover_requested_until_ms{0};
  bool control_active{false};
};

SourceArbiter::~SourceArbiter() = default;

std::optional<SourceConfig> parse_source_spec(const std::string & text)
{
  const auto fields = split(text);
  if (fields.size() != 3U || fields[1].empty()) {
    return std::nullopt;
  }
  const auto id = parse_u32(fields[0]);
  const auto priority = parse_u32(fields[2]);
  if (!id || *id == 0U || !priority) {
    return std::nullopt;
  }
  return SourceConfig{*id, fields[1], *priority};
}

SourceArbiter::SourceArbiter(std::vector<SourceConfig> configs, SourceArbiterConfig config)
: config_(config)
{
  if (configs.empty() || config_.command_timeout_ms == 0U || config_.heartbeat_timeout_ms == 0U ||
    config_.takeover_request_timeout_ms == 0U)
  {
    throw std::invalid_argument("source arbitration requires positive timeouts and at least one source");
  }
  for (const auto & source : configs) {
    if (source.id == 0U || source.name.empty() || known_source(source.id)) {
      throw std::invalid_argument("source arbitration policy has an invalid or duplicate source");
    }
    sources_.push_back(SourceState{source, std::nullopt, 0U, 0U, 0U, false});
  }
}

bool SourceArbiter::known_source(uint32_t source_id) const
{
  return find_source(source_id) != nullptr;
}

const SourceArbiter::SourceState * SourceArbiter::find_source(uint32_t source_id) const
{
  const auto it = std::find_if(sources_.begin(), sources_.end(), [source_id](const auto & source) {
    return source.config.id == source_id;
  });
  return it == sources_.end() ? nullptr : &*it;
}

SourceArbiter::SourceState * SourceArbiter::find_source(uint32_t source_id)
{
  return const_cast<SourceState *>(std::as_const(*this).find_source(source_id));
}

bool SourceArbiter::command_fresh(const SourceState & source, uint64_t now_ms) const
{
  return source.joy.has_value() && now_ms >= source.last_command_ms &&
         now_ms - source.last_command_ms <= config_.command_timeout_ms;
}

bool SourceArbiter::heartbeat_fresh(const SourceState & source, uint64_t now_ms) const
{
  return source.last_heartbeat_ms != 0U && now_ms >= source.last_heartbeat_ms &&
         now_ms - source.last_heartbeat_ms <= config_.heartbeat_timeout_ms;
}

bool SourceArbiter::eligible(const SourceState & source, uint64_t now_ms) const
{
  return source.control_active && command_fresh(source, now_ms);
}

const SourceArbiter::SourceState * SourceArbiter::best_eligible(uint64_t now_ms) const
{
  const SourceState * best = nullptr;
  for (const auto & source : sources_) {
    if (!eligible(source, now_ms)) {
      continue;
    }
    if (best == nullptr || source.config.priority > best->config.priority ||
      (source.config.priority == best->config.priority && source.config.id < best->config.id))
    {
      best = &source;
    }
  }
  return best;
}

void SourceArbiter::accept(
  const Frame & frame, const std::optional<sensor_msgs::msg::Joy> & joy, uint64_t receive_ms)
{
  auto * source = find_source(frame.source_id);
  if (source == nullptr) {
    return;
  }
  if (frame.stream == StreamId::kJoy && joy) {
    source->joy = *joy;
    source->last_command_ms = receive_ms;
    source->control_active = (frame.flags & kFlagControlActive) != 0U;
    if ((frame.flags & kFlagTakeoverRequest) != 0U) {
      source->takeover_requested_until_ms = receive_ms + config_.takeover_request_timeout_ms;
    }
  } else if (frame.stream == StreamId::kGroundHeartbeat) {
    source->last_heartbeat_ms = receive_ms;
  }
}

void SourceArbiter::begin_handover(std::optional<uint32_t> target, uint64_t now_ms)
{
  active_source_.reset();
  pending_source_ = target;
  handover_pending_ = true;
  neutral_until_ms_ = now_ms + config_.neutral_before_handover_ms;
}

ArbitrationDecision SourceArbiter::tick(uint64_t now_ms)
{
  ArbitrationDecision decision;
  decision.ground_station_alive = std::any_of(
    sources_.begin(), sources_.end(), [this, now_ms](const auto & source) {
      return heartbeat_fresh(source, now_ms);
    });

  const auto * candidate = best_eligible(now_ms);
  if (handover_pending_) {
    if (now_ms < neutral_until_ms_) {
      return decision;
    }
    if (pending_source_) {
      const auto * pending = find_source(*pending_source_);
      if (pending != nullptr && eligible(*pending, now_ms)) {
        active_source_ = pending->config.id;
      }
    }
    if (!active_source_ && candidate != nullptr) {
      active_source_ = candidate->config.id;
    }
    pending_source_.reset();
    handover_pending_ = false;
  } else if (!active_source_) {
    if (candidate != nullptr) {
      active_source_ = candidate->config.id;
    }
  } else {
    const auto * active = find_source(*active_source_);
    if (active == nullptr || !eligible(*active, now_ms)) {
      begin_handover(candidate == nullptr ? std::nullopt :
        std::optional<uint32_t>(candidate->config.id), now_ms);
      return decision;
    }
    const bool higher_priority_takeover = candidate != nullptr &&
      candidate->config.id != active->config.id &&
      candidate->config.priority > active->config.priority &&
      candidate->takeover_requested_until_ms > now_ms;
    if (higher_priority_takeover) {
      begin_handover(candidate->config.id, now_ms);
      return decision;
    }
  }

  if (active_source_) {
    const auto * active = find_source(*active_source_);
    if (active != nullptr && eligible(*active, now_ms)) {
      decision.joy = *active->joy;
      decision.selected_source = active->config.id;
    }
  }
  return decision;
}

std::vector<SourceStatus> SourceArbiter::statuses(uint64_t now_ms) const
{
  std::vector<SourceStatus> result;
  result.reserve(sources_.size());
  for (const auto & source : sources_) {
    result.push_back(SourceStatus{
      source.config,
      command_fresh(source, now_ms),
      heartbeat_fresh(source, now_ms),
      source.control_active,
      active_source_ && *active_source_ == source.config.id,
      source.last_command_ms,
      source.last_heartbeat_ms,
    });
  }
  return result;
}

}  // namespace critical_link
