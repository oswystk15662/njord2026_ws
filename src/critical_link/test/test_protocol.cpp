#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

#include "critical_link/joy_codec.hpp"
#include "critical_link/protocol.hpp"
#include "critical_link/source_arbiter.hpp"

namespace critical_link
{
namespace
{

Frame sample_frame()
{
  Frame frame;
  frame.stream = StreamId::kJoy;
  frame.flags = 3;
  frame.source_id = 7;
  frame.session_id = 0x0123456789ABCDEFULL;
  frame.sequence = 42;
  frame.source_monotonic_ms = 987654321ULL;
  frame.payload = {1, 2, 3, 4, 5};
  return frame;
}

TEST(Protocol, RoundTripsFrame)
{
  const auto source = sample_frame();
  const auto bytes = encode_frame(source);
  ASSERT_LE(bytes.size(), 250U);
  const auto decoded = decode_frame(bytes.data(), bytes.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->stream, source.stream);
  EXPECT_EQ(decoded->flags, source.flags);
  EXPECT_EQ(decoded->source_id, source.source_id);
  EXPECT_EQ(decoded->session_id, source.session_id);
  EXPECT_EQ(decoded->sequence, source.sequence);
  EXPECT_EQ(decoded->source_monotonic_ms, source.source_monotonic_ms);
  EXPECT_EQ(decoded->payload, source.payload);
}

TEST(Protocol, RejectsCorruptedFrame)
{
  auto bytes = encode_frame(sample_frame());
  bytes[critical_link::kHeaderSize + 1U] ^= 0x80U;
  EXPECT_FALSE(decode_frame(bytes.data(), bytes.size()).has_value());
}

TEST(Protocol, StreamDecoderResynchronizesAndHandlesFragments)
{
  const auto frame = sample_frame();
  const auto bytes = encode_frame(frame);
  const std::array<uint8_t, 5> garbage{{9, 8, 'N', 'C', 7}};
  FrameStreamDecoder decoder;
  EXPECT_TRUE(decoder.push(garbage.data(), garbage.size()).empty());
  EXPECT_TRUE(decoder.push(bytes.data(), 11).empty());
  const auto frames = decoder.push(bytes.data() + 11, bytes.size() - 11);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].sequence, frame.sequence);
}

TEST(SequenceGate, TracksSessionsAndSequencesPerSource)
{
  SequenceGate gate;
  EXPECT_TRUE(gate.accept(100, 10, StreamId::kJoy, 100));
  EXPECT_FALSE(gate.accept(100, 10, StreamId::kJoy, 100));
  EXPECT_FALSE(gate.accept(100, 10, StreamId::kJoy, 99));
  EXPECT_TRUE(gate.accept(100, 10, StreamId::kJoy, 101));
  EXPECT_TRUE(gate.accept(100, 10, StreamId::kGroundHeartbeat, 1));
  EXPECT_TRUE(gate.accept(200, 10, StreamId::kJoy, 100));
  EXPECT_TRUE(gate.accept(100, 11, StreamId::kJoy, 1));
  EXPECT_TRUE(gate.accept(200, 10, StreamId::kJoy, 101));
  EXPECT_FALSE(gate.accept(100, 10, StreamId::kJoy, 102));
  EXPECT_EQ(gate.active_session(100), 11U);
  EXPECT_EQ(gate.active_session(200), 10U);
}

TEST(SequenceGate, HandlesSequenceWrap)
{
  SequenceGate gate;
  EXPECT_TRUE(gate.accept(1, 1, StreamId::kJoy, std::numeric_limits<uint32_t>::max()));
  EXPECT_TRUE(gate.accept(1, 1, StreamId::kJoy, 0));
}

TEST(JoyCodec, RoundTripsCompactPayload)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {-1.0F, 0.25F, 1.0F};
  joy.buttons = {0, 1, 0, 1};
  const auto payload = encode_joy_payload(joy);
  ASSERT_TRUE(payload.has_value());
  ASSERT_LE(payload->size(), kMaxPayloadSize);
  const auto decoded = decode_joy_payload(payload->data(), payload->size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->axes, joy.axes);
  EXPECT_EQ(decoded->buttons, joy.buttons);
}

TEST(JoyCodec, RejectsNonFiniteAndMalformedValues)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {std::numeric_limits<float>::quiet_NaN()};
  EXPECT_FALSE(encode_joy_payload(joy).has_value());
  const std::array<uint8_t, 3> invalid_button{{0, 1, 2}};
  EXPECT_FALSE(decode_joy_payload(invalid_button.data(), invalid_button.size()).has_value());
}

sensor_msgs::msg::Joy joy_with_axis(float axis)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {axis};
  return joy;
}

Frame source_frame(uint32_t source_id, uint16_t flags, StreamId stream = StreamId::kJoy)
{
  Frame frame;
  frame.source_id = source_id;
  frame.session_id = 1;
  frame.stream = stream;
  frame.flags = flags;
  return frame;
}

TEST(SourceArbiter, KeepsActiveSourceUntilItIsStaleThenPublishesNeutralBeforeFailover)
{
  SourceArbiter arbiter(
    {{100, "primary", 100}, {200, "backup", 90}},
    {100, 500, 50, 100});
  arbiter.accept(source_frame(100, kFlagControlActive), joy_with_axis(1.0F), 100);

  auto decision = arbiter.tick(120);
  ASSERT_TRUE(decision.selected_source.has_value());
  EXPECT_EQ(*decision.selected_source, 100U);
  EXPECT_EQ(decision.joy.axes[0], 1.0F);

  arbiter.accept(source_frame(200, kFlagControlActive), joy_with_axis(2.0F), 151);
  decision = arbiter.tick(160);
  ASSERT_TRUE(decision.selected_source.has_value());
  EXPECT_EQ(*decision.selected_source, 100U);

  decision = arbiter.tick(201);
  EXPECT_FALSE(decision.selected_source.has_value());
  EXPECT_TRUE(decision.joy.axes.empty());

  decision = arbiter.tick(251);
  ASSERT_TRUE(decision.selected_source.has_value());
  EXPECT_EQ(*decision.selected_source, 200U);
  EXPECT_EQ(decision.joy.axes[0], 2.0F);
}

TEST(SourceArbiter, HigherPrioritySourceNeedsExplicitTakeoverWhileAnotherSourceIsActive)
{
  SourceArbiter arbiter(
    {{100, "primary", 100}, {200, "backup", 90}},
    {200, 500, 50, 100});
  arbiter.accept(source_frame(200, kFlagControlActive), joy_with_axis(2.0F), 100);
  auto decision = arbiter.tick(110);
  ASSERT_TRUE(decision.selected_source.has_value());
  EXPECT_EQ(*decision.selected_source, 200U);

  arbiter.accept(source_frame(100, kFlagControlActive), joy_with_axis(1.0F), 120);
  decision = arbiter.tick(125);
  ASSERT_TRUE(decision.selected_source.has_value());
  EXPECT_EQ(*decision.selected_source, 200U);

  arbiter.accept(
    source_frame(100, kFlagControlActive | kFlagTakeoverRequest), joy_with_axis(1.0F), 130);
  decision = arbiter.tick(130);
  EXPECT_FALSE(decision.selected_source.has_value());
  decision = arbiter.tick(180);
  ASSERT_TRUE(decision.selected_source.has_value());
  EXPECT_EQ(*decision.selected_source, 100U);
}

TEST(SourceArbiter, AggregatesHeartbeatOnlyFromConfiguredSources)
{
  SourceArbiter arbiter({{100, "primary", 100}}, {100, 500, 50, 100});
  arbiter.accept(source_frame(100, 0U, StreamId::kGroundHeartbeat), std::nullopt, 100);
  EXPECT_TRUE(arbiter.tick(550).ground_station_alive);
  EXPECT_FALSE(arbiter.tick(601).ground_station_alive);
}

TEST(SourceArbiter, ParsesStrictSourceSpecifications)
{
  const auto source = parse_source_spec("100|primary_ground|90");
  ASSERT_TRUE(source.has_value());
  EXPECT_EQ(source->id, 100U);
  EXPECT_EQ(source->name, "primary_ground");
  EXPECT_EQ(source->priority, 90U);
  EXPECT_FALSE(parse_source_spec("0|primary|100").has_value());
  EXPECT_FALSE(parse_source_spec("100|primary").has_value());
}

}  // namespace
}  // namespace critical_link
