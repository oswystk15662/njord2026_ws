#include <gtest/gtest.h>

#include "alert_lamp/status_evaluator.hpp"

TEST(AlertLampCommandMapping, AutoNormalBlinkTiming)
{
  const auto display = alert_lamp::StatusEvaluator().displayFor(alert_lamp::AlertState::AUTO_NORMAL,
    1.0F, 1.0F, 0.5F, 0.5F, {});
  EXPECT_EQ(display.color, alert_lamp::LampColor::GREEN);
  EXPECT_EQ(display.pattern, alert_lamp::LampPattern::BLINK);
  EXPECT_FLOAT_EQ(display.period, 1.0F);
  EXPECT_FLOAT_EQ(display.duty_ratio, 0.5F);
  EXPECT_FALSE(display.reason.empty());
}
