#include <gtest/gtest.h>
#include <limits>
#include "mobile_openarm_shared_control/preference.hpp"
using mobile_openarm_shared_control::Preference;
using mobile_openarm_shared_control::PreferenceSnapshot;

TEST(Preference, NoInputIsExactlyBaseline)
{
  Preference p(0.5, 2.0);
  EXPECT_DOUBLE_EQ(p.snapshot(0).cost(0.3, 0.6, 400, 800), 0);
  p.receive(1, true, false, 0.3, 0.0, 1);
  EXPECT_DOUBLE_EQ(p.snapshot(1).cost(0, -0.6, 400, 800), 0);
}
TEST(Preference, L1PreferenceRanksDirectionAndForwardSpeed)
{
  Preference p(0.5, 2.0);
  p.receive(1, true, true, 0.3, 0.5, 0);
  auto s = p.snapshot(0);
  EXPECT_DOUBLE_EQ(s.cost(0.3, 0.5, 400, 800), 0);
  EXPECT_NEAR(s.cost(0.2, 0.4, 400, 800), 120, 1e-10);
  EXPECT_GT(s.cost(0.3, -0.5, 400, 800), s.cost(0.3, 0.0, 400, 800));
}
TEST(Preference, ReleaseUsesChosenSpeedAndExpiresDespiteHeartbeats)
{
  Preference p(0.5, 2.0);
  p.receive(1, true, true, 0.3, 0.5, 0);
  p.selected(0.12, 0.1);
  p.receive(1, true, false, 0.3, 0.0, 0.2);
  EXPECT_EQ(p.snapshot(0.2).phase, PreferenceSnapshot::TRANSITION);
  EXPECT_DOUBLE_EQ(p.snapshot(0.2).linear, 0.12);
  EXPECT_DOUBLE_EQ(p.snapshot(0.2).angular, 0.0);
  for (double t = 0.3; t < 2.3; t += 0.1) {p.receive(1, true, false, 0.3, 0, t);}
  EXPECT_EQ(p.snapshot(2.3).phase, PreferenceSnapshot::AUTONOMOUS);
}
TEST(Preference, TimeoutStopAndNewGoalDiscardOldIntent)
{
  Preference p(0.5, 2.0);
  p.receive(1, true, true, 0.3, 0.5, 0);
  p.selected(0.2, 0.1);
  EXPECT_EQ(p.snapshot(0.6).phase, PreferenceSnapshot::AUTONOMOUS);
  p.receive(1, true, true, 0.3, 0.5, 1);
  p.receive(1, false, false, 0.3, 0, 1.1);
  EXPECT_EQ(p.snapshot(1.1).phase, PreferenceSnapshot::AUTONOMOUS);
  p.receive(1, true, true, 0.3, 0.5, 2);
  p.selected(0.2, 2.1);
  p.receive(2, true, false, 0.3, 0, 2.2);
  EXPECT_EQ(p.snapshot(2.2).phase, PreferenceSnapshot::AUTONOMOUS);
}
TEST(Preference, ReleaseAfterRotationGapDoesNotReplayOldSpeed)
{
  Preference p(0.5, 2.0);
  p.receive(1, true, true, 0.3, 0.5, 0);
  p.selected(0.3, 0);
  for (double t = 0.1; t < 1; t += 0.1) {p.receive(1, true, true, 0.3, 0.5, t);}
  p.receive(1, true, false, 0.3, 0, 1);
  EXPECT_EQ(p.snapshot(1).phase, PreferenceSnapshot::AUTONOMOUS);
}
TEST(Preference, ReengageAndInvalidInput)
{
  Preference p(0.5, 2.0);
  p.receive(1, true, true, 0.3, 0.5, 0);
  p.selected(0.2, 0.1);
  p.receive(1, true, false, 0.3, 0, 0.2);
  p.receive(1, true, true, 0.3, -0.5, 0.3);
  EXPECT_EQ(p.snapshot(0.3).phase, PreferenceSnapshot::ASSIST);
  EXPECT_DOUBLE_EQ(p.snapshot(0.3).angular, -0.5);
  p.receive(1, true, true, std::numeric_limits<double>::quiet_NaN(), 0, 0.4);
  EXPECT_EQ(p.snapshot(0.4).phase, PreferenceSnapshot::AUTONOMOUS);
}
