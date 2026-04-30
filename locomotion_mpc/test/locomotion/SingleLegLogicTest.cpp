//
// Created by rgrandia on 29.04.20.
//

#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/SingleLegLogic.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;

TEST(TestContactTiming, stanceOnly) {
  ContactTiming stanceOnly{timingNaN(), timingNaN()};
  std::vector<ContactTiming> stanceOnlyVector = {stanceOnly};

  ASSERT_FALSE(hasStartTime(stanceOnly));
  ASSERT_FALSE(hasEndTime(stanceOnly));
  ASSERT_FALSE(startsWithSwingPhase(stanceOnlyVector));
  ASSERT_TRUE(startsWithStancePhase(stanceOnlyVector));
  ASSERT_FALSE(endsWithSwingPhase(stanceOnlyVector));
  ASSERT_TRUE(endsWithStancePhase(stanceOnlyVector));
  ASSERT_FALSE(touchesDownAtLeastOnce(stanceOnlyVector));
  ASSERT_FALSE(liftsOffAtLeastOnce(stanceOnlyVector));
}

TEST(TestContactTiming, SwingOnly) {
  std::vector<ContactTiming> swingOnlyVector = {};

  ASSERT_TRUE(startsWithSwingPhase(swingOnlyVector));
  ASSERT_FALSE(startsWithStancePhase(swingOnlyVector));
  ASSERT_TRUE(endsWithSwingPhase(swingOnlyVector));
  ASSERT_FALSE(endsWithStancePhase(swingOnlyVector));
  ASSERT_FALSE(touchesDownAtLeastOnce(swingOnlyVector));
  ASSERT_FALSE(liftsOffAtLeastOnce(swingOnlyVector));
}

TEST(TestContactTiming, SwingToStance) {
  ContactTiming stance{0.0, timingNaN()};
  std::vector<ContactTiming> swingToStanceVector = {stance};

  ASSERT_TRUE(hasStartTime(stance));
  ASSERT_FALSE(hasEndTime(stance));
  ASSERT_TRUE(startsWithSwingPhase(swingToStanceVector));
  ASSERT_FALSE(startsWithStancePhase(swingToStanceVector));
  ASSERT_FALSE(endsWithSwingPhase(swingToStanceVector));
  ASSERT_TRUE(endsWithStancePhase(swingToStanceVector));
  ASSERT_TRUE(touchesDownAtLeastOnce(swingToStanceVector));
  ASSERT_FALSE(liftsOffAtLeastOnce(swingToStanceVector));
}

TEST(TestContactTiming, StanceToSwing) {
  ContactTiming stance{timingNaN(), 0.0};
  std::vector<ContactTiming> stanceToSwingVector = {stance};

  ASSERT_FALSE(hasStartTime(stance));
  ASSERT_TRUE(hasEndTime(stance));
  ASSERT_FALSE(startsWithSwingPhase(stanceToSwingVector));
  ASSERT_TRUE(startsWithStancePhase(stanceToSwingVector));
  ASSERT_TRUE(endsWithSwingPhase(stanceToSwingVector));
  ASSERT_FALSE(endsWithStancePhase(stanceToSwingVector));
  ASSERT_FALSE(touchesDownAtLeastOnce(stanceToSwingVector));
  ASSERT_TRUE(liftsOffAtLeastOnce(stanceToSwingVector));
}
TEST(TestFootPlanner, extractPhases_SinglePhase) {
  std::vector<double> eventTimes = {};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true};
  auto contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_TRUE(std::isnan(contactPhases.front().start));
  ASSERT_TRUE(std::isnan(contactPhases.front().end));

  // Single swing phase
  contactFlags = {false};
  contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_TRUE(contactPhases.empty());
}

TEST(TestFootPlanner, extractPhases_MergePhases) {
  std::vector<double> eventTimes = {0.5};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true, true};
  auto contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_TRUE(std::isnan(contactPhases.front().start));
  ASSERT_TRUE(std::isnan(contactPhases.front().end));

  // Single swing phase
  contactFlags = {false, false};
  contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_TRUE(contactPhases.empty());
}

TEST(TestFootPlanner, extractPhases_singleSwitch) {
  std::vector<double> eventTimes;
  std::vector<bool> contactFlags;

  // Switched phase
  eventTimes = {0.5};
  contactFlags = {false, true};
  auto contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_EQ(contactPhases.size(), 1);
  ASSERT_DOUBLE_EQ(contactPhases.front().start, eventTimes.front());
  ASSERT_TRUE(std::isnan(contactPhases.front().end));

  // change switching direction
  contactFlags = {true, false};
  contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_EQ(contactPhases.size(), 1);
  ASSERT_TRUE(std::isnan(contactPhases.front().start));
  ASSERT_DOUBLE_EQ(contactPhases.front().end, eventTimes.front());
}

TEST(TestFootPlanner, extractPhases_multiSwitch_startContact) {
  std::vector<double> eventTimes;
  std::vector<bool> contactFlags;

  eventTimes = {1.0, 2.0, 3.0, 4.0};
  contactFlags = {true, true, false, false, true};
  auto contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_EQ(contactPhases.size(), 2);
  // contact phase 1
  ASSERT_TRUE(std::isnan(contactPhases[0].start));
  ASSERT_DOUBLE_EQ(contactPhases[0].end, eventTimes[1]);
  // contact phase 2
  ASSERT_DOUBLE_EQ(contactPhases[1].start, eventTimes[3]);
  ASSERT_TRUE(std::isnan(contactPhases[1].end));
}

TEST(TestFootPlanner, extractPhases_multiSwitch_startSwing) {
  std::vector<double> eventTimes;
  std::vector<bool> contactFlags;

  eventTimes = {1.0, 2.0, 3.0, 4.0};
  contactFlags = {false, false, true, true, false};
  auto contactPhases = extractContactTimings(eventTimes, contactFlags);
  ASSERT_EQ(contactPhases.size(), 1);
  // contact phase 1
  ASSERT_DOUBLE_EQ(contactPhases[0].start, eventTimes[1]);
  ASSERT_DOUBLE_EQ(contactPhases[0].end, eventTimes[3]);
}
