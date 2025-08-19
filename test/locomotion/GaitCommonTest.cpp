#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;

const scalar_t eps = 1e-6;

TEST(GaitCommonTest, ALL)
{
  const size_t randomValue = std::rand() & ((0x01 << MAX_LEG_NUMBER) - 0x01);
  

  const auto bitSetRandomValue = modeNumber2ContactFlags(randomValue);

  ASSERT_TRUE(randomValue == bitSetRandomValue.to_ulong());
  ASSERT_TRUE(randomValue == contactFlags2ModeNumber(bitSetRandomValue));

  const size_t value = 0x55;
  const auto bistSetValue = modeNumber2ContactFlags(value);
  for(int i = 0; i < 4; ++i)
  {
    ASSERT_TRUE(getContactFlag(bistSetValue, 2 * i) == 0x01);
    ASSERT_TRUE(getContactFlag(bistSetValue, 2 * i + 1) == 0x00);
  }


  size_t currentMode = 0b001101;
  size_t newMode = setContactFlag(currentMode, 0, 0);
  ASSERT_TRUE(newMode == 0b01100);

  newMode = setContactFlag(currentMode, 2, 0);
  ASSERT_TRUE(newMode == 0b01001);

  newMode = setContactFlag(currentMode, 5, 1);
  ASSERT_TRUE(newMode == 0b101101);


  scalar_t fullValue0  = 2.7;
  scalar_t fullValue1  = 3.45;
  scalar_t fullValue2  = 1.0;
  scalar_t fullValue3  = 0.7;

  ASSERT_NEAR(normalizePhase(fullValue0), 0.7, eps);
  ASSERT_NEAR(normalizePhase(fullValue1), 0.45, eps);
  ASSERT_NEAR(normalizePhase(fullValue2), 0.0, eps);
  ASSERT_NEAR(normalizePhase(fullValue3), 0.7, eps);

  scalar_t currentPhase = 0.2;
  scalar_t swingRatio = 0.5;
  scalar_t gaitPeriod = 1.0;
  ASSERT_NEAR(getTimeToNextMode(currentPhase, swingRatio, gaitPeriod), 0.3, eps);

  currentPhase = 0.7;
  swingRatio = 0.5;
  gaitPeriod = 1.0;
  ASSERT_NEAR(getTimeToNextMode(currentPhase, swingRatio, gaitPeriod), 0.3, eps);

  currentPhase = 0.3;
  swingRatio = 0.3;
  gaitPeriod = 2.0;
  ASSERT_NEAR(getTimeToNextMode(currentPhase, swingRatio, gaitPeriod), 1.4, eps);
}