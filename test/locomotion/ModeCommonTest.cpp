#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/ModeCommon.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;

TEST(ModeCommon, ALL)
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
}