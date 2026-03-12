#include <gtest/gtest.h>

#include <legged_locomotion_mpc/soft_constraint/SelfCollisionAvoidanceSoftConstraint.hpp>

#include "../test/include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(SelfCollisionAvoidanceSoftConstraintTest, loader)
{
  const std::string constraintFilePath = meldogConfigFolder + "self_collison_avoidance_soft_constraint_settings.info";

  const auto constraintSettings = loadSelfCollisionAvoidanceSoftConstraintSettings(constraintFilePath);

  EXPECT_TRUE(constraintSettings.barrierSettings.mu == 0.5);
  EXPECT_TRUE(constraintSettings.barrierSettings.delta == 0.6);
}