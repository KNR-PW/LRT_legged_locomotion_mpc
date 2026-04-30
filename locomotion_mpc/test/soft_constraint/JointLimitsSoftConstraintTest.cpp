#include <gtest/gtest.h>

#include <legged_locomotion_mpc/soft_constraint/JointLimitsSoftConstraint.hpp>

#include "../test/include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(JointLimitsSoftConstraintTest, loader)
{
  const std::string constraintFilePath = meldogConfigFolder + "joint_limits_soft_constraint_settings.info";

  const auto constraintSettings = loadJointLimitsSoftConstraintSettings(constraintFilePath);

  EXPECT_TRUE(constraintSettings.barrierSettings.mu == 0.5);
  EXPECT_TRUE(constraintSettings.barrierSettings.delta == 0.6);
}