#include <gtest/gtest.h>

#include <legged_locomotion_mpc/soft_constraint/JointTorqueLimitsSoftConstraint.hpp>

#include "../test/include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(JointTorqueLimitsSoftConstraintTest, loader)
{
  const std::string constraintFilePath = meldogConfigFolder + "joint_torque_limits_soft_constraint_settings.info";

  const auto constraintSettings = loadJointTorqueLimitsSoftConstraintSettings(constraintFilePath);

  EXPECT_TRUE(constraintSettings.barrierSettings.mu == 0.5);
  EXPECT_TRUE(constraintSettings.barrierSettings.delta == 0.6);
}