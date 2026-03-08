#include <gtest/gtest.h>

#include <legged_locomotion_mpc/robot_interface/LeggedInterface.hpp>

#include "../test/include/definitions.hpp"

using namespace legged_locomotion_mpc;

TEST(LeggedInterfaceTest, loader) 
{
  const std::string filePath = meldogConfigFolder + "legged_interface_settings.info";

  const auto settings = loadLeggedInterfaceSettings(filePath);

  EXPECT_TRUE(settings.useTrajectoryTrackingCost               == true);
  EXPECT_TRUE(settings.useTerminalTrackingCost                 == true);
  EXPECT_TRUE(settings.useJointTorqueCost                      == false);
  EXPECT_TRUE(settings.useNormalVelocityConstraint             == true);
  EXPECT_TRUE(settings.useForceFrictionConeConstraint          == false);
  EXPECT_TRUE(settings.useZero3DofVelocityConstraint           == true);
  EXPECT_TRUE(settings.useZeroForceConstraint                  == false);
  EXPECT_TRUE(settings.useWrenchFrictionConeConstraint         == true);
  EXPECT_TRUE(settings.useZero6DofVelocityConstraint           == false);
  EXPECT_TRUE(settings.useZeroWrenchConstraint                 == true);
  EXPECT_TRUE(settings.useEndEffectorPlacementSoftConstraint   == false);
  EXPECT_TRUE(settings.useJointLimitsSoftConstraint            == true);
  EXPECT_TRUE(settings.useJointTorqueLimitsSoftConstraint      == false);
  EXPECT_TRUE(settings.useTerrainAvoidanceSoftConstraint       == true);
  EXPECT_TRUE(settings.useSelfCollisionAvoidanceSoftConstraint == false);
}