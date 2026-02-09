#include <gtest/gtest.h>

#include <legged_locomotion_mpc/constraint/WrenchFrictionConeConstraint.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(WrenchFrictionConeConstraint, loader)
{
  const std::string filePath = meldogConfigFolder + "wrench_friction_constraint_settings.info";
  
  const auto config = loadWrenchFrictionConeConfig(filePath);

  EXPECT_TRUE(config.frictionCoefficient == 0.2);
  EXPECT_TRUE(config.footHalfLengthX == 0.25);
  EXPECT_TRUE(config.footHalfLengthY == 0.4);
}