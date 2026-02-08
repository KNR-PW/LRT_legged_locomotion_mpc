#include <gtest/gtest.h>

#include <legged_locomotion_mpc/constraint/ForceFrictionConeConstraint.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(ForceFrictionConeConstraint, loader)
{
  const std::string filePath = meldogConfigFolder + "force_friction_constraint_settings.info";
  
  const auto config = loadForceFrictionConeConfig(filePath);

  EXPECT_TRUE(config.frictionCoefficient == 0.9);
  EXPECT_TRUE(config.regularization == 0.5);
  EXPECT_TRUE(config.hessianDiagonalShift == 0.1);
}