#include <gtest/gtest.h>

#include <legged_locomotion_mpc/soft_constraint/EndEffectorPlacementSoftConstraint.hpp>

#include "../test/include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(EndEffectorPlacementSoftConstraintTest, loader)
{
  const std::string modelFilePath = meldogConfigFolder + "model_settings.info";

  const auto modelSettings = loadModelSettings(modelFilePath);

  const std::string constraintFilePath = meldogConfigFolder + "end_effector_soft_constraint_settings.info";

  const auto constraintSettings = loadEndEffectorPlacementSoftConstraintSettings(
    constraintFilePath, modelSettings);

  const std::vector<scalar_t> trueRadiuses{0.1, 0.2, 0.3, 0.4};

  EXPECT_TRUE(constraintSettings.endEffectorSafetyRadiuses == trueRadiuses);
  EXPECT_TRUE(constraintSettings.barrierSettings.mu == 0.5);
  EXPECT_TRUE(constraintSettings.barrierSettings.delta == 0.6);
}