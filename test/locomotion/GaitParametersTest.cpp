#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;

TEST(GaitParameters, loaders)
{
  ModelSettings modelSettings;
  modelSettings.baseLinkName =  baseLink;
  modelSettings.contactNames3DoF = meldog3DofContactNames;
  modelSettings.contactNames6DoF = meldog6DofContactNames;
  modelSettings.hipFrameNames = meldogHipNames;

  const std::string gaitFilePath = meldogConfigFolder + "gait_settings.info";

  const auto staticSettings = loadGaitStaticParameters(gaitFilePath, modelSettings);

  EXPECT_TRUE(staticSettings.maximumSteppingFrequency == 2.0);
  EXPECT_TRUE(staticSettings.minimumSteppingFrequency == 0.5);
  EXPECT_TRUE(staticSettings.touchdownWindow          == 0.75);

  const auto dynamicSettings = loadGaitDynamicParameters(gaitFilePath, modelSettings, 
    staticSettings);

  EXPECT_TRUE(dynamicSettings.steppingFrequency == 1.5);
  EXPECT_TRUE(dynamicSettings.swingRatio        == 0.2);

  const std::vector<scalar_t> truePhaseOffsets{0.1, 0.2, 0.3};
  EXPECT_TRUE(dynamicSettings.phaseOffsets == truePhaseOffsets);

}